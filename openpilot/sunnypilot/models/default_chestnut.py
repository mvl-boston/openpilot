"""
Runtime resolution and download of the stock chestnut driving model.

Forks ship the big ONNX pointer but often omit the compiled tinygrad PKL chunks
that stock modeld needs. When chestnut hardware is present, this module finds
the matching artifact for the bundled ONNX and downloads it into MODELS_DIR.
"""

from __future__ import annotations

import os
import re
import time
from pathlib import Path

import requests

from openpilot.cereal import custom
from openpilot.common.file_chunker import get_manifest_path
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, chestnut_compiled
from openpilot.sunnypilot.models.helpers import get_selected_bundle, resolve_bundle_by_ref
from openpilot.sunnypilot.models.tinygrad_ref import get_tinygrad_ref

BIG_ONNX_PATH = MODELS_DIR / "big_driving_supercombo.onnx"
CANONICAL_PKL = "big_driving_tinygrad.pkl"
DEFAULT_MODEL_NAME_PATH = Path(__file__).resolve().parent / "model_name.py"
ONNX_HASH_PARAM = "ModelManager_DefaultChestnutOnnxHash"
HF_REPO = os.getenv("SUNNYPILOT_HF_MODEL_REPO", "sunnypilot/sunnypilot_models_v1")
HF_DEFAULTS_PATH = os.getenv("SUNNYPILOT_HF_BIG_DEFAULTS_PATH", "models/defaults/big")
HF_DEFAULTS_URL = f"https://huggingface.co/datasets/{HF_REPO}/resolve/main/{HF_DEFAULTS_PATH}/default_models.json"
FETCH_TIMEOUT = 15
# HF uploads and chestnut compiles often trail ONNX bumps by hours.
RESOLVE_RETRY_BACKOFFS = (300, 600, 900, 3600)
DOWNLOAD_RETRY_BACKOFFS = (60, 300, 900, 3600)


def read_default_big_model_fields() -> dict[str, str]:
  try:
    content = DEFAULT_MODEL_NAME_PATH.read_text()
  except OSError:
    return {}
  fields = {}
  for line in content.splitlines():
    if "=" in line:
      key, val = line.split("=", 1)
      fields[key.strip()] = val.strip().strip('"')
  return fields


def read_bundled_big_onnx_hash() -> str | None:
  """Return the SHA256 of the bundled big ONNX, from LFS pointer or file contents."""
  if not BIG_ONNX_PATH.is_file():
    return None

  try:
    text = BIG_ONNX_PATH.read_text()
  except OSError as e:
    cloudlog.warning(f"Failed to read {BIG_ONNX_PATH}: {e}")
    return None

  if text.startswith("version https://git-lfs.github.com/spec/v1"):
    for line in text.splitlines():
      if line.startswith("oid sha256:"):
        return line.split(":", 1)[1].strip().lower()
    return None

  import hashlib
  digest = hashlib.sha256()
  with open(BIG_ONNX_PATH, "rb") as f:
    while block := f.read(1024 * 1024):
      digest.update(block)
  return digest.hexdigest().lower()


def fetch_hf_defaults() -> dict | None:
  try:
    response = requests.get(HF_DEFAULTS_URL, timeout=FETCH_TIMEOUT)
    response.raise_for_status()
    return response.json()
  except Exception as e:
    cloudlog.warning(f"Failed to fetch HF default chestnut models from {HF_DEFAULTS_URL}: {e}")
    return None


def _warn_tinygrad_ref_mismatch(defaults: dict) -> None:
  expected = get_tinygrad_ref()
  actual = defaults.get("tinygrad_ref")
  if expected and actual and expected != actual:
    cloudlog.warning(f"HF defaults tinygrad_ref {actual} does not match repo {expected}; trying ONNX match anyway")


def _bundle_for_onnx(defaults: dict, onnx_hash: str) -> dict | None:
  for bundle in defaults.get("bundles", []):
    if bundle.get("onnx_sha256", "").lower() == onnx_hash.lower():
      return bundle
  return None


def _bundle_for_ref(defaults: dict, ref: str) -> dict | None:
  for bundle in defaults.get("bundles", []):
    if bundle.get("ref") == ref:
      return bundle
  return None


def _bundle_for_model_name(defaults: dict, model_name: str) -> dict | None:
  if not model_name:
    return None
  pattern = re.compile(model_name, re.IGNORECASE)
  matches = [
    bundle for bundle in defaults.get("bundles", [])
    if pattern.search(bundle.get("display_name", "")) or pattern.search(bundle.get("short_name", ""))
  ]
  if not matches:
    return None
  return max(matches, key=lambda bundle: int(bundle.get("index", 0)))


def _hf_bundle_matches_onnx(bundle: dict | None, onnx_hash: str | None) -> bool:
  if bundle is None:
    return False
  if not onnx_hash:
    return True
  return bundle.get("onnx_sha256", "").lower() == onnx_hash.lower()


def artifact_from_hf_bundle(bundle: dict, canonical_name: str = CANONICAL_PKL) -> custom.ModelManagerSP.Artifact | None:
  models = bundle.get("models") or []
  if not models:
    return None

  artifact_data = models[0].get("artifact") or {}
  chunks_data = artifact_data.get("chunks") or []
  download_uri = artifact_data.get("download_uri") or {}
  pkl_url = download_uri.get("url")
  if not pkl_url or not chunks_data:
    return None

  artifact = custom.ModelManagerSP.Artifact.new_message()
  artifact.fileName = canonical_name
  artifact.downloadUri.uri = pkl_url.rsplit("/", 1)[0] + "/" + canonical_name
  artifact.downloadUri.sha256 = download_uri.get("sha256", "")

  artifact.init('chunks', len(chunks_data))
  for i, chunk_data in enumerate(chunks_data):
    chunk = artifact.chunks[i]
    chunk.fileName = chunk_data.get("file_name", "")
    chunk.sha256 = chunk_data.get("sha256", "")

  return artifact


def artifact_from_manifest_bundle(bundle: custom.ModelManagerSP.ModelBundle,
                                  canonical_name: str = CANONICAL_PKL) -> tuple[custom.ModelManagerSP.Artifact, str] | None:
  drive_model = next((model for model in bundle.models if model.type == "supercombo"), None)
  if drive_model is None or not drive_model.artifact.fileName:
    return None

  source = drive_model.artifact
  artifact = custom.ModelManagerSP.Artifact.new_message()
  artifact.fileName = source.fileName
  artifact.downloadUri.uri = source.downloadUri.uri
  artifact.downloadUri.sha256 = source.downloadUri.sha256
  artifact.init('chunks', len(source.chunks))
  for i, chunk in enumerate(source.chunks):
    copied = artifact.chunks[i]
    copied.fileName = chunk.fileName
    copied.sha256 = chunk.sha256

  return artifact, source.fileName


def promote_to_canonical(dest_dir: str | Path, source_name: str, canonical_name: str = CANONICAL_PKL) -> None:
  if source_name == canonical_name:
    return

  dest = Path(dest_dir)
  for path in sorted(dest.iterdir()):
    if not path.is_file() or not path.name.startswith(source_name):
      continue
    path.rename(dest / path.name.replace(source_name, canonical_name, 1))


def clear_stale_default_chestnut_artifacts(onnx_hash: str | None, params: Params | None = None) -> None:
  """Drop cached default PKL chunks when the bundled ONNX hash changes."""
  if not onnx_hash:
    return

  params = params or Params()
  cached_hash = params.get(ONNX_HASH_PARAM)
  if cached_hash == onnx_hash and chestnut_compiled():
    return

  if cached_hash and cached_hash != onnx_hash:
    cloudlog.warning(f"Bundled big ONNX changed ({cached_hash[:12]} -> {onnx_hash[:12]}); clearing stale default chestnut model")
    base = str(MODELS_DIR / CANONICAL_PKL)
    for path in Path(MODELS_DIR).glob(f"{CANONICAL_PKL}*"):
      if path.is_file():
        path.unlink()
    manifest = get_manifest_path(base)
    if os.path.isfile(manifest):
      os.remove(manifest)

  params.put(ONNX_HASH_PARAM, onnx_hash, block=True)


def resolve_bundle_from_manifest(source_models: dict[str, list[custom.ModelManagerSP.ModelBundle]],
                                 fields: dict[str, str]) -> custom.ModelManagerSP.ModelBundle | None:
  chestnut_bundles = source_models.get("chestnut", [])
  ref = fields.get("DEFAULT_BIG_MODEL_REF", "")
  if ref:
    resolved = resolve_bundle_by_ref(ref, source_models)
    if resolved is not None:
      bundle, source = resolved
      if source == "chestnut":
        return bundle

  model_name = fields.get("DEFAULT_BIG_MODEL", "")
  if not model_name:
    return None

  pattern = re.compile(model_name, re.IGNORECASE)
  matches = [
    bundle for bundle in chestnut_bundles
    if pattern.search(bundle.displayName) or pattern.search(bundle.internalName)
  ]
  if not matches:
    return None
  return max(matches, key=lambda bundle: bundle.index)


def resolve_default_chestnut_artifact(
  source_models: dict[str, list[custom.ModelManagerSP.ModelBundle]],
) -> tuple[custom.ModelManagerSP.Artifact, str | None] | None:
  """
  Resolve the stock chestnut PKL artifact for this fork.

  Returns (artifact, source_name_to_promote) where source_name_to_promote is set
  when the downloaded files must be renamed to the canonical stock path.
  """
  onnx_hash = read_bundled_big_onnx_hash()
  fields = read_default_big_model_fields()
  defaults = fetch_hf_defaults()

  if defaults:
    _warn_tinygrad_ref_mismatch(defaults)

    if onnx_hash:
      bundle = _bundle_for_onnx(defaults, onnx_hash)
      if bundle:
        artifact = artifact_from_hf_bundle(bundle)
        if artifact is not None:
          cloudlog.info(f"Resolved default chestnut model from HF for ONNX {onnx_hash[:12]}")
          return artifact, None
      cloudlog.warning(f"No HF default chestnut model for ONNX {onnx_hash[:12]}")

    model_name = fields.get("DEFAULT_BIG_MODEL", "")
    name_bundle = _bundle_for_model_name(defaults, model_name)
    if name_bundle is not None and _hf_bundle_matches_onnx(name_bundle, onnx_hash):
      artifact = artifact_from_hf_bundle(name_bundle)
      if artifact is not None:
        cloudlog.info(f"Resolved default chestnut model from HF for {model_name}")
        return artifact, None
    elif name_bundle is not None and onnx_hash:
      cloudlog.warning(f"HF bundle for {model_name} does not match bundled ONNX {onnx_hash[:12]}")

  manifest_bundle = resolve_bundle_from_manifest(source_models, fields)
  if manifest_bundle is None:
    return None

  ref = manifest_bundle.ref
  if onnx_hash and defaults:
    hf_bundle = _bundle_for_ref(defaults, ref)
    if not _hf_bundle_matches_onnx(hf_bundle, onnx_hash):
      cloudlog.warning(f"Refusing manifest fallback for ref {ref}: HF ONNX does not match bundled {onnx_hash[:12]}")
      return None
  elif onnx_hash and defaults is None:
    cloudlog.warning("Cannot verify manifest fallback without HF metadata; waiting for HF defaults")
    return None

  parsed = artifact_from_manifest_bundle(manifest_bundle)
  if parsed is None:
    return None

  artifact, original_name = parsed
  promote_name = original_name if original_name != CANONICAL_PKL else None
  cloudlog.info(f"Resolved default chestnut model from manifest ref {ref}")
  return artifact, promote_name


def needs_default_chestnut_download(params: Params | None = None, chestnut_present: bool = False) -> bool:
  if not chestnut_present:
    return False
  if chestnut_compiled():
    return False
  if get_selected_bundle(params, "chestnut") is not None:
    return False
  return True


class DefaultChestnutDownloader:
  """Downloads the stock chestnut model into MODELS_DIR with retry backoff."""

  def __init__(self):
    self._resolve_failures = 0
    self._download_failures = 0
    self._next_attempt = 0.0
    self._in_progress = False

  def should_attempt(self) -> bool:
    return not self._in_progress and time.monotonic() >= self._next_attempt

  def mark_resolve_failure(self) -> None:
    self._resolve_failures += 1
    delay = RESOLVE_RETRY_BACKOFFS[min(self._resolve_failures - 1, len(RESOLVE_RETRY_BACKOFFS) - 1)]
    self._next_attempt = time.monotonic() + delay
    cloudlog.warning(f"Default chestnut model not available yet; retrying in {delay}s")

  def mark_download_failure(self) -> None:
    self._download_failures += 1
    delay = DOWNLOAD_RETRY_BACKOFFS[min(self._download_failures - 1, len(DOWNLOAD_RETRY_BACKOFFS) - 1)]
    self._next_attempt = time.monotonic() + delay
    cloudlog.warning(f"Default chestnut model download failed; retrying in {delay}s")

  def mark_success(self) -> None:
    self._resolve_failures = 0
    self._download_failures = 0
    self._next_attempt = 0.0

  @property
  def in_progress(self) -> bool:
    return self._in_progress

  def run(self, manager, source_models: dict[str, list[custom.ModelManagerSP.ModelBundle]], params: Params) -> bool:
    if self._in_progress:
      return False

    onnx_hash = read_bundled_big_onnx_hash()
    clear_stale_default_chestnut_artifacts(onnx_hash, params)

    resolved = resolve_default_chestnut_artifact(source_models)
    if resolved is None:
      self.mark_resolve_failure()
      return False

    artifact, promote_name = resolved
    self._in_progress = True
    try:
      import asyncio
      asyncio.run(manager._process_artifact(artifact, str(MODELS_DIR)))
      if promote_name:
        promote_to_canonical(MODELS_DIR, promote_name)
      if chestnut_compiled():
        if onnx_hash:
          params.put(ONNX_HASH_PARAM, onnx_hash, block=True)
        cloudlog.info("Default chestnut model is available in MODELS_DIR")
        self.mark_success()
        return True
      raise RuntimeError("download finished but chestnut model is still missing")
    except Exception:
      self.mark_download_failure()
      raise
    finally:
      self._in_progress = False
