"""
Runtime download of the stock chestnut driving model.

Stock modeld runs the big model from LFS-tracked files in MODELS_DIR
(big_driving_tinygrad.pkl plus the big warp pkls). Forks often ship only the
LFS pointers, or release builds strip the big_* files entirely, so a device
with chestnut hardware falls back to the small model forever. When chestnut
is present but those files are not loadable, this module resolves the exact
artifacts for this checkout and downloads them into MODELS_DIR.

Resolution order:
- LFS: the pointers in the checkout name the exact objects; fetch them from the
  LFS server in .lfsconfig (override: SUNNYPILOT_LFS_URL)
- HuggingFace defaults: sunnypilot CI uploads the stock big pkl keyed by its LFS
  sha256 next to the matching warp pkls; matched by hash when a pointer is
  known, otherwise by DEFAULT_BIG_MODEL name plus tinygrad ref
  (overrides: SUNNYPILOT_HF_MODEL_REPO, SUNNYPILOT_HF_BIG_DEFAULTS_PATH)

Downloads land in a staging directory and are moved into MODELS_DIR only once
every file verified, so modeld never sees a partial model.
"""

from __future__ import annotations

import os
import re
import shutil
import time
from pathlib import Path

import requests

from openpilot.cereal import custom
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.helpers import MODELS_DIR, LFS_POINTER_HEADER, big_model_paths, chestnut_compiled, is_lfs_pointer
from openpilot.sunnypilot.models.helpers import get_selected_bundle
from openpilot.sunnypilot.models.tinygrad_ref import get_tinygrad_ref

CANONICAL_PKL = "big_driving_tinygrad.pkl"
STAGING_DIRNAME = ".default_chestnut_staging"
DEFAULT_MODEL_NAME_PATH = Path(__file__).resolve().parent / "model_name.py"
LFS_CONFIG_PATH = Path(BASEDIR) / ".lfsconfig"
LFS_URL_ENV = "SUNNYPILOT_LFS_URL"
HF_REPO = os.getenv("SUNNYPILOT_HF_MODEL_REPO", "sunnypilot/sunnypilot_models_v1")
HF_DEFAULTS_PATH = os.getenv("SUNNYPILOT_HF_BIG_DEFAULTS_PATH", "models/defaults/big")
HF_DEFAULTS_URL = f"https://huggingface.co/datasets/{HF_REPO}/resolve/main/{HF_DEFAULTS_PATH}/default_models.json"
HF_TREE_URL = f"https://huggingface.co/api/datasets/{HF_REPO}/tree/main"
FETCH_TIMEOUT = 15
# HF uploads trail model bumps by hours; LFS objects should be there right away
RESOLVE_RETRY_BACKOFFS = (300, 600, 900, 3600)
DOWNLOAD_RETRY_BACKOFFS = (60, 300, 900, 3600)

Artifact = custom.ModelManagerSP.Artifact


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


def parse_lfs_pointer(text: str) -> tuple[str, int] | None:
  if not text.startswith(LFS_POINTER_HEADER.decode()):
    return None
  oid, size = None, None
  for line in text.splitlines():
    if line.startswith("oid sha256:"):
      oid = line.split(":", 1)[1].strip().lower()
    elif line.startswith("size "):
      size = int(line.split(" ", 1)[1].strip())
  if oid is None or size is None:
    return None
  return oid, size


def read_lfs_pointer(path: Path) -> tuple[str, int] | None:
  if not is_lfs_pointer(path):
    return None
  try:
    return parse_lfs_pointer(path.read_text())
  except (OSError, ValueError):
    return None


def missing_big_model_files() -> list[Path]:
  return [path for path in big_model_paths() if not path.is_file() or is_lfs_pointer(path)]


def read_stock_pointers() -> dict[str, tuple[str, int]]:
  """LFS (oid, size) for every stock big model file that is still a pointer in the checkout."""
  pointers = {}
  for path in big_model_paths():
    pointer = read_lfs_pointer(path)
    if pointer is not None:
      pointers[path.name] = pointer
  return pointers


def read_bundled_big_model_hash() -> str | None:
  pointer = read_lfs_pointer(MODELS_DIR / CANONICAL_PKL)
  return pointer[0] if pointer else None


def make_artifact(file_name: str, url: str, sha256: str) -> Artifact:
  artifact = Artifact.new_message()
  artifact.fileName = file_name
  artifact.downloadUri.uri = url
  artifact.downloadUri.sha256 = sha256
  return artifact


# --- LFS source -------------------------------------------------------------

def lfs_url_from_config(text: str) -> str | None:
  match = re.search(r"^\s*url\s*=\s*(\S+)", text, re.MULTILINE)
  return match.group(1) if match else None


def get_lfs_url() -> str | None:
  if url := os.getenv(LFS_URL_ENV):
    return url
  try:
    return lfs_url_from_config(LFS_CONFIG_PATH.read_text())
  except OSError:
    return None


def resolve_from_lfs(pointers: dict[str, tuple[str, int]]) -> list[Artifact] | None:
  lfs_url = get_lfs_url()
  if not pointers or not lfs_url:
    return None

  headers = {"Accept": "application/vnd.git-lfs+json", "Content-Type": "application/vnd.git-lfs+json"}
  payload = {
    "operation": "download",
    "transfers": ["basic"],
    "objects": [{"oid": oid, "size": size} for oid, size in pointers.values()],
  }
  try:
    response = requests.post(f"{lfs_url.rstrip('/')}/objects/batch", json=payload, headers=headers, timeout=FETCH_TIMEOUT)
    response.raise_for_status()
    objects = {obj["oid"]: obj for obj in response.json().get("objects", [])}
  except Exception as e:
    cloudlog.warning(f"LFS batch request to {lfs_url} failed: {e}")
    return None

  artifacts = []
  for file_name, (oid, _) in pointers.items():
    download = objects.get(oid, {}).get("actions", {}).get("download") or {}
    if not download.get("href") or download.get("header"):
      cloudlog.warning(f"LFS server has no plain download for {file_name} ({oid[:12]}): {objects.get(oid, {}).get('error')}")
      return None
    artifacts.append(make_artifact(file_name, download["href"], oid))
  return artifacts


# --- HuggingFace defaults source --------------------------------------------

def fetch_hf_defaults() -> dict | None:
  try:
    response = requests.get(HF_DEFAULTS_URL, timeout=FETCH_TIMEOUT)
    response.raise_for_status()
    return response.json()
  except Exception as e:
    cloudlog.warning(f"Failed to fetch HF default chestnut models from {HF_DEFAULTS_URL}: {e}")
    return None


def fetch_hf_folder_hashes(folder: str) -> dict[str, str] | None:
  """file name -> sha256 for the LFS files in a dataset folder."""
  try:
    response = requests.get(f"{HF_TREE_URL}/{folder}", timeout=FETCH_TIMEOUT)
    response.raise_for_status()
    entries = response.json()
  except Exception as e:
    cloudlog.warning(f"Failed to list HF folder {folder}: {e}")
    return None
  return {entry["path"].rsplit("/", 1)[-1]: entry["lfs"]["oid"].lower() for entry in entries if entry.get("lfs")}


def _bundle_for_hash(defaults: dict, model_hash: str, tinygrad_ref: str | None) -> dict | None:
  # the same pkl gets re-uploaded per tinygrad bump with freshly compiled warps; prefer our tinygrad
  matches = [bundle for bundle in defaults.get("bundles", []) if bundle.get("onnx_sha256", "").lower() == model_hash]
  if not matches:
    return None
  same_tinygrad = [bundle for bundle in matches if tinygrad_ref and bundle.get("tinygrad_ref") == tinygrad_ref]
  return max(same_tinygrad or matches, key=lambda bundle: bundle.get("build_time", ""))


def _bundle_for_model_name(defaults: dict, model_name: str, tinygrad_ref: str | None) -> dict | None:
  # without a pointer the only compatibility signal is the tinygrad the pkl was compiled against
  if not model_name or not tinygrad_ref:
    return None
  pattern = re.compile(re.escape(model_name), re.IGNORECASE)
  matches = [
    bundle for bundle in defaults.get("bundles", [])
    if bundle.get("tinygrad_ref") == tinygrad_ref
    and (pattern.search(bundle.get("display_name", "")) or pattern.search(bundle.get("short_name", "")))
  ]
  if not matches:
    return None
  return max(matches, key=lambda bundle: bundle.get("build_time", ""))


def _hf_artifact_source(bundle: dict) -> tuple[str, str] | None:
  """(pkl url, sha256) of a defaults bundle, or None if it is not a whole-file artifact."""
  models = bundle.get("models") or []
  if not models:
    return None
  artifact = models[0].get("artifact") or {}
  if artifact.get("chunks"):
    cloudlog.warning(f"HF default bundle {bundle.get('ref')} is chunked; this manager only downloads whole files")
    return None
  download_uri = artifact.get("download_uri") or {}
  if not download_uri.get("url") or not download_uri.get("sha256"):
    return None
  return download_uri["url"], download_uri["sha256"].lower()


def artifacts_from_hf_bundle(bundle: dict, needed: list[str], folder_hashes: dict[str, str]) -> list[Artifact] | None:
  source = _hf_artifact_source(bundle)
  if source is None:
    return None
  pkl_url, pkl_sha = source
  folder_url = pkl_url.rsplit("/", 1)[0]

  artifacts = []
  for file_name in needed:
    if file_name == CANONICAL_PKL:
      artifacts.append(make_artifact(file_name, pkl_url, pkl_sha))
      continue
    sha = folder_hashes.get(file_name)
    if not sha:
      cloudlog.warning(f"HF default bundle {bundle.get('ref')} has no {file_name}")
      return None
    artifacts.append(make_artifact(file_name, f"{folder_url}/{file_name}", sha))
  return artifacts


def resolve_from_hf(needed: list[str], model_hash: str | None) -> list[Artifact] | None:
  defaults = fetch_hf_defaults()
  if not defaults:
    return None

  repo_ref = get_tinygrad_ref()
  if model_hash:
    bundle = _bundle_for_hash(defaults, model_hash, repo_ref)
    if bundle is None:
      cloudlog.warning(f"No HF default chestnut model for pkl {model_hash[:12]}")
      return None
    bundle_ref = bundle.get("tinygrad_ref")
    if repo_ref and bundle_ref and repo_ref != bundle_ref:
      cloudlog.warning(f"HF default chestnut model tinygrad_ref {bundle_ref} differs from repo {repo_ref}; pkl hash matches, using it anyway")
  else:
    model_name = read_default_big_model_fields().get("DEFAULT_BIG_MODEL", "")
    bundle = _bundle_for_model_name(defaults, model_name, repo_ref)
    if bundle is None:
      cloudlog.warning(f"No HF default chestnut model named {model_name!r} for this tinygrad ref")
      return None

  source = _hf_artifact_source(bundle)
  if source is None:
    return None
  folder = source[0].rsplit("/", 1)[0].split("/resolve/main/", 1)[-1]
  folder_hashes = fetch_hf_folder_hashes(folder) or {}
  artifacts = artifacts_from_hf_bundle(bundle, needed, folder_hashes)
  if artifacts:
    cloudlog.info(f"Resolved default chestnut model from HF bundle {bundle.get('ref')} ({bundle.get('display_name')})")
  return artifacts


# --- orchestration ----------------------------------------------------------

def resolve_default_chestnut_artifacts() -> list[Artifact] | None:
  needed = [path.name for path in missing_big_model_files()]
  if not needed:
    return []

  pointers = read_stock_pointers()
  if len(pointers) == len(needed):
    artifacts = resolve_from_lfs(pointers)
    if artifacts:
      cloudlog.info(f"Resolved default chestnut model from LFS pointers: {', '.join(needed)}")
      return artifacts

  return resolve_from_hf(needed, read_bundled_big_model_hash())


def needs_default_chestnut_download(params: Params | None = None, chestnut_present: bool = False) -> bool:
  if not chestnut_present:
    return False
  if chestnut_compiled():
    return False
  if get_selected_bundle(params, "chestnut") is not None:
    return False
  return True


def install_staged_files(staging: Path, file_names: list[str]) -> None:
  for file_name in file_names:
    os.replace(staging / file_name, MODELS_DIR / file_name)
  shutil.rmtree(staging, ignore_errors=True)


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

  def run(self, manager) -> bool:
    if self._in_progress:
      return False

    artifacts = resolve_default_chestnut_artifacts()
    if not artifacts:
      if artifacts is None:
        self.mark_resolve_failure()
      return False

    self._in_progress = True
    staging = MODELS_DIR / STAGING_DIRNAME
    try:
      staging.mkdir(exist_ok=True)
      manager.download_background(artifacts, str(staging))
      install_staged_files(staging, [artifact.fileName for artifact in artifacts])
      if not chestnut_compiled():
        raise RuntimeError("download finished but the chestnut model is still not loadable")
      cloudlog.info("Default chestnut model is available in MODELS_DIR")
      self.mark_success()
      return True
    except Exception:
      self.mark_download_failure()
      raise
    finally:
      self._in_progress = False
