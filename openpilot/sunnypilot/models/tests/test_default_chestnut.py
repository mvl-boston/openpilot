"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import hashlib
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld import helpers as modeld_helpers
from openpilot.sunnypilot.models import default_chestnut
from openpilot.sunnypilot.models.default_chestnut import (
  CANONICAL_PKL,
  DefaultChestnutDownloader,
  lfs_url_from_config,
  needs_default_chestnut_download,
  parse_lfs_pointer,
  read_stock_pointers,
  resolve_default_chestnut_artifacts,
  resolve_from_hf,
  resolve_from_lfs,
)
from openpilot.sunnypilot.models.manager import ModelManagerSP

WARP_FILES = [f"big_driving_warp_{size}_tinygrad.pkl" for size in modeld_helpers.BIG_WARP_SIZES]
STOCK_FILES = [CANONICAL_PKL] + WARP_FILES

PKL_OID = "d4c7a3fcfc95813826ca871a80bea18df833aacb2eb6f5333e25a01e65b1a422"
WARP_OIDS = {
  WARP_FILES[0]: "18c994e4500fe9f34daa8f33202a8606d3817e09e2c79ae8f9795e37159b5143",
  WARP_FILES[1]: "e30b5cada2c2a861999de6103c42053fae524850e30d63cd8f4d9e9bdd319e54",
}
TINYGRAD_REF = "92b742f711c7d216fd3ed7a46a30a959e3c43bd4"


def hf_folder(ref: str) -> str:
  return f"models/defaults/big/model-Mountain-Dew-Model-v3-{ref}-123"


def hf_pkl_url(ref: str) -> str:
  return f"https://huggingface.co/datasets/sunnypilot/sunnypilot_models_v1/resolve/main/{hf_folder(ref)}/driving_mountain_dew_model_v3_tinygrad.pkl"


HF_WARP_HASHES = {name: hashlib.sha256(name.encode()).hexdigest() for name in WARP_FILES}

LFS_CONFIG = """[lfs]
\turl = https://huggingface.co/sunnypilot/sunnypilot-lfs.git/info/lfs
\tpushurl = https://huggingface.co/sunnypilot/sunnypilot-lfs.git/info/lfs

\tlocksverify = false
"""


def lfs_pointer(oid: str, size: int) -> str:
  return f"version https://git-lfs.github.com/spec/v1\noid sha256:{oid}\nsize {size}\n"


def hf_bundle(ref: str, display_name: str, onnx_sha256: str | None, tinygrad_ref: str | None, chunked: bool = False,
              build_time: str = "2026-10-21T18:14:25Z") -> dict:
  artifact = {"file_name": "driving_mountain_dew_model_v3_tinygrad.pkl", "download_uri": {"url": hf_pkl_url(ref), "sha256": PKL_OID}}
  if chunked:
    artifact["chunks"] = [{"file_name": "x.chunk01of01", "sha256": "aa"}]
  bundle = {"ref": ref, "short_name": "MOUNTAIN", "display_name": display_name, "index": -1,
            "build_time": build_time, "models": [{"type": "chunked", "artifact": artifact}]}
  if onnx_sha256:
    bundle["onnx_sha256"] = onnx_sha256
  if tinygrad_ref:
    bundle["tinygrad_ref"] = tinygrad_ref
  return bundle


OTHER_TINYGRAD_REF = "69fd4d34d4430036378aba2b43f9d339cdd6ea73"
HF_DEFAULTS = {
  "tinygrad_ref": TINYGRAD_REF,
  "bundles": [
    hf_bundle("old-ref", "Mountain Dew Model v3", "0" * 64, OTHER_TINYGRAD_REF, build_time="2026-10-01T00:00:00Z"),
    # the same pkl uploaded again after a tinygrad bump, with warps compiled against the newer tinygrad
    hf_bundle("afc8e8a1", "Mountain Dew Model v3", PKL_OID, OTHER_TINYGRAD_REF, build_time="2026-10-20T00:00:00Z"),
    hf_bundle("1aa3a430", "Mountain Dew Model v3", PKL_OID, TINYGRAD_REF, build_time="2026-10-21T00:00:00Z"),
    hf_bundle("lebowski", "Lebowski", "1" * 64, TINYGRAD_REF),
  ],
}


class ModelsDirMixin:
  def setUp(self):
    super().setUp()
    self._tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self._tmp.cleanup)
    self.models_dir = Path(self._tmp.name)
    for module in (modeld_helpers, default_chestnut):
      self.enterContext(mock.patch.object(module, "MODELS_DIR", self.models_dir))

  def write_pointers(self) -> None:
    (self.models_dir / CANONICAL_PKL).write_text(lfs_pointer(PKL_OID, 799941965))
    for name, oid in WARP_OIDS.items():
      (self.models_dir / name).write_text(lfs_pointer(oid, 850515))

  def write_real_files(self, names=STOCK_FILES) -> None:
    for name in names:
      (self.models_dir / name).write_bytes(b"\x80\x05" + name.encode())


class TestStockModelDetection(ModelsDirMixin, OpenpilotTestCase):
  def test_parse_lfs_pointer(self):
    assert parse_lfs_pointer(lfs_pointer(PKL_OID, 42)) == (PKL_OID, 42)
    assert parse_lfs_pointer("not a pointer") is None
    assert parse_lfs_pointer("version https://git-lfs.github.com/spec/v1\n") is None

  def test_lfs_url_from_config(self):
    assert lfs_url_from_config(LFS_CONFIG) == "https://huggingface.co/sunnypilot/sunnypilot-lfs.git/info/lfs"
    assert lfs_url_from_config("") is None

  def test_chestnut_compiled_rejects_pointers_and_missing_files(self):
    assert not modeld_helpers.chestnut_compiled()
    self.write_pointers()
    assert not modeld_helpers.chestnut_compiled()
    self.write_real_files(WARP_FILES)
    assert not modeld_helpers.chestnut_compiled()  # pkl is still a pointer
    self.write_real_files()
    assert modeld_helpers.chestnut_compiled()

  def test_read_stock_pointers(self):
    self.write_pointers()
    pointers = read_stock_pointers()
    assert pointers[CANONICAL_PKL] == (PKL_OID, 799941965)
    assert {name: oid for name, (oid, _) in pointers.items() if name != CANONICAL_PKL} == WARP_OIDS

    self.write_real_files([CANONICAL_PKL])
    assert CANONICAL_PKL not in read_stock_pointers()

  def test_needs_default_chestnut_download(self):
    params = mock.MagicMock()
    params.get.return_value = None
    self.write_pointers()
    assert needs_default_chestnut_download(params, chestnut_present=False) is False
    assert needs_default_chestnut_download(params, chestnut_present=True) is True

    with mock.patch("openpilot.sunnypilot.models.default_chestnut.get_selected_bundle", return_value={"ref": "custom"}):
      assert needs_default_chestnut_download(params, chestnut_present=True) is False

    self.write_real_files()
    assert needs_default_chestnut_download(params, chestnut_present=True) is False


class TestResolveFromLfs(ModelsDirMixin, OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.write_pointers()
    self.enterContext(mock.patch.object(default_chestnut, "get_lfs_url", return_value="https://lfs.example/info/lfs"))

  def _batch_response(self, objects):
    response = mock.MagicMock()
    response.json.return_value = {"transfer": "basic", "objects": objects}
    return response

  def test_resolves_every_pointer(self):
    pointers = read_stock_pointers()
    objects = [{"oid": oid, "size": size, "actions": {"download": {"href": f"https://cdn.example/{oid}"}}} for oid, size in pointers.values()]
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.requests.post", return_value=self._batch_response(objects)) as post:
      artifacts = resolve_from_lfs(pointers)

    assert post.call_args.args[0] == "https://lfs.example/info/lfs/objects/batch"
    sent = post.call_args.kwargs["json"]["objects"]
    assert {o["oid"] for o in sent} == {PKL_OID, *WARP_OIDS.values()}

    assert artifacts is not None
    by_name = {a.fileName: a for a in artifacts}
    assert set(by_name) == set(STOCK_FILES)
    assert by_name[CANONICAL_PKL].downloadUri.sha256 == PKL_OID
    assert by_name[CANONICAL_PKL].downloadUri.uri == f"https://cdn.example/{PKL_OID}"
    for name, oid in WARP_OIDS.items():
      assert by_name[name].downloadUri.sha256 == oid

  def test_missing_object_fails_resolution(self):
    pointers = read_stock_pointers()
    objects = [{"oid": PKL_OID, "size": 1, "error": {"code": 404, "message": "Object does not exist"}}]
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.requests.post", return_value=self._batch_response(objects)):
      assert resolve_from_lfs(pointers) is None

  def test_server_error_fails_resolution(self):
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.requests.post", side_effect=OSError("down")):
      assert resolve_from_lfs(read_stock_pointers()) is None


class TestResolveFromHf(ModelsDirMixin, OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.enterContext(mock.patch.object(default_chestnut, "fetch_hf_defaults", return_value=HF_DEFAULTS))
    self.folder_hashes = self.enterContext(mock.patch.object(default_chestnut, "fetch_hf_folder_hashes", return_value=dict(HF_WARP_HASHES)))
    self.enterContext(mock.patch.object(default_chestnut, "get_tinygrad_ref", return_value=TINYGRAD_REF))

  def _check_artifacts(self, artifacts, ref: str = "1aa3a430"):
    assert artifacts is not None
    by_name = {a.fileName: a for a in artifacts}
    assert set(by_name) == set(STOCK_FILES)
    assert by_name[CANONICAL_PKL].downloadUri.uri == hf_pkl_url(ref)
    assert by_name[CANONICAL_PKL].downloadUri.sha256 == PKL_OID
    for name in WARP_FILES:
      assert by_name[name].downloadUri.uri == hf_pkl_url(ref).rsplit("/", 1)[0] + "/" + name
      assert by_name[name].downloadUri.sha256 == HF_WARP_HASHES[name]
    assert self.folder_hashes.call_args.args[0] == hf_folder(ref)

  def test_matches_bundle_by_pkl_hash(self):
    self._check_artifacts(resolve_from_hf(STOCK_FILES, PKL_OID))

  def test_unknown_hash_is_not_resolved(self):
    assert resolve_from_hf(STOCK_FILES, "f" * 64) is None

  def test_hash_match_prefers_our_tinygrad(self):
    # same pkl under two tinygrad refs: take the warps compiled for ours
    with mock.patch.object(default_chestnut, "get_tinygrad_ref", return_value=OTHER_TINYGRAD_REF):
      self._check_artifacts(resolve_from_hf(STOCK_FILES, PKL_OID), ref="afc8e8a1")

  def test_hash_match_tolerates_tinygrad_mismatch(self):
    with mock.patch.object(default_chestnut, "get_tinygrad_ref", return_value="other"):
      self._check_artifacts(resolve_from_hf(STOCK_FILES, PKL_OID))  # newest build wins

  def test_name_fallback_requires_tinygrad_match(self):
    # no pointer (release build without big_* files): match DEFAULT_BIG_MODEL for this tinygrad ref
    with mock.patch.object(default_chestnut, "read_default_big_model_fields", return_value={"DEFAULT_BIG_MODEL": "Mountain Dew Model v3"}):
      self._check_artifacts(resolve_from_hf(STOCK_FILES, None))
      with mock.patch.object(default_chestnut, "get_tinygrad_ref", return_value="unpublished"):
        assert resolve_from_hf(STOCK_FILES, None) is None
    with mock.patch.object(default_chestnut, "read_default_big_model_fields", return_value={"DEFAULT_BIG_MODEL": "Not A Model"}):
      assert resolve_from_hf(STOCK_FILES, None) is None

  def test_only_missing_files_are_requested(self):
    artifacts = resolve_from_hf(WARP_FILES, PKL_OID)
    assert artifacts is not None
    assert {a.fileName for a in artifacts} == set(WARP_FILES)

  def test_rejects_bundle_without_warp_files(self):
    self.folder_hashes.return_value = {}
    assert resolve_from_hf(STOCK_FILES, PKL_OID) is None

  def test_rejects_chunked_bundle(self):
    defaults = {"tinygrad_ref": TINYGRAD_REF, "bundles": [hf_bundle("1aa3a430", "Mountain Dew Model v3", PKL_OID, TINYGRAD_REF, chunked=True)]}
    with mock.patch.object(default_chestnut, "fetch_hf_defaults", return_value=defaults):
      assert resolve_from_hf(STOCK_FILES, PKL_OID) is None


class TestResolveOrder(ModelsDirMixin, OpenpilotTestCase):
  def test_nothing_needed(self):
    self.write_real_files()
    assert resolve_default_chestnut_artifacts() == []

  def test_lfs_first_then_hf(self):
    self.write_pointers()
    lfs_artifacts = [default_chestnut.make_artifact(name, "lfs://x", "aa") for name in STOCK_FILES]
    with mock.patch.object(default_chestnut, "resolve_from_lfs", return_value=lfs_artifacts) as lfs, \
         mock.patch.object(default_chestnut, "resolve_from_hf") as hf:
      assert resolve_default_chestnut_artifacts() is lfs_artifacts
      assert lfs.call_args.args[0][CANONICAL_PKL] == (PKL_OID, 799941965)
      hf.assert_not_called()

    with mock.patch.object(default_chestnut, "resolve_from_lfs", return_value=None), \
         mock.patch.object(default_chestnut, "resolve_from_hf", return_value=None) as hf:
      assert resolve_default_chestnut_artifacts() is None
      assert hf.call_args.args == (STOCK_FILES, PKL_OID)

  def test_without_pointers_goes_straight_to_hf(self):
    # release builds strip big_* files entirely
    with mock.patch.object(default_chestnut, "resolve_from_lfs") as lfs, \
         mock.patch.object(default_chestnut, "resolve_from_hf", return_value=None) as hf:
      resolve_default_chestnut_artifacts()
      lfs.assert_not_called()
      assert hf.call_args.args == (STOCK_FILES, None)


class TestDefaultChestnutDownloader(ModelsDirMixin, OpenpilotTestCase):
  def test_run_stages_then_installs(self):
    self.write_pointers()
    downloader = DefaultChestnutDownloader()
    manager = mock.MagicMock()
    artifacts = [default_chestnut.make_artifact(name, f"https://x/{name}", "aa") for name in STOCK_FILES]

    def fake_download(artifacts, destination):
      assert Path(destination) != self.models_dir
      for artifact in artifacts:
        with open(os.path.join(destination, artifact.fileName), "wb") as f:
          f.write(b"\x80\x05" + artifact.fileName.encode())
        assert modeld_helpers.is_lfs_pointer(self.models_dir / artifact.fileName)  # pointer untouched until the end

    manager.download_background.side_effect = fake_download
    with mock.patch.object(default_chestnut, "resolve_default_chestnut_artifacts", return_value=artifacts):
      assert downloader.run(manager) is True

    assert modeld_helpers.chestnut_compiled()
    assert not (self.models_dir / default_chestnut.STAGING_DIRNAME).exists()
    assert downloader.should_attempt()

  def test_run_backs_off_on_failure(self):
    self.write_pointers()
    downloader = DefaultChestnutDownloader()
    manager = mock.MagicMock()
    manager.download_background.side_effect = OSError("network")
    artifacts = [default_chestnut.make_artifact(CANONICAL_PKL, "https://x", "aa")]

    with mock.patch.object(default_chestnut, "resolve_default_chestnut_artifacts", return_value=artifacts):
      with self.assertRaises(OSError):
        downloader.run(manager)
    assert not downloader.should_attempt()
    assert modeld_helpers.is_lfs_pointer(self.models_dir / CANONICAL_PKL)

    with mock.patch.object(default_chestnut, "resolve_default_chestnut_artifacts", return_value=None):
      downloader = DefaultChestnutDownloader()
      assert downloader.run(manager) is False
      assert not downloader.should_attempt()


class TestManagerBackgroundDownload(OpenpilotTestCase):
  def setUp(self):
    super().setUp()
    self.manager = ModelManagerSP.__new__(ModelManagerSP)
    self.params = self.manager.params = mock.MagicMock()

  def test_background_download_is_cancelled_by_a_selection(self):
    self.params.get.return_value = None
    assert self.manager._download_interrupted()  # a user download with no ref left is cancelled
    self.manager._background_download = True
    assert not self.manager._download_interrupted()
    self.params.get.return_value = b"some-ref"
    assert self.manager._download_interrupted()

  def test_download_background_processes_every_artifact(self):
    processed = []

    async def fake_process(artifact, destination):
      assert self.manager._background_download
      processed.append((artifact.fileName, destination))

    artifacts = [default_chestnut.make_artifact(name, "https://x", "aa") for name in STOCK_FILES]
    with tempfile.TemporaryDirectory() as tmp, mock.patch.object(self.manager, "_process_artifact", fake_process):
      dest = os.path.join(tmp, "staging")
      self.manager.download_background(artifacts, dest)
      assert os.path.isdir(dest)
    assert [name for name, _ in processed] == STOCK_FILES
    assert not self.manager._background_download


if __name__ == "__main__":
  unittest.main()
