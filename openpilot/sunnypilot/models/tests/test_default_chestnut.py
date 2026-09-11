"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import os
import tempfile
import typing
import unittest
from pathlib import Path
from unittest import mock

from openpilot.cereal import custom
from openpilot.common.file_chunker import get_chunk_name, get_manifest_path
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.models.default_chestnut import (
  CANONICAL_PKL,
  DefaultChestnutDownloader,
  artifact_from_hf_bundle,
  clear_stale_default_chestnut_artifacts,
  needs_default_chestnut_download,
  promote_to_canonical,
  read_bundled_big_onnx_hash,
  resolve_default_chestnut_artifact,
)


LFS_POINTER = """version https://git-lfs.github.com/spec/v1
oid sha256:1791d5940b2c048d0639813426dd2cf1d6f2a6727ed51e17c8bcea8bbe754123
size 765950064
"""

LEBOWSKI_ONNX = "a501760a9d1d5fef0eab2b8c5d122d06124fc26dc8e0782e0aa94b82a208f0ff"
BMRLNAP_ONNX = LFS_POINTER.split("oid sha256:", maxsplit=1)[1].split(maxsplit=1)[0]


HF_DEFAULTS = {
  "tinygrad_ref": "abc123",
  "bundles": [
    {
      "index": 0,
      "short_name": "LEBOWSKI",
      "display_name": "Lebowski",
      "ref": "lebowski-ref",
      "onnx_sha256": LEBOWSKI_ONNX,
      "models": [{
        "artifact": {
          "file_name": "driving_lebowski_tinygrad.pkl",
          "download_uri": {
            "url": "https://hf.example/lebowski/driving_lebowski_tinygrad.pkl",
            "sha256": "deadbeef",
          },
          "chunks": [
            {"file_name": "driving_lebowski_tinygrad.pkl.chunk01of02", "sha256": "aa"},
            {"file_name": "driving_lebowski_tinygrad.pkl.chunk02of02", "sha256": "bb"},
          ],
        },
      }],
    },
    {
      "index": 1,
      "short_name": "BMRLNAP",
      "display_name": "BMRLNAP Model v4",
      "ref": "bmrlnap-ref",
      "onnx_sha256": BMRLNAP_ONNX,
      "models": [{
        "artifact": {
          "file_name": "big_driving_tinygrad.pkl",
          "download_uri": {
            "url": "https://hf.example/bmrlnap/big_driving_tinygrad.pkl",
            "sha256": "deadbeef",
          },
          "chunks": [
            {"file_name": "big_driving_tinygrad.pkl.chunk01of02", "sha256": "aa"},
            {"file_name": "big_driving_tinygrad.pkl.chunk02of02", "sha256": "bb"},
          ],
        },
      }],
    },
  ],
}


class TestDefaultChestnutHelpers(OpenpilotTestCase):
  def test_read_bundled_big_onnx_hash_from_lfs_pointer(self):
    with tempfile.TemporaryDirectory() as tmp:
      onnx_path = Path(tmp) / "big_driving_supercombo.onnx"
      onnx_path.write_text(LFS_POINTER)
      with mock.patch("openpilot.sunnypilot.models.default_chestnut.BIG_ONNX_PATH", onnx_path):
        assert read_bundled_big_onnx_hash() == BMRLNAP_ONNX

  def test_artifact_from_hf_bundle(self):
    bundle = typing.cast(dict, HF_DEFAULTS["bundles"][1])
    artifact = artifact_from_hf_bundle(bundle)
    assert artifact.fileName == CANONICAL_PKL
    assert artifact.downloadUri.uri.endswith("/big_driving_tinygrad.pkl")
    assert len(artifact.chunks) == 2

  def test_promote_to_canonical(self):
    with tempfile.TemporaryDirectory() as tmp:
      source = "driving_custom_tinygrad.pkl"
      manifest = get_manifest_path(source)
      chunk = get_chunk_name(source, 0, 1)
      Path(tmp, manifest).write_text("1")
      Path(tmp, chunk).write_text("data")

      promote_to_canonical(tmp, source, CANONICAL_PKL)

      assert Path(tmp, get_manifest_path(CANONICAL_PKL)).is_file()
      assert Path(tmp, get_chunk_name(CANONICAL_PKL, 0, 1)).is_file()

  def test_clear_stale_default_chestnut_artifacts(self):
    with tempfile.TemporaryDirectory() as tmp:
      models_dir = Path(tmp)
      manifest = models_dir / get_manifest_path(CANONICAL_PKL)
      chunk = models_dir / get_chunk_name(CANONICAL_PKL, 0, 1)
      manifest.write_text("1")
      chunk.write_text("old")

      params = mock.MagicMock()
      params.get.return_value = LEBOWSKI_ONNX

      with mock.patch("openpilot.sunnypilot.models.default_chestnut.MODELS_DIR", models_dir), \
           mock.patch("openpilot.sunnypilot.models.default_chestnut.chestnut_compiled", return_value=True):
        clear_stale_default_chestnut_artifacts(BMRLNAP_ONNX, params)

      assert not manifest.exists()
      assert not chunk.exists()
      params.put.assert_called()

  def test_needs_default_chestnut_download(self):
    params = mock.MagicMock()
    params.get.return_value = None
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.chestnut_compiled", return_value=False):
      assert needs_default_chestnut_download(params, chestnut_present=True) is True

    params.get.return_value = {"ref": "custom"}
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.chestnut_compiled", return_value=False):
      assert needs_default_chestnut_download(params, chestnut_present=True) is False


class TestResolveDefaultChestnutArtifact(OpenpilotTestCase):
  def test_prefers_hf_onnx_hash_match(self):
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.read_bundled_big_onnx_hash", return_value=BMRLNAP_ONNX), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.fetch_hf_defaults", return_value=HF_DEFAULTS), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.read_default_big_model_fields",
                    return_value={"DEFAULT_BIG_MODEL": "BMRLNAP Model v4", "DEFAULT_BIG_MODEL_REF": "bmrlnap-ref"}):
      resolved = resolve_default_chestnut_artifact({})
      assert resolved is not None
      artifact, promote_name = resolved
      assert artifact.fileName == CANONICAL_PKL
      assert promote_name is None

  def test_hf_name_fallback_requires_onnx_match(self):
    with mock.patch("openpilot.sunnypilot.models.default_chestnut.read_bundled_big_onnx_hash", return_value=BMRLNAP_ONNX), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.fetch_hf_defaults", return_value=HF_DEFAULTS), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.read_default_big_model_fields",
                    return_value={"DEFAULT_BIG_MODEL": "BMRLNAP Model v4"}):
      resolved = resolve_default_chestnut_artifact({})
      assert resolved is not None

  def test_refuses_manifest_when_hf_onnx_mismatches(self):
    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.ref = "lebowski-ref"
    bundle.displayName = "Lebowski"
    bundle.internalName = "LEBOWSKI"
    bundle.index = 0
    model = bundle.models[0]
    model.type = "supercombo"
    model.artifact.fileName = CANONICAL_PKL
    model.artifact.downloadUri.uri = "https://example.com/big_driving_tinygrad.pkl"
    chunk = custom.ModelManagerSP.Chunk.new_message()
    chunk.sha256 = "111"
    model.artifact.chunks = [chunk]

    with mock.patch("openpilot.sunnypilot.models.default_chestnut.read_bundled_big_onnx_hash", return_value=BMRLNAP_ONNX), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.fetch_hf_defaults", return_value=HF_DEFAULTS), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.read_default_big_model_fields",
                    return_value={"DEFAULT_BIG_MODEL": "Lebowski", "DEFAULT_BIG_MODEL_REF": "lebowski-ref"}):
      assert resolve_default_chestnut_artifact({"chestnut": [bundle]}) is None

  def test_allows_manifest_when_hf_confirms_ref(self):
    bundle = custom.ModelManagerSP.ModelBundle.new_message()
    bundle.ref = "bmrlnap-ref"
    bundle.displayName = "BMRLNAP Model v4"
    bundle.internalName = "BMRLNAP"
    bundle.index = 1
    model = bundle.models[0]
    model.type = "supercombo"
    model.artifact.fileName = CANONICAL_PKL
    model.artifact.downloadUri.uri = "https://example.com/big_driving_tinygrad.pkl"
    chunk = custom.ModelManagerSP.Chunk.new_message()
    chunk.sha256 = "111"
    model.artifact.chunks = [chunk]

    with mock.patch("openpilot.sunnypilot.models.default_chestnut.read_bundled_big_onnx_hash", return_value=BMRLNAP_ONNX), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.fetch_hf_defaults", return_value=HF_DEFAULTS), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut.read_default_big_model_fields",
                    return_value={"DEFAULT_BIG_MODEL": "BMRLNAP Model v4", "DEFAULT_BIG_MODEL_REF": "bmrlnap-ref"}), \
         mock.patch("openpilot.sunnypilot.models.default_chestnut._bundle_for_onnx", return_value=None):
      resolved = resolve_default_chestnut_artifact({"chestnut": [bundle]})
      assert resolved is not None


class TestDefaultChestnutDownloader(OpenpilotTestCase):
  def test_run_promotes_manifest_artifacts(self):
    with tempfile.TemporaryDirectory() as tmp:
      downloader = DefaultChestnutDownloader()
      manager = mock.MagicMock()
      params = mock.MagicMock()

      def write_fake_chunks(destination, source):
        with open(os.path.join(destination, get_manifest_path(source)), "w") as f:
          f.write("1")
        with open(os.path.join(destination, get_chunk_name(source, 0, 1)), "w") as f:
          f.write("data")

      async def fake_process(artifact, destination):
        write_fake_chunks(destination, artifact.fileName)

      manager._process_artifact = fake_process
      artifact = custom.ModelManagerSP.Artifact.new_message()
      artifact.fileName = "driving_custom_tinygrad.pkl"
      resolved = (artifact, "driving_custom_tinygrad.pkl")

      with mock.patch("openpilot.sunnypilot.models.default_chestnut.resolve_default_chestnut_artifact", return_value=resolved), \
           mock.patch("openpilot.sunnypilot.models.default_chestnut.clear_stale_default_chestnut_artifacts"), \
           mock.patch("openpilot.sunnypilot.models.default_chestnut.MODELS_DIR", Path(tmp)), \
           mock.patch("openpilot.sunnypilot.models.default_chestnut.chestnut_compiled") as compiled:
        compiled.side_effect = lambda: Path(tmp, get_manifest_path(CANONICAL_PKL)).is_file()
        assert downloader.run(manager, {}, params) is True


if __name__ == "__main__":
  unittest.main()
