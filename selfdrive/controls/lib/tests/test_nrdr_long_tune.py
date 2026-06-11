"""Unit tests for the M9 live-tune transport (nrdr_long_tune.py).

Pure stdlib on purpose: these run on any host (including the Windows dev box) with no
openpilot deps, so the fail-safe behavior is provable before anything touches the car.
"""
import json
import os

import pytest

from openpilot.selfdrive.controls.lib.nrdr_long_tune import (
  LongTune, write_tune, NRDR_LONG_TUNE_PATH, PERSONALITIES,
)


def make_tune(tmp_path, log_fn=None):
  return LongTune(path=str(tmp_path / "nrdr_long_tune.json"), log_fn=log_fn)


def force_refresh(tune):
  # refresh() stats only every REFRESH_INTERVAL frames; jump straight to a check
  tune._frame = 0
  tune.refresh()


def write_raw(tmp_path, text):
  (tmp_path / "nrdr_long_tune.json").write_text(text, encoding="utf-8")


def assert_defaults(tune):
  assert tune.comfort_brake == 2.5
  assert tune.stop_distance == 6.0
  assert tune.low_speed_jerk_scale == 1.0
  assert tune.a_cruise_max_scale == (1.0, 1.0, 1.0, 1.0)
  assert tune.t_follow_offsets == {}
  assert tune.jerk_factors("standard", (1.0, 1.0)) == (1.0, 1.0)
  assert tune.t_follow_offset("relaxed") == 0.0


class TestFailSafe:
  def test_missing_file_is_noop(self, tmp_path):
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert not tune.active
    assert_defaults(tune)

  def test_corrupt_json_falls_back(self, tmp_path):
    write_raw(tmp_path, "{ not json !!!")
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert not tune.active
    assert_defaults(tune)

  def test_half_written_file_falls_back_then_recovers(self, tmp_path):
    write_raw(tmp_path, '{"comfort_brake": 2.')  # torn write
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert_defaults(tune)
    write_raw(tmp_path, '{"comfort_brake": 2.2}')
    os.utime(tmp_path / "nrdr_long_tune.json", ns=(1, 10**15))  # guarantee sig change
    force_refresh(tune)
    assert tune.comfort_brake == 2.2

  def test_top_level_array_rejected(self, tmp_path):
    write_raw(tmp_path, "[1, 2, 3]")
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert_defaults(tune)

  def test_nan_and_inf_rejected(self, tmp_path):
    write_raw(tmp_path, '{"comfort_brake": NaN, "stop_distance": Infinity}')
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.comfort_brake == 2.5
    assert tune.stop_distance == 6.0
    assert tune.active  # file loaded, bad fields individually rejected

  def test_wrong_types_rejected_per_field(self, tmp_path):
    write_raw(tmp_path, json.dumps({
      "comfort_brake": "fast",
      "stop_distance": 5.0,
      "a_cruise_max_scale": [1.1, 1.1],          # wrong length
      "t_follow_offsets": {"standard": "tight"},  # wrong type
      "jerk_factors": {"relaxed": 1.4},           # not an object
    }))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.comfort_brake == 2.5      # rejected
    assert tune.stop_distance == 5.0      # good field still applies
    assert tune.a_cruise_max_scale == (1.0,) * 4
    assert tune.t_follow_offset("standard") == 0.0
    assert tune.jerk_factors("relaxed", (1.0, 1.0)) == (1.0, 1.0)

  def test_file_removed_reverts_to_defaults(self, tmp_path):
    write_raw(tmp_path, '{"comfort_brake": 2.2}')
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.comfort_brake == 2.2
    (tmp_path / "nrdr_long_tune.json").unlink()
    force_refresh(tune)
    assert not tune.active
    assert_defaults(tune)


class TestClamps:
  def test_scalars_clamped(self, tmp_path):
    write_raw(tmp_path, json.dumps({
      "comfort_brake": 0.5,        # below lo 2.0
      "stop_distance": 99.0,       # above hi 7.5
      "low_speed_jerk_scale": 5.0, # above hi 2.0
    }))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.comfort_brake == 2.0
    assert tune.stop_distance == 7.5
    assert tune.low_speed_jerk_scale == 2.0

  def test_a_cruise_scale_clamped_elementwise(self, tmp_path):
    write_raw(tmp_path, json.dumps({"a_cruise_max_scale": [0.1, 1.25, 2.0, 1.0]}))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.a_cruise_max_scale == (0.5, 1.25, 1.5, 1.0)

  def test_t_follow_offsets_clamped(self, tmp_path):
    write_raw(tmp_path, json.dumps({"t_follow_offsets": {"aggressive": -1.0, "relaxed": 1.0}}))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.t_follow_offset("aggressive") == -0.35
    assert tune.t_follow_offset("relaxed") == 0.5
    assert tune.t_follow_offset("standard") == 0.0

  def test_jerk_factors_partial_override(self, tmp_path):
    write_raw(tmp_path, json.dumps({"jerk_factors": {"relaxed": {"a_change": 1.4}}}))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    # a_change overridden, j_ego falls back to the compiled default passed in
    assert tune.jerk_factors("relaxed", (1.0, 1.0)) == (1.4, 1.0)
    assert tune.jerk_factors("aggressive", (0.5, 0.5)) == (0.5, 0.5)

  def test_jerk_factors_clamped(self, tmp_path):
    write_raw(tmp_path, json.dumps(
      {"jerk_factors": {"standard": {"a_change": 0.01, "j_ego": 10.0}}}))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.jerk_factors("standard", (1.0, 1.0)) == (0.2, 3.0)

  def test_unknown_personality_ignored(self, tmp_path):
    write_raw(tmp_path, json.dumps({"t_follow_offsets": {"econ": -0.3}}))
    tune = make_tune(tmp_path)
    force_refresh(tune)
    assert tune.t_follow_offsets == {}


class TestRefreshCadence:
  def test_stats_only_every_interval(self, tmp_path):
    tune = make_tune(tmp_path)
    force_refresh(tune)  # frame 0 -> stat
    write_raw(tmp_path, '{"comfort_brake": 2.2}')
    for _ in range(LongTune.REFRESH_INTERVAL - 1):
      tune.refresh()  # frames 1..19 -> no stat
    assert tune.comfort_brake == 2.5
    tune.refresh()  # frame 20 -> stat
    assert tune.comfort_brake == 2.2

  def test_no_reread_when_unchanged(self, tmp_path):
    write_raw(tmp_path, '{"comfort_brake": 2.2}')
    logs = []
    tune = make_tune(tmp_path, log_fn=logs.append)
    force_refresh(tune)
    n = len(logs)
    force_refresh(tune)
    force_refresh(tune)
    assert len(logs) == n  # same sig -> no re-read, no log spam

  def test_log_dedup(self, tmp_path):
    logs = []
    tune = make_tune(tmp_path, log_fn=logs.append)
    force_refresh(tune)
    assert logs == []  # never-seen file is not "removed"; stays quiet


class TestWriter:
  def test_write_tune_roundtrip(self, tmp_path):
    path = str(tmp_path / "nrdr_long_tune.json")
    write_tune({"comfort_brake": 2.3, "jerk_factors": {"relaxed": {"j_ego": 1.5}}}, path=path)
    tune = LongTune(path=path)
    force_refresh(tune)
    assert tune.active
    assert tune.comfort_brake == 2.3
    assert tune.jerk_factors("relaxed", (1.0, 1.0)) == (1.0, 1.5)

  def test_write_tune_atomic_no_tmp_left(self, tmp_path):
    path = str(tmp_path / "nrdr_long_tune.json")
    write_tune({"comfort_brake": 2.4}, path=path)
    assert not os.path.exists(path + ".tmp")

  def test_default_path_is_data(self):
    assert NRDR_LONG_TUNE_PATH == "/data/nrdr_long_tune.json"

  def test_personalities_match_branch_enum(self):
    # this branch's cereal LongitudinalPersonality: aggressive@0, standard@1, relaxed@2 (no econ)
    assert PERSONALITIES == ("aggressive", "standard", "relaxed")


if __name__ == "__main__":
  raise SystemExit(pytest.main([__file__, "-v"]))
