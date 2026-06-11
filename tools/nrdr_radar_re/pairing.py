"""
pairing.py — sweep assembly + track<->truth pairing

Step 1: group FrameRecords into sweeps (one sweep = one burst covering all active slots).
Step 2: decode the current radar fields per sweep (RANGE via b2:b3 on 0x74-tagged header frames,
        LAT_RAW via b4:b5, STRENGTH via b0, CNTR via b7, TRACK_TAG via b1).
Step 3: derive truth channels for each sweep from the decoded fields (for CSV sources
        the truth channels ARE the current decode: dRel, derived vRel, yRel, STRENGTH, CNTR).
Step 4 (optional): if vision-lead columns are present in a supplementary DataFrame, match
        on nearest-in-time with a position-sanity check.

Truth channel registry is pluggable: callers supply a TruthProvider that maps sweep data
to a dict of {channel_name: float}.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from tools.nrdr_radar_re import BOSCH_RADAR_HDR_TAG, BOSCH_RADAR_LAT_SCALE_DEG_PER_LSB
from tools.nrdr_radar_re.ingest import FrameRecord

# Sentinel values
_RAW_UNSET = 0x8000
_RAW_SAT   = 0xFF80
_STR_IDLE  = 0xFE

# DBC-style decode constants (from radar_interface.py / shipped DBC)
# RANGE: raw = (b2 << 8 | b3); scale 0.00357 m/LSB, offset -3.0 m
_RANGE_SCALE  = 0.00357
_RANGE_OFFSET = -3.0

# LAT_RAW: raw = (b4 << 8 | b5); offset -32768 (signed)
_LAT_OFFSET = 32768


@dataclass
class SweepRecord:
    """Decoded data for one complete sweep across all 6 slots."""
    sweep_idx: int
    t_mono: float

    # Per-slot decoded values (NaN if slot not live)
    slot_raw: dict[int, dict[str, object]] = field(default_factory=dict)
    # Accumulated raw bytes per (slot, sub_idx): for bit-window enumeration
    slot_bytes: dict[int, dict[int, bytes]] = field(default_factory=dict)

    # "Truth" channels derived from the current decode (pluggable; see build_truth_channels)
    truth: dict[str, float] = field(default_factory=dict)

    # Derived track-level fields for the best active slot (slot 0 priority)
    dRel: float = float("nan")
    vRel: float = float("nan")   # derived d(dRel)/dt between consecutive sweeps
    yRel: float = float("nan")
    STRENGTH: float = float("nan")
    CNTR: int = -1
    active_slot: int = -1


TruthProvider = Callable[[SweepRecord], dict[str, float]]


def _decode_range_frame(data: bytes) -> dict[str, object] | None:
    """Decode a header sub-frame tagged b1==0x74.  Returns None if not a valid range frame."""
    if len(data) < 6:
        return None
    strength = data[0]
    tag = data[1]
    if tag != BOSCH_RADAR_HDR_TAG:
        return None
    if strength == _STR_IDLE:
        return None
    raw_range = (data[2] << 8) | data[3]
    if raw_range == _RAW_UNSET or raw_range >= _RAW_SAT:
        return None
    raw_lat = (data[4] << 8) | data[5]
    cntr = data[7] if len(data) >= 8 else 0

    dRel = raw_range * _RANGE_SCALE + _RANGE_OFFSET
    lat_signed = raw_lat - _LAT_OFFSET
    # yRel = -dRel * sin(lat_signed * scale * pi/180)
    angle_rad = lat_signed * BOSCH_RADAR_LAT_SCALE_DEG_PER_LSB * math.pi / 180.0
    yRel = -dRel * math.sin(angle_rad)

    return {
        "strength": strength,
        "tag": tag,
        "raw_range": raw_range,
        "raw_lat": raw_lat,
        "lat_signed": lat_signed,
        "dRel": dRel,
        "yRel": yRel,
        "cntr": cntr,
    }


def assemble_sweeps(frames: list[FrameRecord]) -> list[SweepRecord]:
    """
    Group frames into sweeps.  A new sweep starts on each arrival of slot=0, sub_idx=0 (addr 0x280).
    Returns SweepRecord list in temporal order.
    """
    sweeps: list[SweepRecord] = []
    cur_frames: list[FrameRecord] = []

    def _flush(frames_in: list[FrameRecord], idx: int) -> SweepRecord | None:
        if not frames_in:
            return None
        t = frames_in[0].t_mono
        sr = SweepRecord(sweep_idx=idx, t_mono=t)
        # collect bytes per (slot, sub_idx)
        for fr in frames_in:
            sr.slot_bytes.setdefault(fr.slot, {})[fr.sub_idx] = fr.data
        # decode each slot's header sub-frame (sub_idx=0 carries range on tag 0x74)
        for slot, subs in sr.slot_bytes.items():
            hdr_data = subs.get(0, b"")
            decoded = _decode_range_frame(hdr_data)
            if decoded is not None:
                sr.slot_raw[slot] = decoded
        # pick best active slot (lowest slot index)
        for slot in range(6):
            if slot in sr.slot_raw:
                d = sr.slot_raw[slot]
                sr.dRel = float(d["dRel"])
                sr.yRel = float(d["yRel"])
                sr.STRENGTH = float(d["strength"])
                sr.CNTR = int(d["cntr"])
                sr.active_slot = slot
                break
        return sr

    sweep_idx = 0
    for fr in frames:
        # new sweep boundary: slot 0, sub_idx 0
        if fr.slot == 0 and fr.sub_idx == 0 and cur_frames:
            s = _flush(cur_frames, sweep_idx)
            if s:
                sweeps.append(s)
            sweep_idx += 1
            cur_frames = []
        cur_frames.append(fr)

    if cur_frames:
        s = _flush(cur_frames, sweep_idx)
        if s:
            sweeps.append(s)

    return sweeps


def derive_vrel(sweeps: list[SweepRecord], vrel_max: float = 100.0,
                dt_max_s: float = 0.5) -> None:
    """Compute derived vRel in-place via d(dRel)/dt between consecutive active sweeps, per-slot."""
    # per-slot last (dRel, t_mono) for derivative
    last: dict[int, tuple[float, float]] = {}
    for sr in sweeps:
        if sr.active_slot < 0:
            continue
        slot = sr.active_slot
        dRel = sr.dRel
        t = sr.t_mono
        if slot in last:
            prev_dRel, prev_t = last[slot]
            dt = t - prev_t
            if 0 < dt <= dt_max_s:
                vr = (dRel - prev_dRel) / dt
                if abs(vr) <= vrel_max:
                    sr.vRel = vr
        last[slot] = (dRel, t)


def build_csv_truth_channels(sr: SweepRecord) -> dict[str, float]:
    """
    Default truth provider for CSV captures (no external vision lead).
    Truth = the current decode's own fields: dRel, vRel, yRel, STRENGTH, CNTR.
    """
    return {
        "dRel": sr.dRel,
        "vRel": sr.vRel,
        "yRel": sr.yRel,
        "STRENGTH": sr.STRENGTH,
        "CNTR": float(sr.CNTR),
    }


def attach_truth(sweeps: list[SweepRecord],
                 provider: TruthProvider = build_csv_truth_channels) -> None:
    """Attach truth channels to each sweep in-place."""
    for sr in sweeps:
        sr.truth = provider(sr)


def build_matched_table(sweeps: list[SweepRecord]) -> list[dict]:
    """
    Build the matched track<->truth pair table.
    For CSV sources (truth = current decode), every active sweep is a row.
    Returns list of dicts: {sweep_idx, t_mono, slot, raw bytes per sub_idx, dRel, yRel,
    vRel, STRENGTH, CNTR, truth channels...}
    """
    rows = []
    for sr in sweeps:
        if sr.active_slot < 0:
            continue
        slot = sr.active_slot
        subs = sr.slot_bytes.get(slot, {})
        row: dict = {
            "sweep_idx": sr.sweep_idx,
            "t_mono": sr.t_mono,
            "slot": slot,
            "dRel": sr.dRel,
            "yRel": sr.yRel,
            "vRel": sr.vRel,
            "STRENGTH": sr.STRENGTH,
            "CNTR": sr.CNTR,
        }
        # embed raw bytes for each sub-frame
        for sub_idx in range(4):
            row[f"bytes_{sub_idx}"] = subs.get(sub_idx, b"")
        row.update(sr.truth)
        rows.append(row)
    return rows
