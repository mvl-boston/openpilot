"""
enums.py — state-field enum detector

For a sequence of raw values (one per sweep), detect whether the field behaves like
a finite-state machine:
  - distinct value count (cardinality)
  - dwell-time distribution (how many sweeps before a transition)
  - transition matrix (from_state -> to_state counts)
  - correlation with track lifecycle events (birth = first non-nan; death = last non-nan;
    break = NaN gap in the middle of an active run)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from collections import Counter, defaultdict

import numpy as np


@dataclass
class EnumDetectorResult:
    """Results of the enum detector for one (sub_idx, bit_window, coding)."""
    sub_idx: int
    bit_start: int
    n_bits: int
    coding: str
    channel_label: str = ""

    n_total: int = 0
    n_valid: int = 0
    n_distinct: int = 0
    values: list = field(default_factory=list)  # sorted distinct values

    # dwell: list of consecutive run lengths (in sweeps)
    dwell_lengths: list[int] = field(default_factory=list)
    dwell_mean: float = float("nan")
    dwell_p50: float = float("nan")
    dwell_p95: float = float("nan")

    # transition matrix: dict[(from_val, to_val)] -> count
    transition_matrix: dict = field(default_factory=dict)

    # lifecycle event correlation
    birth_state_dist: dict = field(default_factory=dict)   # state at birth (sweep 0 of a run)
    death_state_dist: dict = field(default_factory=dict)   # state at death (last sweep of a run)

    is_likely_enum: bool = False
    reason: str = ""


def detect_enum(values_seq: list[float | None],
                sub_idx: int,
                bit_start: int,
                n_bits: int,
                coding: str,
                enum_max_cardinality: int = 16,
                label: str = "") -> EnumDetectorResult:
    """
    Run enum detection on a time series of extracted field values.

    values_seq: list of float or None (None = missing/sentinel frame)
    enum_max_cardinality: if distinct count > this, field is probably not an enum.
    """
    res = EnumDetectorResult(sub_idx=sub_idx, bit_start=bit_start, n_bits=n_bits,
                              coding=coding, channel_label=label)
    res.n_total = len(values_seq)

    valid = [(i, v) for i, v in enumerate(values_seq) if v is not None and math.isfinite(float(v))]
    res.n_valid = len(valid)
    if res.n_valid < 4:
        res.reason = "too few valid samples"
        return res

    float_vals = [float(v) for _, v in valid]
    # round to nearest integer for enum analysis
    int_vals = [round(v) for v in float_vals]
    counts = Counter(int_vals)
    res.n_distinct = len(counts)
    res.values = sorted(counts.keys())

    if res.n_distinct > enum_max_cardinality:
        res.reason = f"cardinality {res.n_distinct} > {enum_max_cardinality}: not enum"
        return res

    # dwell analysis: run-length encoding of the integer sequence
    dwells: list[int] = []
    cur_val = int_vals[0]
    cur_run = 1
    for v in int_vals[1:]:
        if v == cur_val:
            cur_run += 1
        else:
            dwells.append(cur_run)
            cur_val = v
            cur_run = 1
    dwells.append(cur_run)
    res.dwell_lengths = dwells

    arr = np.array(dwells, dtype=float)
    res.dwell_mean = float(arr.mean())
    res.dwell_p50 = float(np.percentile(arr, 50))
    res.dwell_p95 = float(np.percentile(arr, 95))

    # transition matrix
    tm: dict[tuple[int, int], int] = defaultdict(int)
    for a, b in zip(int_vals, int_vals[1:]):
        if a != b:
            tm[(a, b)] += 1
    res.transition_matrix = dict(tm)

    # lifecycle correlation: birth = first valid sweep index; death = last valid sweep index
    # break = gap (missing sample) between first and last valid
    indices = [i for i, _ in valid]
    birth_idx = indices[0]
    death_idx = indices[-1]
    if len(indices) > 1:
        expected_consecutive = set(range(birth_idx, death_idx + 1))
        actual_set = set(indices)
        breaks = expected_consecutive - actual_set

    # state at birth and death
    state_at = {i: v for i, v in zip(indices, int_vals)}
    res.birth_state_dist = Counter([state_at[birth_idx]])
    res.death_state_dist = Counter([state_at[death_idx]])

    # heuristic: likely enum if cardinality is low and mean dwell >= 1
    res.is_likely_enum = (res.n_distinct <= enum_max_cardinality and
                          res.n_distinct >= 2 and
                          res.dwell_mean >= 1.0)
    res.reason = (f"cardinality={res.n_distinct}, dwell_mean={res.dwell_mean:.1f}, "
                  f"dwell_p95={res.dwell_p95:.0f}")

    return res


def detect_all_enums(rows: list[dict],
                     frame_len: int = 8,
                     enum_max_cardinality: int = 16) -> list[EnumDetectorResult]:
    """
    Run enum detection on all byte-aligned fields across all sub-frames.
    Only byte-aligned 8-bit windows under unsigned coding (enum states are raw bytes).
    """
    from tools.nrdr_radar_re.sweep import _extract_window

    results: list[EnumDetectorResult] = []
    n = len(rows)

    for sub_idx in range(4):
        for byte_off in range(frame_len):
            bit_start = byte_off * 8
            vals: list[float | None] = []
            for row in rows:
                data = row.get(f"bytes_{sub_idx}", b"")
                v = _extract_window(data, bit_start, 8, "unsigned")
                vals.append(v)
            res = detect_enum(vals, sub_idx=sub_idx, bit_start=bit_start,
                               n_bits=8, coding="unsigned",
                               enum_max_cardinality=enum_max_cardinality,
                               label=f"s{sub_idx}_b{byte_off}")
            if res.is_likely_enum:
                results.append(res)

    results.sort(key=lambda r: r.n_distinct)
    return results
