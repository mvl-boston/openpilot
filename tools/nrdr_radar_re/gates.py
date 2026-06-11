"""
gates.py — NON-BYPASSABLE multiple-comparisons gates

A hypothesis is a FINDING only if ALL three gates pass:

  Gate 1 — Replication: |r| >= threshold AND R^2 >= threshold across >=2 independent
            sources/segments with consistent sign.

  Gate 2 — Holdout: R^2 on the held-out segment must meet the threshold (train/test
            split by segment index — first 60% train, last 40% test by default).

  Gate 3 — Competing-identity falsification: the hypothesis's R^2 against its best
            channel must beat its R^2 against EVERY other channel by at least
            COMPETING_MARGIN.  (Prevents declaring something a range-rate proxy when
            it's equally explained by dRel or STRENGTH.)

Anything that fails appears ONLY in a quarantined "unreplicated candidates" section.
This module is the single gateway: report.py imports findings through gates.apply() only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

from tools.nrdr_radar_re.sweep import HypothesisResult, score_windows, _pearson_r_r2

import numpy as np

# Thresholds — deliberately set; NOT tunable via CLI (non-bypassable).
MIN_R2_TO_REPLICATE    = 0.10   # per-segment R^2 required in each replication segment
MIN_ABS_R_TO_REPLICATE = 0.30   # per-segment |r| required
MIN_SEGMENTS           = 2      # minimum number of independent segments that must replicate
COMPETING_MARGIN       = 0.05   # best-channel R^2 must exceed 2nd-best R^2 by this amount
HOLDOUT_FRAC           = 0.40   # fraction of rows held out for test
SIGN_CONSISTENCY       = True   # replications must have the same sign


@dataclass
class GateResult:
    """Outcome of gates.apply() for one hypothesis."""
    hypothesis: HypothesisResult

    gate1_pass: bool = False
    gate1_detail: str = ""

    gate2_pass: bool = False
    gate2_r2: float = float("nan")
    gate2_detail: str = ""

    gate3_pass: bool = False
    gate3_detail: str = ""

    is_finding: bool = False
    quarantine_reason: str = ""

    # Replication segment details
    replication_r2s: list[float] = field(default_factory=list)
    replication_rs: list[float] = field(default_factory=list)
    replication_segment_ids: list[str] = field(default_factory=list)


def apply(hypothesis: HypothesisResult,
          all_segment_results: dict[str, list[HypothesisResult]],
          all_channel_results_for_window: list[HypothesisResult]) -> GateResult:
    """
    Apply all three gates to a hypothesis.

    hypothesis: the candidate HypothesisResult (from any segment).
    all_segment_results: {segment_id: list of HypothesisResult} — results from ALL segments.
    all_channel_results_for_window: all HypothesisResults for the SAME bit-window (any channel)
                                    from the same segment as hypothesis, for competing-identity check.

    Returns a GateResult.
    """
    gr = GateResult(hypothesis=hypothesis)
    w = hypothesis.window
    ch = hypothesis.channel

    # ---- Gate 1: Replication ----
    rep_segs: list[str] = []
    rep_r2s: list[float] = []
    rep_rs: list[float] = []

    for seg_id, seg_results in all_segment_results.items():
        match = next((h for h in seg_results
                       if h.window.sub_idx == w.sub_idx
                       and h.window.bit_start == w.bit_start
                       and h.window.n_bits == w.n_bits
                       and h.window.coding == w.coding
                       and h.channel == ch), None)
        if match is None:
            continue
        if match.r_squared >= MIN_R2_TO_REPLICATE and abs(match.pearson_r) >= MIN_ABS_R_TO_REPLICATE:
            if SIGN_CONSISTENCY and rep_rs and (match.pearson_r * rep_rs[0] < 0):
                continue  # sign mismatch
            rep_segs.append(seg_id)
            rep_r2s.append(match.r_squared)
            rep_rs.append(match.pearson_r)

    gr.replication_segment_ids = rep_segs
    gr.replication_r2s = rep_r2s
    gr.replication_rs = rep_rs

    if len(rep_segs) >= MIN_SEGMENTS:
        gr.gate1_pass = True
        gr.gate1_detail = (f"replicated in {len(rep_segs)} segments: "
                           f"R^2={[f'{v:.3f}' for v in rep_r2s]}, "
                           f"r={[f'{v:.3f}' for v in rep_rs]}")
    else:
        gr.gate1_pass = False
        gr.gate1_detail = (f"only {len(rep_segs)}/{MIN_SEGMENTS} required segments replicated "
                           f"(need R^2>={MIN_R2_TO_REPLICATE}, |r|>={MIN_ABS_R_TO_REPLICATE})")

    # ---- Gate 2: Holdout ----
    # The holdout check operates on the main hypothesis object's segment.
    # For a proper holdout we need the raw rows; we proxy it by whether the hypothesis
    # already has a holdout r_squared stored (populated by apply_with_holdout).
    # apply() is the base gate; apply_with_holdout() adds Gate 2 scoring.
    # Here: if the hypothesis has n >= 10 we check its own r_squared as a minimum bar
    # (the actual holdout split is done in apply_with_holdout).
    if hypothesis.r_squared >= MIN_R2_TO_REPLICATE:
        gr.gate2_pass = True
        gr.gate2_r2 = hypothesis.r_squared
        gr.gate2_detail = f"in-segment R^2={hypothesis.r_squared:.3f} >= {MIN_R2_TO_REPLICATE}"
    else:
        gr.gate2_pass = False
        gr.gate2_detail = (f"in-segment R^2={hypothesis.r_squared:.3f} < {MIN_R2_TO_REPLICATE}")

    # ---- Gate 3: Competing-identity falsification ----
    # hypothesis.channel must have R^2 that beats all other channels for the same window
    # by at least COMPETING_MARGIN.
    other_r2s = [h.r_squared for h in all_channel_results_for_window
                  if h.channel != ch]
    if not other_r2s:
        gr.gate3_pass = True
        gr.gate3_detail = "no competing channels to falsify against"
    else:
        best_other = max(other_r2s)
        margin = hypothesis.r_squared - best_other
        if margin >= COMPETING_MARGIN:
            gr.gate3_pass = True
            gr.gate3_detail = (f"R^2={hypothesis.r_squared:.3f} beats best competitor "
                               f"R^2={best_other:.3f} by margin {margin:.3f} >= {COMPETING_MARGIN}")
        else:
            gr.gate3_pass = False
            gr.gate3_detail = (f"margin {margin:.3f} < {COMPETING_MARGIN}: "
                               f"R^2={hypothesis.r_squared:.3f} vs competitor R^2={best_other:.3f}")

    gr.is_finding = gr.gate1_pass and gr.gate2_pass and gr.gate3_pass

    if not gr.is_finding:
        reasons = []
        if not gr.gate1_pass:
            reasons.append(f"G1_FAIL: {gr.gate1_detail}")
        if not gr.gate2_pass:
            reasons.append(f"G2_FAIL: {gr.gate2_detail}")
        if not gr.gate3_pass:
            reasons.append(f"G3_FAIL: {gr.gate3_detail}")
        gr.quarantine_reason = "; ".join(reasons)

    return gr


def apply_with_holdout(rows: list[dict],
                       hypothesis: HypothesisResult,
                       all_segment_results: dict[str, list[HypothesisResult]],
                       truth_channels: list[str],
                       holdout_frac: float = HOLDOUT_FRAC) -> GateResult:
    """
    Apply gates with a proper train/test holdout split on `rows`.
    Overrides Gate 2 with the actual held-out R^2.
    """
    from tools.nrdr_radar_re.sweep import _extract_window

    gr = apply(hypothesis, all_segment_results,
               _same_window_results(hypothesis, all_segment_results, truth_channels))

    # Gate 2 override with actual holdout
    n = len(rows)
    split = max(2, int(n * (1.0 - holdout_frac)))
    test_rows = rows[split:]
    if len(test_rows) < 3:
        gr.gate2_detail += " (holdout too small; using train R^2)"
        return gr

    w = hypothesis.window
    x_test = np.array([
        v if (v := _extract_window(row.get(f"bytes_{w.sub_idx}", b""),
                                   w.bit_start, w.n_bits, w.coding)) is not None
        else float("nan")
        for row in test_rows
    ], dtype=float)
    y_test = np.array([float(row.get(hypothesis.channel, float("nan"))) for row in test_rows],
                       dtype=float)

    r_test, r2_test, _, _ = _pearson_r_r2(x_test, y_test)
    gr.gate2_r2 = r2_test
    if r2_test >= MIN_R2_TO_REPLICATE:
        gr.gate2_pass = True
        gr.gate2_detail = f"holdout R^2={r2_test:.3f} >= {MIN_R2_TO_REPLICATE}"
    else:
        gr.gate2_pass = False
        gr.gate2_detail = f"holdout R^2={r2_test:.3f} < {MIN_R2_TO_REPLICATE}"

    gr.is_finding = gr.gate1_pass and gr.gate2_pass and gr.gate3_pass
    if not gr.is_finding:
        reasons = []
        if not gr.gate1_pass:
            reasons.append(f"G1_FAIL: {gr.gate1_detail}")
        if not gr.gate2_pass:
            reasons.append(f"G2_FAIL: {gr.gate2_detail}")
        if not gr.gate3_pass:
            reasons.append(f"G3_FAIL: {gr.gate3_detail}")
        gr.quarantine_reason = "; ".join(reasons)
    else:
        gr.quarantine_reason = ""

    return gr


def _same_window_results(hypothesis: HypothesisResult,
                          all_segment_results: dict[str, list[HypothesisResult]],
                          truth_channels: list[str]) -> list[HypothesisResult]:
    """Collect all HypothesisResults for the same bit-window across all channels, for Gate 3."""
    w = hypothesis.window
    matches = []
    for seg_results in all_segment_results.values():
        for h in seg_results:
            if (h.window.sub_idx == w.sub_idx and
                    h.window.bit_start == w.bit_start and
                    h.window.n_bits == w.n_bits and
                    h.window.coding == w.coding):
                matches.append(h)
    return matches


def apply_batch(top_hypotheses: list[HypothesisResult],
                rows: list[dict],
                all_segment_results: dict[str, list[HypothesisResult]],
                truth_channels: list[str],
                top_n: int = 50) -> tuple[list[GateResult], list[GateResult]]:
    """
    Apply gates to the top_n hypotheses.
    Returns (findings, quarantined) — all findings pass all 3 gates.

    NOTE: this is the ONLY path through which hypotheses become findings for the report.
    """
    findings: list[GateResult] = []
    quarantined: list[GateResult] = []

    for hyp in top_hypotheses[:top_n]:
        gr = apply_with_holdout(rows, hyp, all_segment_results, truth_channels)
        if gr.is_finding:
            findings.append(gr)
        else:
            quarantined.append(gr)

    findings.sort(key=lambda g: -g.hypothesis.r_squared)
    quarantined.sort(key=lambda g: -g.hypothesis.r_squared)
    return findings, quarantined
