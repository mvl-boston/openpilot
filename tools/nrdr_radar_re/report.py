"""
report.py — ranked findings report (Markdown + JSON)

Generates report ONLY from GateResult objects — findings arrive exclusively
through gates.apply_batch().  Raw hypothesis scores are NOT visible in the
findings section; unreplicated candidates appear only in the quarantined appendix.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from typing import Sequence

from tools.nrdr_radar_re.gates import GateResult
from tools.nrdr_radar_re.enums import EnumDetectorResult
from tools.nrdr_radar_re import BOSCH_RADAR_ALL_IDS, BOSCH_RADAR_HDR_MSGS


def _window_label(gr: GateResult) -> str:
    w = gr.hypothesis.window
    byte_lo = w.bit_start // 8
    byte_hi = (w.bit_start + w.n_bits - 1) // 8
    return f"sub{w.sub_idx} b{byte_lo}:{byte_hi} ({w.n_bits}b, {w.coding})"


def _circularity_note(gr: GateResult) -> str:
    """Flag known circular findings (b4:b5 vs yRel) with a methodology note."""
    w = gr.hypothesis.window
    ch = gr.hypothesis.channel
    # b4:b5 on sub_idx=0, bits 32..47 (offset-binary or unsigned 16-bit)
    if w.sub_idx == 0 and w.bit_start == 32 and w.n_bits == 16 and ch in ("yRel",):
        return ("  > CIRCULARITY NOTE: yRel is derived from b4:b5 via the current decode "
                "(yRel = -dRel * sin(lat_signed * scale)).  This result validates the "
                "EXTRACTION + SWEEP machinery, NOT the original field-identity discovery.  "
                "The original azimuth discovery used independent vision-model data (rlogs).")
    return ""


def generate_markdown(findings: list[GateResult],
                       quarantined: list[GateResult],
                       enum_results: list[EnumDetectorResult],
                       meta: dict | None = None) -> str:
    lines = []

    lines.append("# NRDR Radar Bit-Discovery Report")
    lines.append("")
    if meta:
        for k, v in meta.items():
            lines.append(f"- **{k}**: {v}")
        lines.append("")

    # 24-ID map
    lines.append("## 24-ID Map")
    lines.append("")
    lines.append("| Slot | Header | +1 | +2 | +3 |")
    lines.append("|------|--------|----|----|-----|")
    for i, hdr in enumerate(BOSCH_RADAR_HDR_MSGS):
        cols = [f"0x{hdr+j:03X}" for j in range(4)]
        lines.append(f"| {i} | {cols[0]} | {cols[1]} | {cols[2]} | {cols[3]} |")
    lines.append("")

    # Gate contract
    lines.append("## Gate Contract (Non-Bypassable)")
    lines.append("")
    lines.append("A hypothesis becomes a FINDING only if ALL three gates pass:")
    lines.append("1. **Replication**: R^2 >= 0.10 and |r| >= 0.30 in >= 2 independent segments "
                 "with consistent sign.")
    lines.append("2. **Holdout**: R^2 >= 0.10 on the held-out 40% of rows.")
    lines.append("3. **Competing-identity**: best-channel R^2 beats all other channels "
                 "by >= 0.05 margin.")
    lines.append("")
    lines.append("Anything failing appears ONLY in the quarantined appendix below.")
    lines.append("")

    # Findings
    lines.append(f"## Findings ({len(findings)} passed all gates)")
    lines.append("")
    if not findings:
        lines.append("_No hypotheses passed all three gates._")
        lines.append("")
    else:
        for rank, gr in enumerate(findings, 1):
            h = gr.hypothesis
            w = h.window
            lines.append(f"### Finding #{rank}: {_window_label(gr)} → {h.channel}")
            lines.append(f"- Pearson r = {h.pearson_r:.4f}, OLS R^2 = {h.r_squared:.4f}, n = {h.n}")
            lines.append(f"- OLS slope = {h.ols_slope:.6g}, intercept = {h.ols_intercept:.6g}")
            lines.append(f"- Segment: {h.segment_id}")
            lines.append(f"- **Gate 1 (replication)**: {'PASS' if gr.gate1_pass else 'FAIL'} — "
                         f"{gr.gate1_detail}")
            lines.append(f"- **Gate 2 (holdout)**: {'PASS' if gr.gate2_pass else 'FAIL'} — "
                         f"{gr.gate2_detail}")
            lines.append(f"- **Gate 3 (competing-identity)**: "
                         f"{'PASS' if gr.gate3_pass else 'FAIL'} — {gr.gate3_detail}")
            circ = _circularity_note(gr)
            if circ:
                lines.append(circ)
            lines.append("")

    # Enum results
    lines.append(f"## State-Field Enum Candidates ({len(enum_results)} detected)")
    lines.append("")
    if not enum_results:
        lines.append("_No enum-like fields detected._")
        lines.append("")
    else:
        for e in enum_results:
            lines.append(f"### Enum: sub{e.sub_idx} byte {e.bit_start // 8} ({e.channel_label})")
            lines.append(f"- Cardinality: {e.n_distinct}, values: {e.values}")
            lines.append(f"- Dwell: mean={e.dwell_mean:.1f}, p50={e.dwell_p50:.0f}, "
                         f"p95={e.dwell_p95:.0f}")
            lines.append(f"- Transitions: {len(e.transition_matrix)}")
            lines.append("")

    # Quarantined appendix
    lines.append(f"## Quarantined (Unreplicated Candidates) — {len(quarantined)} entries")
    lines.append("")
    lines.append("_These hypotheses did NOT pass all gates.  Shown for completeness only._")
    lines.append("")
    for gr in quarantined[:20]:  # top 20 only
        h = gr.hypothesis
        lines.append(f"- {_window_label(gr)} → {h.channel}: "
                     f"r={h.pearson_r:.3f}, R^2={h.r_squared:.3f}; "
                     f"QUARANTINE: {gr.quarantine_reason}")
    if len(quarantined) > 20:
        lines.append(f"- ... and {len(quarantined) - 20} more (see JSON)")
    lines.append("")

    return "\n".join(lines)


def generate_json(findings: list[GateResult],
                   quarantined: list[GateResult],
                   enum_results: list[EnumDetectorResult],
                   meta: dict | None = None) -> str:
    def _gr_to_dict(gr: GateResult) -> dict:
        h = gr.hypothesis
        w = h.window
        return {
            "window": {
                "sub_idx": w.sub_idx,
                "bit_start": w.bit_start,
                "n_bits": w.n_bits,
                "coding": w.coding,
            },
            "channel": h.channel,
            "pearson_r": h.pearson_r,
            "r_squared": h.r_squared,
            "ols_slope": h.ols_slope,
            "ols_intercept": h.ols_intercept,
            "n": h.n,
            "segment_id": h.segment_id,
            "gate1_pass": gr.gate1_pass,
            "gate1_detail": gr.gate1_detail,
            "gate2_pass": gr.gate2_pass,
            "gate2_r2": gr.gate2_r2,
            "gate2_detail": gr.gate2_detail,
            "gate3_pass": gr.gate3_pass,
            "gate3_detail": gr.gate3_detail,
            "is_finding": gr.is_finding,
            "quarantine_reason": gr.quarantine_reason,
        }

    def _enum_to_dict(e: EnumDetectorResult) -> dict:
        return {
            "sub_idx": e.sub_idx,
            "bit_start": e.bit_start,
            "n_bits": e.n_bits,
            "coding": e.coding,
            "label": e.channel_label,
            "n_distinct": e.n_distinct,
            "values": e.values,
            "dwell_mean": e.dwell_mean,
            "dwell_p50": e.dwell_p50,
            "dwell_p95": e.dwell_p95,
            "n_transitions": len(e.transition_matrix),
        }

    payload = {
        "meta": meta or {},
        "id_map": {
            f"slot_{i}": [f"0x{hdr+j:03X}" for j in range(4)]
            for i, hdr in enumerate(BOSCH_RADAR_HDR_MSGS)
        },
        "findings": [_gr_to_dict(gr) for gr in findings],
        "quarantined": [_gr_to_dict(gr) for gr in quarantined],
        "enum_candidates": [_enum_to_dict(e) for e in enum_results],
    }
    return json.dumps(payload, indent=2)


def write_report(findings: list[GateResult],
                  quarantined: list[GateResult],
                  enum_results: list[EnumDetectorResult],
                  md_path: str,
                  json_path: str,
                  meta: dict | None = None) -> None:
    md = generate_markdown(findings, quarantined, enum_results, meta)
    js = generate_json(findings, quarantined, enum_results, meta)
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(md)
    with open(json_path, "w", encoding="utf-8") as f:
        f.write(js)
