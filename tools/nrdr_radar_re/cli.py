"""
cli.py — command-line interface for nrdr_radar_re

Usage:
    py -3.13 -m tools.nrdr_radar_re.cli --captures <path1.csv> [<path2.csv> ...] \\
        --report-dir <output_dir> [--top-n 100] [--no-enums]

For rlog input, pass .rlog or .bz2 file paths.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# ensure the worktree root is on sys.path
_wt_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _wt_root not in sys.path:
    sys.path.insert(0, _wt_root)

from tools.nrdr_radar_re import BOSCH_RADAR_ALL_IDS
from tools.nrdr_radar_re.ingest import load, rlog_available
from tools.nrdr_radar_re.pairing import (
    assemble_sweeps, derive_vrel, attach_truth, build_matched_table,
    build_csv_truth_channels,
)
from tools.nrdr_radar_re.sweep import score_windows
from tools.nrdr_radar_re.enums import detect_all_enums
from tools.nrdr_radar_re.gates import apply_batch
from tools.nrdr_radar_re.report import write_report


TRUTH_CHANNELS = ["dRel", "yRel", "vRel", "STRENGTH", "CNTR"]


def run(capture_paths: list[str],
        report_dir: str,
        top_n: int = 100,
        no_enums: bool = False,
        verbose: bool = True) -> dict:
    """
    Main analysis pipeline.  Returns a summary dict.
    """
    os.makedirs(report_dir, exist_ok=True)

    t0 = time.monotonic()
    all_rows: list[dict] = []
    segment_results: dict[str, list] = {}
    per_segment_rows: dict[str, list[dict]] = {}

    for path in capture_paths:
        seg_id = os.path.basename(path)
        if verbose:
            print(f"[nrdr_radar_re] loading {path} ...")
        try:
            frames = load(path)
        except FileNotFoundError as e:
            print(f"[nrdr_radar_re] WARNING: {e}")
            continue

        if not frames:
            print(f"[nrdr_radar_re] WARNING: no fine-radar frames found in {path}")
            continue

        sweeps = assemble_sweeps(frames)
        derive_vrel(sweeps)
        attach_truth(sweeps, build_csv_truth_channels)
        rows = build_matched_table(sweeps)
        if not rows:
            print(f"[nrdr_radar_re] WARNING: no matched rows for {path}")
            continue

        if verbose:
            print(f"  {len(frames)} frames -> {len(sweeps)} sweeps -> {len(rows)} matched rows")

        seg_results_this = score_windows(rows, TRUTH_CHANNELS, segment_id=seg_id)
        segment_results[seg_id] = seg_results_this
        per_segment_rows[seg_id] = rows
        all_rows.extend(rows)

    if not all_rows:
        print("[nrdr_radar_re] ERROR: no data loaded from any capture.  Aborting.")
        return {"error": "no data"}

    if verbose:
        print(f"[nrdr_radar_re] total rows: {len(all_rows)}")
        print(f"[nrdr_radar_re] scoring windows across {len(segment_results)} segments ...")

    # Score on pooled data for the main candidate list
    pooled_results = score_windows(all_rows, TRUTH_CHANNELS, segment_id="pooled")

    # Gate filtering (top_n candidates from pooled)
    findings, quarantined = apply_batch(
        pooled_results, all_rows, segment_results, TRUTH_CHANNELS, top_n=top_n
    )

    # Enum detection
    enum_results = [] if no_enums else detect_all_enums(all_rows)

    elapsed = time.monotonic() - t0
    meta = {
        "captures": [os.path.basename(p) for p in capture_paths],
        "total_rows": len(all_rows),
        "segments": list(segment_results.keys()),
        "findings": len(findings),
        "quarantined": len(quarantined),
        "enum_candidates": len(enum_results),
        "elapsed_s": f"{elapsed:.1f}",
        "rlog_available": rlog_available(),
    }

    md_path = os.path.join(report_dir, "radar_re_report.md")
    json_path = os.path.join(report_dir, "radar_re_report.json")
    write_report(findings, quarantined, enum_results, md_path, json_path, meta=meta)

    if verbose:
        print(f"[nrdr_radar_re] findings: {len(findings)}, quarantined: {len(quarantined)}, "
              f"enums: {len(enum_results)}")
        print(f"[nrdr_radar_re] report written to {md_path} and {json_path}")
        print(f"[nrdr_radar_re] done in {elapsed:.1f}s")

        if findings:
            print("\n=== TOP FINDINGS ===")
            for i, gr in enumerate(findings[:5], 1):
                h = gr.hypothesis
                w = h.window
                print(f"  #{i}: sub{w.sub_idx} b{w.bit_start//8}:{(w.bit_start+w.n_bits-1)//8} "
                      f"({w.n_bits}b {w.coding}) -> {h.channel}: "
                      f"r={h.pearson_r:.4f} R^2={h.r_squared:.4f}")

    return {
        "findings": findings,
        "quarantined": quarantined,
        "enum_results": enum_results,
        "meta": meta,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="NRDR offline radar bit-discovery tool (D6 nrdrbranchdebug-2td.1)"
    )
    parser.add_argument("--captures", nargs="+", required=True,
                        help="Capture file paths (CSV or rlog)")
    parser.add_argument("--report-dir", default=".",
                        help="Output directory for report files")
    parser.add_argument("--top-n", type=int, default=100,
                        help="Number of top hypotheses to gate-check")
    parser.add_argument("--no-enums", action="store_true",
                        help="Skip enum detection")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress progress output")
    args = parser.parse_args()

    result = run(
        capture_paths=args.captures,
        report_dir=args.report_dir,
        top_n=args.top_n,
        no_enums=args.no_enums,
        verbose=not args.quiet,
    )
    if "error" in result:
        sys.exit(1)


if __name__ == "__main__":
    main()
