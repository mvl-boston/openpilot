# nrdr_radar_re — Offline Radar Bit-Discovery Tool

D6 (nrdrbranchdebug-2td.1). Zero on-car risk: this tool never ships to device.

## Purpose

Builds the bespoke three-source regression method (which settled b4:b5 = azimuth and
FALSIFIED range-rate at R^2 0.01-0.08) into a standing offline tool.  Ingests radar
captures, sweeps every aligned bit window under all three codings, and produces a
gate-filtered ranked report.

## 24-ID Map

The 36802-TBA Bosch radar emits a 6-slot sweep.  Each slot is a 4-frame burst on
consecutive CAN IDs:

| Slot | Header | +1    | +2    | +3    |
|------|--------|-------|-------|-------|
| 0    | 0x280  | 0x281 | 0x282 | 0x283 |
| 1    | 0x284  | 0x285 | 0x286 | 0x287 |
| 2    | 0x2D0  | 0x2D1 | 0x2D2 | 0x2D3 |
| 3    | 0x2D4  | 0x2D5 | 0x2D6 | 0x2D7 |
| 4    | 0x2D8  | 0x2D9 | 0x2DA | 0x2DB |
| 5    | 0x2DC  | 0x2DD | 0x2DE | 0x2DF |

The header sub-frame (sub_idx=0) carries RANGE and LAT_RAW when b1==0x74.

## Gate Contract (Non-Bypassable)

**A hypothesis is a FINDING only if ALL three gates pass** — enforced in
`gates.py`, not documentation.  The report generator imports findings exclusively
through `gates.apply_batch()`.

1. **Replication (Gate 1)**: R^2 >= 0.10 and |r| >= 0.30 in >= 2 independent
   segments/sources, with consistent sign.

2. **Holdout (Gate 2)**: R^2 >= 0.10 on the held-out 40% of rows (train/test
   split by segment index — first 60% train, last 40% test).

3. **Competing-identity falsification (Gate 3)**: the hypothesis's R^2 against
   its best channel must beat its R^2 against every other channel by at least
   0.05.  This prevents declaring something a range-rate proxy when it is equally
   explained by dRel or STRENGTH.

Anything failing any gate appears ONLY in the quarantined "unreplicated candidates"
appendix of the report.

## Methodology

### Hypothesis sweep

For every undeclared byte-region in every sub-frame (all 4 sub-frames, all slots):
- Enumerate every aligned 4/8/12/16-bit window.
- Decode under three codings: unsigned, two's-complement, offset-binary.
- Score each (window, coding) against every truth channel via Pearson r and OLS R^2.

### Truth channels

For CSV captures (no external vision-lead):
- `dRel`, `vRel` (d(dRel)/dt), `yRel`, `STRENGTH`, `CNTR` — the current decode.

For rlog captures (if openpilot.tools.lib.logreader is importable):
- Same current-decode channels + any vision-model lead channels that can be
  matched by nearest-in-time with position-sanity check.

The truth-channel registry is pluggable: supply a `TruthProvider` callable to
`pairing.attach_truth()`.

### Circularity disclosure

The acceptance test finds b4:b5 as the top predictor of `yRel` — expected,
because `yRel` IS derived from b4:b5 in the current decode
(`yRel = -dRel * sin((b4b5 - 0x8000) * 0.001 * pi/180)`).  This test validates
the EXTRACTION + SWEEP machinery, not the original field-identity discovery.
The original azimuth discovery used independent vision-model data (rlogs) and is
documented in `radar_interface.py:40-69`.

## Usage

```
py -3.13 -m tools.nrdr_radar_re.cli \
    --captures path/to/closing_bfcar.csv path/to/closing_10m.csv \
    --report-dir /tmp/radar_report \
    --top-n 100
```

Output: `radar_re_report.md` and `radar_re_report.json` in `--report-dir`.

## Running Tests

From `C:\nrdrbranchdebug\pyshim`:

```
PYTHONPATH="C:\nrdrbranchdebug\wt-long" py -3.13 -m pytest ^
    "C:\nrdrbranchdebug\wt-long\tools\nrdr_radar_re\tests" ^
    -q -p no:cacheprovider -o addopts="" --noconftest
```

`--noconftest` is required on Windows when the worktree root `conftest.py`
imports on-device openpilot modules (`openpilot.common`, etc.) that are not
available in the Windows Python environment.  The `pytest.ini` in this directory
prevents the root conftest from loading when pytest is started with
`--rootdir=tools/nrdr_radar_re` or `--noconftest`.

Known-answer bfcar tests skip cleanly if captures are not present.

## Module List

| Module     | Role |
|------------|------|
| `__init__.py` | Constants: 24-ID map, header IDs, shipped constants |
| `ingest.py`   | CSV + optional rlog loader -> FrameRecord list |
| `pairing.py`  | Sweep assembly, vRel derivation, truth attachment, matched table |
| `sweep.py`    | Bit-window enumeration, multi-coding decode, Pearson/OLS scoring |
| `enums.py`    | State-field enum detector (cardinality, dwell, transition matrix) |
| `gates.py`    | Non-bypassable replication/holdout/falsification gates |
| `report.py`   | Markdown + JSON report generator (ONLY via gates.apply_batch) |
| `cli.py`      | argparse CLI: captures in, report out |
| `tests/test_d6_harness.py` | Full test suite |

## rlog Support

rlog ingest requires `openpilot.tools.lib.logreader` (available on-device or in WSL
with the full openpilot environment).  On Windows without that environment, rlog ingest
degrades gracefully with a clear warning.  CSV ingest works everywhere.
