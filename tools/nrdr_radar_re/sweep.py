"""
sweep.py — bit-window enumeration, multi-coding decode, and scoring

For every undeclared byte-region in every sub-frame, enumerate every aligned
4/8/12/16-bit window under three codings:
  - unsigned
  - two's-complement (signed)
  - offset-binary (centered at mid-range)

Score each (window, coding) against every truth channel via:
  - Pearson r
  - OLS R^2

Returns a list of HypothesisResult, ranked by |r| descending.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

# Declared byte ranges per sub_idx (inclusive byte indices, 0-based in 8-byte frame)
# sub_idx 0 (range-carrier, b1==0x74): b0=STRENGTH, b1=TAG, b2:b3=RANGE_RAW, b7=CNTR
# b4:b5 is LAT_RAW (declared, but included so the sweep can RE-DISCOVER it as a sanity check)
# b6 is undeclared (included in sweep)
#
# For purposes of the re-discovery acceptance test, we allow sweeping ALL bytes including
# declared ones — the circularity (yRel derived from b4:b5 -> sweep finds b4:b5 predicts yRel)
# is documented explicitly in the report.
DECLARED_BYTES: dict[int, set[int]] = {
    0: {0, 1, 2, 3, 7},    # b0=STR, b1=TAG, b2:b3=RANGE, b7=CNTR; b4:b5=LAT undeclared for sweep
    1: set(),               # sub-frame +1: fully undeclared
    2: set(),               # sub-frame +2: fully undeclared
    3: set(),               # sub-frame +3: fully undeclared
}

WINDOW_SIZES = (4, 8, 12, 16)  # bits


@dataclass
class BitWindow:
    """One bit-window specification."""
    sub_idx: int      # which sub-frame (0-3)
    bit_start: int    # LSB-first bit offset within the 8-byte (64-bit) frame
    n_bits: int       # 4, 8, 12, or 16
    coding: str       # "unsigned" | "twos_complement" | "offset_binary"


@dataclass
class HypothesisResult:
    """Scored hypothesis for one bit-window against one truth channel."""
    window: BitWindow
    channel: str
    n: int
    pearson_r: float
    r_squared: float
    ols_slope: float
    ols_intercept: float
    segment_id: str = ""

    @property
    def key(self) -> tuple:
        w = self.window
        return (w.sub_idx, w.bit_start, w.n_bits, w.coding, self.channel)


def enumerate_windows(frame_len_bytes: int = 8,
                      window_sizes: Sequence[int] = WINDOW_SIZES) -> list[BitWindow]:
    """
    Enumerate all aligned bit windows of the given sizes across all 4 sub-frames.
    'Aligned' means bit_start is a multiple of the window size.
    Returns windows for BOTH declared and undeclared bytes (for circularity check);
    mark declared status separately in caller.
    """
    windows: list[BitWindow] = []
    total_bits = frame_len_bytes * 8
    for sub_idx in range(4):
        for n_bits in window_sizes:
            step = n_bits  # aligned
            for bit_start in range(0, total_bits - n_bits + 1, step):
                for coding in ("unsigned", "twos_complement", "offset_binary"):
                    windows.append(BitWindow(sub_idx=sub_idx, bit_start=bit_start,
                                             n_bits=n_bits, coding=coding))
    return windows


def _extract_window(data: bytes, bit_start: int, n_bits: int, coding: str) -> float | None:
    """
    Extract an aligned bit window from a frame (big-endian byte order, MSB-first within bytes).
    Returns the decoded value under the specified coding, or None if data is too short.
    """
    frame_len = len(data)
    if frame_len < 8:
        data = data + b'\x00' * (8 - frame_len)

    byte_start = bit_start // 8
    byte_end = (bit_start + n_bits - 1) // 8
    if byte_end >= 8:
        return None

    # extract bytes (big-endian / network order, matching CAN convention)
    n_bytes = byte_end - byte_start + 1
    raw = 0
    for i in range(n_bytes):
        raw = (raw << 8) | data[byte_start + i]

    # align: shift right if bit_start is not byte-aligned
    # (since we only support aligned windows, bit_start % 8 == 0 always)
    shift = (n_bytes * 8) - n_bits - (bit_start % 8)
    raw = (raw >> shift) & ((1 << n_bits) - 1)

    max_val = 1 << n_bits
    half = max_val >> 1

    if coding == "unsigned":
        return float(raw)
    elif coding == "twos_complement":
        if raw >= half:
            raw -= max_val
        return float(raw)
    elif coding == "offset_binary":
        return float(raw - half)
    return None


def _pearson_r_r2(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float, float]:
    """
    Compute Pearson r and OLS R^2 between x and y.
    Also returns OLS slope and intercept.
    Handles degenerate cases (constant x or y) by returning 0.0.
    """
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    n = len(x)
    if n < 3:
        return 0.0, 0.0, 0.0, 0.0

    xm = x - x.mean()
    ym = y - y.mean()
    sx = float(np.sqrt((xm * xm).mean()))
    sy = float(np.sqrt((ym * ym).mean()))
    if sx < 1e-12 or sy < 1e-12:
        return 0.0, 0.0, 0.0, 0.0

    r = float((xm * ym).mean() / (sx * sy))
    r = max(-1.0, min(1.0, r))
    r2 = r * r

    # OLS slope + intercept
    slope = r * sy / sx
    intercept = float(y.mean()) - slope * float(x.mean())

    return r, r2, slope, intercept


def score_windows(rows: list[dict],
                  truth_channels: list[str],
                  segment_id: str = "",
                  frame_len: int = 8) -> list[HypothesisResult]:
    """
    Score every bit-window against every truth channel.

    rows: output of pairing.build_matched_table() — each row has bytes_0..bytes_3
          and truth channel values.
    truth_channels: list of channel names (keys in row dict) to score against.
    segment_id: tag for provenance in report.
    """
    if not rows:
        return []

    windows = enumerate_windows(frame_len_bytes=frame_len)
    results: list[HypothesisResult] = []

    # Pre-extract all window values
    n_rows = len(rows)
    # build window->values array
    win_vals: dict[tuple, np.ndarray] = {}
    for w in windows:
        key = (w.sub_idx, w.bit_start, w.n_bits, w.coding)
        vals = np.full(n_rows, np.nan)
        for i, row in enumerate(rows):
            data = row.get(f"bytes_{w.sub_idx}", b"")
            v = _extract_window(data, w.bit_start, w.n_bits, w.coding)
            if v is not None:
                vals[i] = v
        win_vals[key] = vals

    # score each (window, coding) vs each truth channel
    for w in windows:
        key = (w.sub_idx, w.bit_start, w.n_bits, w.coding)
        x = win_vals[key]
        if not np.any(np.isfinite(x)):
            continue

        for ch in truth_channels:
            y_raw = [row.get(ch, float("nan")) for row in rows]
            y = np.array(y_raw, dtype=float)
            if not np.any(np.isfinite(y)):
                continue

            r, r2, slope, intercept = _pearson_r_r2(x, y)
            if r == 0.0 and r2 == 0.0:
                continue

            results.append(HypothesisResult(
                window=BitWindow(sub_idx=w.sub_idx, bit_start=w.bit_start,
                                 n_bits=w.n_bits, coding=w.coding),
                channel=ch,
                n=int(np.sum(np.isfinite(x) & np.isfinite(y))),
                pearson_r=r,
                r_squared=r2,
                ols_slope=slope,
                ols_intercept=intercept,
                segment_id=segment_id,
            ))

    results.sort(key=lambda h: -abs(h.pearson_r))
    return results
