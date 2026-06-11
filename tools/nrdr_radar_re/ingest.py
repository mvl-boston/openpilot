"""
ingest.py — two-path capture loader

Path A: CSV format (t_mono, bus, addr_hex, dlc, data_hex, label)
  -> list of FrameRecord namedtuples filtered to the 24 fine IDs on bus 2.

Path B: openpilot rlog via openpilot.tools.lib.logreader
  -> same FrameRecord list; degrades gracefully if logreader not importable.
"""

from __future__ import annotations

import csv
import os
from dataclasses import dataclass

from tools.nrdr_radar_re import BOSCH_RADAR_ALL_IDS

_RLOG_AVAILABLE = False
try:
    from openpilot.tools.lib.logreader import LogReader  # type: ignore
    _RLOG_AVAILABLE = True
except Exception:
    pass


@dataclass(frozen=True)
class FrameRecord:
    """One CAN frame on the radar bus."""
    t_mono: float       # seconds
    addr: int           # CAN address (int)
    data: bytes         # raw payload (8 bytes)
    slot: int           # 0-5 (derived from addr)
    sub_idx: int        # 0-3 within burst (derived from addr)


def _addr_to_slot_sub(addr: int) -> tuple[int, int] | None:
    """Map a CAN address to (slot, sub_idx) if it belongs to the 24-ID table."""
    for slot_idx, base in enumerate([0x280, 0x284, 0x2D0, 0x2D4, 0x2D8, 0x2DC]):
        if base <= addr < base + 4:
            return slot_idx, addr - base
    return None


def load_csv(path: str) -> list[FrameRecord]:
    """
    Load a CSV capture file.

    Expected columns: t_mono (or timestamp), bus (or BUS), addr_hex, data_hex.
    Only rows with bus == '2' and addr in BOSCH_RADAR_ALL_IDS are returned.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"capture not found: {path}")

    fine_set = set(BOSCH_RADAR_ALL_IDS)
    records: list[FrameRecord] = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return records

        # normalize column names (case-insensitive strip)
        cols = {c.strip().lower() for c in reader.fieldnames}

        # pick timestamp column
        if "t_mono" in cols:
            t_col = "t_mono"
        elif "timestamp" in cols:
            t_col = "timestamp"
        elif "time" in cols:
            t_col = "time"
        else:
            t_col = reader.fieldnames[0]  # fallback: first column

        for row in reader:
            bus_val = row.get("bus", row.get("BUS", "")).strip()
            if bus_val != "2":
                continue
            try:
                addr = int(row["addr_hex"], 16)
            except (KeyError, ValueError):
                continue
            if addr not in fine_set:
                continue
            ss = _addr_to_slot_sub(addr)
            if ss is None:
                continue
            try:
                data = bytes.fromhex(row["data_hex"].strip())
            except (KeyError, ValueError):
                continue
            if len(data) < 6:
                data = data.ljust(8, b'\x00')
            try:
                t = float(row[t_col])
            except (KeyError, ValueError):
                t = 0.0
            records.append(FrameRecord(t_mono=t, addr=addr, data=data,
                                        slot=ss[0], sub_idx=ss[1]))

    records.sort(key=lambda r: r.t_mono)
    return records


def load_rlog(path: str) -> list[FrameRecord]:
    """
    Load an openpilot rlog.  Returns [] with a printed warning if logreader
    is not importable (expected on Windows without the full openpilot env).
    """
    if not _RLOG_AVAILABLE:
        print("[nrdr_radar_re] WARNING: openpilot.tools.lib.logreader not importable; "
              "rlog ingest is unavailable on this platform.  "
              "Use load_csv() with a CSV capture instead.")
        return []

    fine_set = set(BOSCH_RADAR_ALL_IDS)
    records: list[FrameRecord] = []

    try:
        lr = LogReader(path)
        for msg in lr:
            if msg.which() != "can":
                continue
            for frame in msg.can:
                if frame.src != 2:
                    continue
                if frame.address not in fine_set:
                    continue
                ss = _addr_to_slot_sub(frame.address)
                if ss is None:
                    continue
                data = bytes(frame.dat)
                if len(data) < 6:
                    data = data.ljust(8, b'\x00')
                t = msg.logMonoTime * 1e-9
                records.append(FrameRecord(t_mono=t, addr=frame.address,
                                            data=data, slot=ss[0], sub_idx=ss[1]))
    except Exception as exc:
        print(f"[nrdr_radar_re] WARNING: rlog load failed: {exc}")
        return []

    records.sort(key=lambda r: r.t_mono)
    return records


def load(path: str) -> list[FrameRecord]:
    """Auto-detect CSV vs rlog by file extension."""
    if path.endswith(".csv"):
        return load_csv(path)
    return load_rlog(path)


def rlog_available() -> bool:
    return _RLOG_AVAILABLE
