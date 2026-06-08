"""Offline validation of the FINE 0x280 track-table radar build (coarse 0x2C8 -> fine 0x280 upgrade).

Drives opendbc's REAL CANParser + the Honda RadarInterface end-to-end (no env, no panda, no CAN TX):
 (1) DBC decode of synthetic + real-capture frames (range scale/offset, offset-binary lateral, sentinels).
 (2) the honda_* CHKSUM/CNTR auto-enforcement trap is avoided (frames decode unconditionally).
 (3) RadarInterface._update_bosch behavior: b1==0x74 gate, sentinel skip, multi-object emit, trackId
     stability, derived vRel sign, staleness -> EMPTY RadarData (not None).
 (4) keep-AEB invariants (source-level): interface.py override + the 6 new radar IDs absent from honda.h.

Run:  py -3.13 _validate_fine.py     (needs Python 3.11+, which opendbc requires.)
The full openpilot replay (card/radard) runs in the built openpilot env; this is the opendbc-side proof.
"""
import math
import os
import re

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.honda.radar_interface import (
  RadarInterface, BOSCH_RADAR_HDR_MSGS, BOSCH_RADAR_HDR_TAG, BOSCH_RADAR_LAT_SCALE, BOSCH_RADAR_STALE_S,
)
from opendbc.car.honda.values import CAR, DBC

HERE = os.path.dirname(os.path.abspath(__file__))
HONDA_H = os.path.join(HERE, "opendbc", "safety", "modes", "honda.h")
INTERFACE = os.path.join(HERE, "opendbc", "car", "honda", "interface.py")
BFCAR_CSV = r"C:/claudecode/firmware-analysis-kit/radar-re/captures/closing_bfcar.csv"
IDLE_CSV = r"C:/claudecode/firmware-analysis-kit/radar-re/captures/closing_10m.csv"


def hf(range_raw, *, tag=BOSCH_RADAR_HDR_TAG, strength=0x00, lat_raw=0x8000, cntr=0x00):
  return bytes([strength, tag, (range_raw >> 8) & 0xFF, range_raw & 0xFF,
                (lat_raw >> 8) & 0xFF, lat_raw & 0xFF, 0x00, cntr])


def make_ri():
  CP = structs.CarParams()
  CP.carFingerprint = CAR.HONDA_CIVIC_BOSCH
  CP.radarUnavailable = False
  return RadarInterface(CP, structs.CarParamsSP())


# ---- (1)+(2) DBC decode + CHKSUM/CNTR trap ----
parser = CANParser(DBC[CAR.HONDA_CIVIC_BOSCH][Bus.radar], [(m, 20) for m in BOSCH_RADAR_HDR_MSGS], 2)
bus = parser.bus
parser.update([0, [(0x280, bytes.fromhex("00740f9f9bc00336"), bus)]])
vl = parser.vl[0x280]
assert int(vl["TRACK_TAG"]) == 0x74
assert int(vl["RANGE_RAW"]) == 0x0F9F
assert abs(vl["RANGE"] - (0.00357 * 3999 - 3.0)) < 1e-4 and abs(vl["RANGE"] - 11.28) < 0.01
assert int(vl["LAT_RAW"]) == 0x9BC0 - 0x8000  # offset-binary
assert int(vl["CNTR"]) == 0x36
print("[1] DBC decode (real bfcar frame): RANGE=%.4f m  RANGE_RAW=0x%04X  LAT_RAW=%d  CNTR=0x%02X  tag=0x74 OK"
      % (vl["RANGE"], int(vl["RANGE_RAW"]), int(vl["LAT_RAW"]), int(vl["CNTR"])))

names = set()
for m in BOSCH_RADAR_HDR_MSGS:
  names |= set(parser.vl[m].keys())
assert "CHECKSUM" not in names and "COUNTER" not in names and "CNTR" in names
parser.update([1, [(0x280, hf(5000, cntr=0xAB), bus)]])  # arbitrary counter must NOT drop the frame
assert int(parser.vl[0x280]["CNTR"]) == 0xAB and abs(parser.vl[0x280]["RANGE"] - (0.00357 * 5000 - 3.0)) < 1e-4
print("[2] CHKSUM/CNTR trap avoided (names != CHECKSUM/COUNTER; arbitrary CNTR still decodes) OK")

# ---- (3) RadarInterface._update_bosch behavior ----
ri = make_ri()
b = ri.rcp.bus
assert ri.bosch_radar and ri.trigger_msg == 0x280

rr = ri.update([0, [(0x280, hf(3999), b)]])
assert rr is not None and len(rr.points) == 1 and rr.points[0].trackId == 0
assert abs(rr.points[0].dRel - (0.00357 * 3999 - 3.0)) < 1e-4 and math.isnan(rr.points[0].vRel)
print("[3a] single live track -> 1 point, trackId 0, dRel calibrated, vRel NaN on first sight OK")

rr = ri.update([int(0.05e9), [(0x280, hf(3999, tag=0x02), b)]])  # b1 != 0x74
assert len(rr.points) == 0
print("[3b] b1==0x74 gate: non-header sub-frame -> 0 points OK")

for label, frame in (("strength 0xFE", hf(3999, strength=0xFE)), ("range 0x8000", hf(0x8000)),
                     ("range 0xFF80", hf(0xFF80))):
  ri2 = make_ri()
  rr = ri2.update([0, [(0x280, frame, ri2.rcp.bus)]])
  assert len(rr.points) == 0, label
print("[3c] sentinel skip (strength 0xFE / range 0x8000 / range>=0xFF80) -> 0 points OK")

ri = make_ri(); b = ri.rcp.bus
rr = ri.update([0, [(0x280, hf(3000), b), (0x284, hf(4000), b), (0x2D0, hf(5000), b)]])
assert {p.trackId for p in rr.points} == {0, 1, 2}
print("[3d] multi-object emission -> 3 points, stable distinct trackIds {0,1,2} OK")

rr2 = ri.update([int(0.05e9), [(0x280, hf(2900), b), (0x284, hf(4000), b), (0x2D0, hf(5000), b)]])
p0 = next(p for p in rr2.points if p.trackId == 0)
assert not math.isnan(p0.vRel) and p0.vRel < 0.0  # range shrank -> closing -> vRel negative
print("[3e] derived vRel (d(dRel)/dt) closing-negative on a shrinking range OK (vRel=%.2f m/s)" % p0.vRel)

# staleness: build a point on 0x280, then let the parser clock advance past STALE_S without 0x280
ri = make_ri(); b = ri.rcp.bus
ri.update([0, [(0x280, hf(3000), b)]])
assert len(ri.pts) == 1
rr = ri.update([int((BOSCH_RADAR_STALE_S + 0.05) * 1e9), [(0x284, hf(0x8000), b)]])
assert rr is not None and len(rr.points) == 0 and rr.errors.radarUnavailableTemporary and len(ri.pts) == 0
print("[3f] staleness gate: 0x280 quiet > %.2fs -> EMPTY RadarData (not None), points cleared OK" % BOSCH_RADAR_STALE_S)

# ---- (4) keep-AEB invariants ----
isrc = open(INTERFACE).read()
blk = re.search(r"if candidate == CAR.HONDA_CIVIC_BOSCH:(.*?)else:", isrc, re.S).group(1)
assert "ret.radarUnavailable = False" in blk
assert "ret.openpilotLongitudinalControl = False" in blk
assert "ret.alphaLongitudinalAvailable = False" in blk
assert "CP.openpilotLongitudinalControl" in isrc and "0x18DAB0F1" in isrc
print("[4a] keep-AEB: openpilotLongitudinalControl HARD-PINNED False -> disable_ecu(0x18DAB0F1) unreachable OK")

hsrc = open(HONDA_H).read()
for tok in ("0x280", "0x284", "0x2D0", "0x2D4", "0x2D8", "0x2DC", "640", "644", "720", "724", "728", "732"):
  assert tok not in hsrc, f"radar id {tok} unexpectedly present in honda.h"
print("[4b] keep-AEB K4: all 6 fine radar IDs absent from honda.h (RX-only; not in any tx/relay/rx list) OK")

# ---- optional real-capture replay ----
def load_280(path):
  import csv
  out = []
  with open(path) as f:
    for row in csv.DictReader(f):
      if row["bus"] == "2" and row["addr_hex"].upper().endswith("280"):
        out.append(bytes.fromhex(row["data_hex"]))
  return out

if os.path.exists(BFCAR_CSV):
  ri = make_ri(); b = ri.rcp.bus
  ranges, vrels = [], []
  for i, data in enumerate(load_280(BFCAR_CSV)):
    rr = ri.update([i * int(0.05e9), [(0x280, data, b)]])
    if rr is not None and rr.points:
      ranges.append(rr.points[0].dRel)
      if not math.isnan(rr.points[0].vRel):
        vrels.append(rr.points[0].vRel)
  assert len(ranges) > 100 and abs(max(ranges) - 11.28) < 0.2 and abs(min(ranges) - 1.52) < 0.5
  assert sum(vrels) / len(vrels) < 0.0
  print("[5] bfcar positive control: %d pts, close %.2f->%.2f m, mean derived vRel=%.2f m/s OK"
        % (len(ranges), max(ranges), min(ranges), sum(vrels) / len(vrels)))

if os.path.exists(IDLE_CSV):
  ri = make_ri(); b = ri.rcp.bus
  mx = 0
  for i, data in enumerate(load_280(IDLE_CSV)):
    rr = ri.update([i * int(0.05e9), [(0x280, data, b)]])
    if rr is not None:
      mx = max(mx, len(rr.points))
  assert mx == 0
  print("[5] 10m idle negative control: 0 points across all frames (sentinel + b1 gate) OK")

print("\nALL FINE-INGEST CHECKS PASSED")
