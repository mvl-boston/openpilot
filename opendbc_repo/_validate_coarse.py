"""Env-independent validation of the coarse-radar build, using opendbc's OWN decode math
(replicated faithfully from opendbc/can/dbc.py + parser.py:get_raw_value -- cantools can't handle the
intentional LONG_DIST_HI/LONG_DIST overlap, but opendbc decodes overlapping signals independently).
(1) decode synthetic frames exactly as opendbc's CANParser will (A2: BE bits, offset-binary scale, sentinel).
(2) prove CHKSUM/CNTR are NOT the auto-enforced names (the honda_* trap that drops frames).
(3) control-flow replica of RadarInterface._update_bosch (A3/A7: track map, sentinel, aging).
Full CANParser/RadarInterface replay runs in the built openpilot env.
"""
import math

# ---- opendbc decode math (verbatim logic from dbc.py + parser.py) ----
BE_BITS = [j + i * 8 for i in range(8) for j in range(7, -1, -1)]  # 8-byte msg

class Sig:
    def __init__(self, name, start_bit, size, signed=False, factor=1.0, offset=0.0):
        self.name, self.size, self.is_signed, self.factor, self.offset = name, size, signed, factor, offset
        self.is_little_endian = False
        idx = BE_BITS.index(start_bit)
        self.lsb = BE_BITS[idx + size - 1]
        self.msb = start_bit

def get_raw_value(dat, sig):  # verbatim from parser.py
    ret = 0; i = sig.msb // 8; bits = sig.size
    while 0 <= i < len(dat) and bits > 0:
        lsb = sig.lsb if (sig.lsb // 8) == i else i * 8
        msb = sig.msb if (sig.msb // 8) == i else (i + 1) * 8 - 1
        size = msb - lsb + 1
        d = (dat[i] >> (lsb - (i * 8))) & ((1 << size) - 1)
        ret |= d << (bits - size)
        bits -= size
        i = i + 1  # big-endian
    return ret

def decode(dat, sigs):
    out = {}
    for s in sigs:
        tmp = get_raw_value(dat, s)
        if s.is_signed:
            tmp -= ((tmp >> (s.size - 1)) & 0x1) * (1 << s.size)
        out[s.name] = tmp * s.factor + s.offset
    return out

SIGS = [
    Sig("LONG_DIST", 7, 16), Sig("LONG_DIST_HI", 7, 8), Sig("NEAR_DIST", 23, 8),
    Sig("FLAGS_B3", 31, 8), Sig("LAT_DIST", 39, 16, factor=1, offset=-32768),
    Sig("REL_SPEED", 55, 8, factor=0.25, offset=-31.25), Sig("CNTR", 63, 4), Sig("CHKSUM", 59, 4),
]
names = {s.name for s in SIGS}
assert "COUNTER" not in names and "CHECKSUM" not in names, "must NOT use auto-enforced names"

def fr(*b):
    return bytes(b)

d = decode(fr(0x05, 0xDC, 0x10, 0x00, 0x80, 0x64, 0x6D, 0x3A), SIGS)
assert d["LONG_DIST"] == 1500, d["LONG_DIST"]            # B0:B1 = 0x05DC
assert d["LONG_DIST_HI"] == 0x05, d["LONG_DIST_HI"]      # B0
assert d["NEAR_DIST"] == 0x10, d["NEAR_DIST"]            # B2
assert d["LAT_DIST"] == 100, d["LAT_DIST"]               # 0x8064-0x8000
assert abs(d["REL_SPEED"] - (-4.0)) < 1e-9, d["REL_SPEED"]  # (0x6D-0x7D)*0.25
assert d["CNTR"] == 0x3 and d["CHKSUM"] == 0xA, (d["CNTR"], d["CHKSUM"])  # B7 nibbles (BE: start63=hi, start59=lo); cosmetic, unused
print("[A2] live decode (opendbc math): LONG_DIST=%d LONG_DIST_HI=0x%02X NEAR=%d LAT=%d REL=%.2f CNTR=%d CHK=%d"
      % (d["LONG_DIST"], int(d["LONG_DIST_HI"]), d["NEAR_DIST"], d["LAT_DIST"], d["REL_SPEED"], d["CNTR"], d["CHKSUM"]))

c = decode(fr(0x05, 0xDC, 0x00, 0x00, 0x80, 0x00, 0x7D, 0x00), SIGS)
assert c["LAT_DIST"] == 0 and abs(c["REL_SPEED"]) < 1e-9
s = decode(fr(0xFF, 0xF0, 0xFF, 0x00, 0x80, 0x00, 0x7D, 0x00), SIGS)
assert s["LONG_DIST_HI"] == 0xFF
print("[A2] center (LAT 0x8000->0, REL 0x7D->0) + no-target sentinel (LONG_DIST_HI==0xFF) OK")

# (2) confirm opendbc's dbc.py would NOT auto-type CHKSUM/CNTR (source-verified: set_signal_type matches
#     exactly 'CHECKSUM'/'COUNTER'; ours are CHKSUM/CNTR -> type DEFAULT, calc_checksum None -> no drop)
print("[A2] CHKSUM/CNTR avoid the honda_* auto-enforcement trap (names != CHECKSUM/COUNTER) -> frames decode unconditionally")

# (3) control-flow replica of _update_bosch
BOSCH = [0x2C8, 0x2C9]
class Pt:  __slots__ = ("trackId","dRel","yRel","vRel","aRel","yvRel","measured")
class Errs: canError = False
class RDt:
    def __init__(self): self.errors = Errs(); self.points = []
def ub(pts, vl, upd, can_valid):
    ret = RDt()
    if not can_valid: ret.errors.canError = True
    for ii in BOSCH:
        tid = BOSCH.index(ii)
        if ii not in upd: pts.pop(tid, None); continue
        cpt = vl[ii]
        if cpt["LONG_DIST_HI"] == 0xFF: pts.pop(tid, None); continue
        if tid not in pts:
            p = Pt(); p.trackId = tid; p.aRel = float("nan"); p.yvRel = float("nan"); pts[tid] = p
        pts[tid].dRel = cpt["LONG_DIST"]; pts[tid].yRel = -cpt["LAT_DIST"]
        pts[tid].vRel = cpt["REL_SPEED"]; pts[tid].measured = True
    ret.points = list(pts.values()); return ret

live = {"LONG_DIST":1500,"LONG_DIST_HI":0x05,"LAT_DIST":100,"REL_SPEED":-4.0}
sent = {"LONG_DIST":65520,"LONG_DIST_HI":0xFF,"LAT_DIST":0,"REL_SPEED":0.0}
pts = {}
r = ub(pts, {0x2C8:live,0x2C9:live}, {0x2C8,0x2C9}, True)
assert len(r.points)==2 and {p.trackId for p in r.points}=={0,1}
assert pts[0].dRel==1500 and pts[0].yRel==-100 and pts[0].vRel==-4.0 and pts[0].measured and math.isnan(pts[0].aRel)
print("[A3] two live leads -> 2 points (track 0/1); dRel/yRel(-LAT)/vRel/measured; aRel NaN; no canError")
r = ub(pts, {0x2C8:sent,0x2C9:live}, {0x2C8,0x2C9}, True)
assert 0 not in pts and 1 in pts and len(r.points)==1
print("[A7] 0x2C8 sentinel -> track 0 cleared, track 1 kept")
pts = {0:Pt(),1:Pt()}; pts[0].trackId=0; pts[1].trackId=1
r = ub(pts, {0x2C9:live}, {0x2C9}, True)
assert 0 not in pts and 1 in pts
print("[A7] 0x2C8 absent from cycle -> stale track 0 aged out")
r = ub({}, {0x2C9:live}, {0x2C9}, False)
assert r.errors.canError is True and r is not None
print("[A7] can_valid False -> canError=True, RadarData returned (not None)")
print("\nALL CHECKS PASSED")
