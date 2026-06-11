# nrdr_radar_re — offline radar bit-discovery tool
# D6 (nrdrbranchdebug-2td.1)
"""
Three-source regression engine for Honda Bosch 36802-TBA radar reverse-engineering.

Settles b4:b5 = AZIMUTH (offset-binary, center 0x8000), falsifies range-rate at R^2 0.01-0.08.
See README.md for full methodology and gate contract.
"""

__version__ = "0.1.0"

# 6 header IDs — each heads a 4-frame burst (header, +1, +2, +3)
BOSCH_RADAR_HDR_MSGS = [0x280, 0x284, 0x2D0, 0x2D4, 0x2D8, 0x2DC]

# Full 24-ID table: each slot is a 4-frame burst on consecutive IDs
# slot 0: 0x280, 0x281, 0x282, 0x283
# slot 1: 0x284, 0x285, 0x286, 0x287
# slot 2: 0x2D0, 0x2D1, 0x2D2, 0x2D3
# slot 3: 0x2D4, 0x2D5, 0x2D6, 0x2D7
# slot 4: 0x2D8, 0x2D9, 0x2DA, 0x2DB
# slot 5: 0x2DC, 0x2DD, 0x2DE, 0x2DF
BOSCH_RADAR_ALL_IDS: list[int] = []
for _h in BOSCH_RADAR_HDR_MSGS:
    BOSCH_RADAR_ALL_IDS.extend([_h + i for i in range(4)])

BOSCH_RADAR_HDR_TAG = 0x74          # b1 tag on range-carrier sub-frame
BOSCH_RADAR_LAT_SCALE_DEG_PER_LSB = 0.001  # shipped value; rlog-regressed band 0.0007-0.001
