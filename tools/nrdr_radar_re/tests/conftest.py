# Local conftest for nrdr_radar_re tests.
# Prevents the root conftest.py (which requires on-device openpilot modules) from
# breaking this suite when run on Windows / without the full openpilot environment.
# pytest discovers this file first (nearest-to-tests wins) and uses it; the root
# conftest's autouse fixtures are NOT applied here because we do not import them.
collect_ignore_glob = []
