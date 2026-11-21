#!/usr/bin/env python3
import os
import subprocess

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.scons_cache import purge_zero_byte_artifacts
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.hardware import HARDWARE, AGNOS

SCONS_CACHE_DIR = "/data/scons_cache" if AGNOS else "/tmp/scons_cache"


def _looks_like_link_failure(compile_output: list[bytes]) -> bool:
  text = b"\n".join(compile_output)
  return b"undefined reference" in text or b"collect2: error" in text


def _run_scons(args: list[str], spinner: Spinner, compile_output: list[bytes]) -> int:
  compile_output.clear()
  with subprocess.Popen(["scons", *args], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR}, stderr=subprocess.PIPE) as scons:
    assert scons.stderr is not None

    # Read progress from stderr and update spinner
    while scons.poll() is None:
      try:
        line = scons.stderr.readline()
        if line is None:
          continue
        line = line.rstrip()

        prefix = b'progress: '
        if line.startswith(prefix):
          progress = float(line[len(prefix):])
          spinner.update_progress(100 * min(1., progress / 100.), 100.)
        elif len(line):
          compile_output.append(line)
          print(line.decode('utf8', 'replace'))
      except Exception:
        pass

    # Drain and close the pipe before retrying or returning.
    for line in scons.stderr.read().split(b'\n'):
      line = line.rstrip()
      if len(line):
        compile_output.append(line)

  return scons.returncode


def build() -> None:
  spinner = Spinner()
  spinner.update_progress(0, 100)

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # building with all cores can result in using too much memory, so retry serially
  compile_output: list[bytes] = []
  returncode = 1
  for parallelism in ([], ["-j4"], ["-j1"]):
    returncode = _run_scons(parallelism, spinner, compile_output)
    if returncode == 0:
      break

  # An interrupted earlier build can leave 0-byte objects that scons keeps
  # restoring from its cache, so linking fails on every boot. Purge and retry
  # once so the device recovers without manual intervention.
  if returncode != 0 and _looks_like_link_failure(compile_output):
    removed = purge_zero_byte_artifacts(BASEDIR, SCONS_CACHE_DIR)
    if removed:
      print(f"removed {removed} zero-byte build artifacts, retrying build")
      returncode = _run_scons(["-j4"], spinner, compile_output)

  os.sync()

  if returncode != 0:
    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

if __name__ == "__main__":
  build()
