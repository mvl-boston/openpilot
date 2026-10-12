#!/usr/bin/env python3
"""Replay LKAS HUD CAN from a route segment and report RDM_OFF on openpilot TX."""
import argparse
import zstandard as zstd

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import DBC as HONDA_DBC, CarControllerParams
from openpilot.cereal import log as capnp_log


def decompress_stream(data: bytes) -> bytes:
  dctx = zstd.ZstdDecompressor()
  with dctx.stream_reader(data) as reader:
    return reader.read()


def decode_rdm_off(packer: CANPacker, address: int, dat: bytes) -> int:
  return int(packer.dbc.addr_to_msg[address].sigs['RDM_OFF'].get_raw_value(dat))


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('rlog', help='path to rlog.zst')
  parser.add_argument('--t-start', type=float, default=25.0)
  parser.add_argument('--t-end', type=float, default=45.0)
  args = parser.parse_args()

  with open(args.rlog, 'rb') as f:
    ents = list(capnp_log.Event.read_multiple_bytes(decompress_stream(f.read())))

  cp_raw = next(e for e in ents if e.which() == 'carParams').carParams.as_builder().to_bytes()
  with structs.CarParams.from_bytes(cp_raw) as CP:
    print(f"CP {CP.carFingerprint} flags={CP.flags} openpilotLongitudinalControl={CP.openpilotLongitudinalControl}")

    dbc_name = HONDA_DBC[CP.carFingerprint][Bus.pt]
    packer = CANPacker(dbc_name)
    CAN = hondacan.CanBus(CP)
    params = CarControllerParams(CP)

    t0 = min(e.logMonoTime for e in ents if e.which() == 'carState')
    latest_car = None
    hud_tx: list[tuple[float, int, int]] = []
    frame = 0

    for e in ents:
      if e.which() == 'carState':
        latest_car = e.carState
      if e.which() != 'carControl' or latest_car is None:
        continue

      t = (e.logMonoTime - t0) / 1e9
      if t < args.t_start or t > args.t_end:
        continue

      with structs.CarControl.from_bytes(e.carControl.as_builder().to_bytes()) as CC:
        if frame % 10 != 0:
          frame += 1
          continue

        steering_available = latest_car.cruiseState.available and latest_car.vEgo > max(
          params.STEER_GLOBAL_MIN_SPEED, CP.minSteerSpeed)
        cmds = hondacan.create_lkas_hud(
          packer, CAN.lkas, CP, CC.hudControl, CC.latActive,
          steering_available, False, {'LKAS_PROBLEM': 0},
        )
        for addr, dat, _bus in cmds:
          hud_tx.append((t, addr, decode_rdm_off(packer, addr, bytes(dat))))
      frame += 1

  if not hud_tx:
    print('No LKAS HUD TX in window')
    return

  on = sum(v for _, _, v in hud_tx)
  print(f"LKAS HUD TX frames in [{args.t_start}, {args.t_end}]s: {len(hud_tx)}")
  print(f"RDM_OFF on TX: {on}/{len(hud_tx)} ({100 * on / len(hud_tx):.1f}%)")
  print('First samples:', hud_tx[:5])


if __name__ == '__main__':
  main()
