#!/usr/bin/env python3
"""Replay LKAS HUD CAN from a route segment and report RDM_OFF on openpilot TX."""
import argparse
from types import SimpleNamespace

import zstandard as zstd

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.values import DBC, CarControllerParams, HondaFlags
from openpilot.cereal import log as capnp_log


def decompress_stream(data: bytes) -> bytes:
  dctx = zstd.ZstdDecompressor()
  with dctx.stream_reader(data) as reader:
    return reader.read()


def decode_rdm_off(parser: CANParser, msg_name: str) -> int:
  return int(parser.vl[msg_name]['RDM_OFF'])


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

    dbc_name = DBC[CP.carFingerprint][Bus.pt]
    packer = CANPacker(dbc_name)
    CAN = hondacan.CanBus(CP)
    params = CarControllerParams(CP)

    # LKAS HUD is on PT bus when alphalong (see CanBus)
    hud_bus = CAN.lkas
    if CP.flags & HondaFlags.BOSCH_EXT_HUD and not CP.openpilotLongitudinalControl:
      hud_msgs = [('LKAS_HUD_A', hud_bus), ('LKAS_HUD_B', hud_bus)]
    else:
      hud_msgs = [('LKAS_HUD', hud_bus)]
    can_parser = CANParser(dbc_name, hud_msgs, hud_bus)

    t0 = min(e.logMonoTime for e in ents if e.which() == 'carState')
    latest_car = None
    hud_tx: list[tuple[float, int, int]] = []
    frame = 0

    CS = SimpleNamespace(out=SimpleNamespace(steerFaultPermanent=False), lkas_hud={'LKAS_PROBLEM': 0})

    for e in ents:
      if e.which() == 'carState':
        latest_car = e.carState
        CS.out.steerFaultPermanent = bool(latest_car.steerFaultPermanent)
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
        hud_control = SimpleNamespace(lanesVisible=CC.hudControl.lanesVisible)
        cmds = hondacan.create_lkas_hud(
          packer, CAN.lkas, CP, hud_control, CC.latActive,
          steering_available, False, False, CS.lkas_hud, True, False, CS,
        )
        for addr, dat, bus in cmds:
          can_parser.update([(0, [(addr, bytes(dat), bus)])])
          msg_name = packer.dbc.addr_to_msg[addr].name
          hud_tx.append((t, addr, decode_rdm_off(can_parser, msg_name)))
      frame += 1

  if not hud_tx:
    print('No LKAS HUD TX in window')
    return

  on = sum(v for _, _, v in hud_tx)
  print(f"LKAS HUD TX frames in [{args.t_start}, {args.t_end}]s: {len(hud_tx)}")
  print(f"RDM_OFF on replay TX: {on}/{len(hud_tx)} ({100 * on / len(hud_tx):.1f}%)")
  print('First samples:', hud_tx[:5])


if __name__ == '__main__':
  main()
