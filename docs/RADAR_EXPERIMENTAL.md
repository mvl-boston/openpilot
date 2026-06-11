# radar-experimental — Long-Controller Upgrade Program

**Branch lineage:** `nrdr-staging-06.10.2026-radar` (radar-fine-0x280-ingest tip + boot guards,
prebuilt marker removed) + the 2026-06-10 long-controller program (15 beads, 8 feature commits,
squashed here). Companion feature branch with full per-bead history: `feat/long-m9-m5-live-tune`.

**Install:** `https://installer.comma.ai/internetadventuresllc/radar-experimental`
⚠️ First boot performs a FULL on-device build (~15–40 min: prebuilt marker absent; params_pyx.so
must bake `HondaCivicRadarTryout`, and the long_mpc.py edits trigger one acados solver codegen —
toolchain is vendored, this lineage has built it before). Do not power-cycle during the build.

---

## What this branch changes, in one paragraph

The radar signal is made honest at the source (per-slot Kalman filter on raw range replaces the
d/dt+EMA hack; tag-demux stops metadata frames from erasing live tracks), propagated honestly
through radard (measured-flag gating, NaN immunity, retuned gains, sweep-cadence freshness gate),
consumed intelligently by the MPC (vLeadK anchor, optional lead-decel extrapolation and obstacle
inflation), actuated through a gas learner that finally attributes effect to the command that
caused it (0.5 s lag alignment) with torqued-grade safety rails, plus a two-phase stopping
profile and persistent brake memory for stop-and-go feel — and every tuning knob lives in a
live-reloaded JSON file at `/data/nrdr_long_tune.json`, so road tuning needs zero flashes and
zero new param keys (the params_keys.h boot-brick class is structurally avoided).

## Bead map (what / where / commit on the feature branch)

| Bead | Name | Files | Commit |
|---|---|---|---|
| M9 (7u2.1) | `/data` JSON live-tune transport | `selfdrive/controls/lib/nrdr_long_tune.py` | 7a1ed77 |
| M5 (7u2.2) | Per-personality (a_change, j_ego) jerk weights | `long_mpc.py` | 7a1ed77 |
| K1 (86t.3) | Honor measured flag; KF coast; NaN hardening | `selfdrive/controls/radard.py` | 3cce797 |
| D1 (86t.1) | vl_all tag-demux assembler | `opendbc…/honda/radar_interface.py` | 8e580e1 |
| R1 (86t.4) | Per-slot [range, range_rate] source KF | same | 8e580e1 |
| K2 (86t.5) | radard DARE gains + freshness gate (atomic w/ R1) | `radard.py` | 87e54e2 |
| M1 (7u2.4) | MPC anchors v_lead_traj at vLeadK | `long_mpc.py` | dc378e0 |
| M2 (7u2.5) | aLeadK·τ decay lead extrapolation (default OFF) | `long_mpc.py` | dc378e0 |
| M3 (7u2.6) | Lead-decel obstacle inflation b_eff (default OFF) | `long_mpc.py` | dc378e0 |
| G1 (poy.3) | Lag-aligned gas learner (0.5 s deque) | `opendbc…/honda/carcontroller.py` | 021b740 |
| G4 (poy.4) | Learner safety rails (clamps/decay/NaN/fingerprint) | same | 021b740 |
| L2 (cra.1) | Two-phase stopping + pitch hold floor | `longcontrol.py`, `controlsd.py` | 396aa06 |
| G3 (cra.3) | Persistent speed-binned brake memory | `carcontroller.py` | 396aa06 |
| D6 (2td.1) | Offline radar bit-discovery harness | `tools/nrdr_radar_re/` (never ships) | 6692c4e |
| 6u8 | S6 re-seed vRel-halving bug | subsumed by R1 | 8e580e1 |

## The live tune file — `/data/nrdr_long_tune.json`

Read `selfdrive/controls/lib/nrdr_long_tune.py`'s docstring for the authoritative field
reference (clamps, defaults, semantics). Summary:

- **Missing/corrupt file = compiled stock behavior** (provable no-op; this is tested).
- Every field hard-clamped in code; NaN/inf rejected; atomic-write CLI included:
  `python3 selfdrive/controls/lib/nrdr_long_tune.py set comfort_brake=2.3` (over SSH),
  `… show`, `… reset`.
- Sections: gap/feel (`comfort_brake`, `stop_distance`, `t_follow_offsets`,
  `a_cruise_max_scale`), jerk (`jerk_factors`, `low_speed_jerk_scale`),
  lead consumption (`m1_anchor` **ON by default**, `m1_alead_escape`, `m2_*` OFF, `m3_*` OFF),
  stopping (`l2_enable` ON, `hold_accel`, `phase_switch_v`, `proximity_scale_m`, `pitch_margin`).

### Defaults that are NOT stock (deliberate, council-adjudicated)
1. `m1_anchor=1` — MPC lead-speed anchor is the Kalman estimate (the honest signal now).
   Set `lead_consumption.m1_anchor=0` to revert to raw vLead.
2. `stopping.l2_enable=1` — two-phase stopping profile active below `vEgoStopping`.
   Set `stopping.l2_enable=0` to revert to the stock monotonic ramp.
3. radard behavior changes (K1/K2/R1/D1) are not tune-gated — they are correctness fixes.
   Rollback = revert the branch.

## Road-test protocol (recommended order)

1. **Shakedown (defaults):** no tune file. Verifies M1 anchor + L2 stopping + honest-vRel chain
   on a normal drive. Watch: steady highway following (no gap hunting), smooth final stop
   (no head-bob), no phantom brake on overpasses/parked cars.
2. **Option A stopping params (bead 78z):** UI params `HondaVEgoStopping=0.30`,
   `HondaStoppingDecelRate=0.18`, `HondaStoppingDecelRateLong=0.20`, `LongPidTuneScale=90`.
3. **Gap/jerk session:** `comfort_brake` 2.5→2.3→2.2 (tighter highway gap), M5 jerk values
   (`jerk_factors.relaxed.a_change=1.4 j_ego=1.5`, `aggressive 0.6/0.5`, `low_speed_jerk_scale=1.3`).
4. **M2+M3 JOINT session (never separately — they pull braking earlier off the same signal):**
   start `m2_w_max=0.3`, `m3_b_eff_max=3.0`; evaluate behind decelerating leads.
5. **Grade testing for L2:** steepest local hill, foot over brake; the hold floor is hard-capped
   at -1.0 m/s² minimum until this passes (then the cap can be revisited in code).

## Safety posture

- No new params_keys.h keys (boot-brick class structurally avoided).
- Codegen identity AST-guarded: `gen_long_ocp()` still emits the committed solver; the one
  acados regen at first boot reproduces it.
- All persistent learned state is bounded and self-healing: hard+soft clamps, decay-back,
  NaN→reset, fingerprint+LEARN_VERSION reset. Brake memory preloads bounded integrator state
  (≤0.5 m/s²), never a multiplier. No CMBS backstop exists under op-long — caps are conservative.
- Firmware untouched (Bundle G-fw is separate, pending operator review).

## Verification

~370 offline tests, runnable off-device (see the test files next to each module; Windows-host
invocation documented in the session memory `nrdr-offline-test-harness`). Highlights: 50
end-to-end radar-interface tests through the real CANParser; sim-tuned KF receipts in
`radar_interface.py` comments; analytic-vs-numeric integral proof for M2; highway bit-identity
proof for L2; known-answer tests for D6 (re-derives azimuth, re-falsifies range-rate).

## Tesla radar forward-compatibility

The consumption chain (source KF → honest radard → vLeadK/aLeadK-consuming MPC) is the
architecture a native-vRel radar drops into: replace `_SlotRangeKF.update()`'s measurement
model with the Tesla DBC's measured range-rate (R from its datasheet noise), keep everything
downstream unchanged. Prep beads: `iri` (community Civic implementations), `k8a` (VIN-learn
kill-gate bench test).
