# NRDR `radar-experimental` — Current State

_Last updated: 2026-06-11 • branch tip: `b3c3958` • device: comma four "mici"_

## TL;DR
- **plannerd crash FIXED** (`b3c3958`), deployed to the device in place, **pending on-car validation**.
- `radar-experimental` @ `b3c3958` pushed to `origin` (`internetadventuresllc/openpilot`).
- MVL `radnew` compared against ours: `radnew` is a **strict ancestor** of our branch; nuanced over-braking verdict (below).

---

## 1. The bug that was fixed (engagement-blocking)
**Symptom:** with alpha-long + experimental enabled, the car "wouldn't do anything" — no braking, no steering.

**Root cause:** `plannerd` crashed on the first engaged `modelV2` frame, every restart:
```
long_mpc.py:331  PERSONALITY_NAMES.get(int(personality))
TypeError: int() ... not 'capnp.lib.capnp._DynamicEnum'
```
`personality` (`sm['selfdriveState'].personality`) is a capnp `_DynamicEnum` with no `__int__`. plannerd death → `processNotRunning`/`commIssue` (no `longitudinalPlan`) → openpilot **never engages**. Bug introduced in `f1ebb6e` (M5 jerk tuning). Confirmed via `/data/community/crashes/error.log` (4 identical timestamps).

**Fix (`b3c3958`):** `long_mpc.py:331` and `:500` →
```python
PERSONALITY_NAMES.get(personality.raw if hasattr(personality, 'raw') else int(personality))
```
`.raw` = the ordinal (matches `PERSONALITY_NAMES` keys); `hasattr` guard keeps int/static-enum (unit-test) callers working. Pure Python → **no device rebuild**. Hardware-verified: `int()` reproduces the TypeError; `.raw` resolves aggressive/standard/relaxed correctly.

**Deploy state:** fixed file written in place on device (md5-verified `14489ce…`), backup at `/data/long_mpc.py.bak`. Device offroad → plannerd starts clean from the fixed file on next onroad (no restart needed).

**False leads ruled out (for the record):**
- "alpha-long wasn't enabled" — wrong; `openpilotLongitudinalControl=True`, gate was open.
- "`sched_setaffinity` Errno 22" — a cgroup/core-gating artifact (comma four power-gates cores 4-7; standalone-SSH repro only). The bare affinity call works when plannerd runs during driving. **NOT** the production crash.

---

## 2. MVL `radnew` vs our `radar-experimental`
**Relationship:** `merge-base(mvl/radnew, radar-experimental) = bfc5716` = **MVL's tip**. `radnew` is a strict ancestor; **ours = MVL + 11 commits**. The substantive delta is `f1ebb6e` (15-bead long-controller + radar rework).

| Bug | MVL `radnew` | Our `radar-experimental` |
|---|---|---|
| plannerd personality crash | never had it (stock `==` enum) | born in `f1ebb6e`, **fixed** in `b3c3958` |
| Phantom leads (brake for nothing) | shared suppression stack | **UNCHANGED** — no gate we add that they lack; D1 stickiness may brake *sooner* on borderline returns |
| Mistune (over-brake real leads) | noisy raw-EMA vLead | **IMPROVED on noise, neutral on hard-decel** |

**Bottom line:** We do **not** fix a phantom-*ingestion* bug (the full suppression stack is in the shared base MVL already runs; `f1ebb6e` adds zero new validity gate). We **improve** the mistune mode **if** MVL's over-braking is EMA-noise-driven (R1 Kalman vRel replaces the `alpha=0.5` raw EMA → fewer `vLead²` chatter brake spikes), but only in the low/steady regime — the M1 escape hatch reverts to the laggy raw signal on hard-decel. At shipped defaults the obstacle math is byte-identical to MVL. **Need MVL rlogs to disambiguate phantom vs mistune before claiming a fix.**

**Port plan:** Nothing to port *from* MVL (strict ancestor). To help MVL's over-braking, port **`f1ebb6e` + `b3c3958` as a unit** — `f1ebb6e` alone transplants the plannerd crash MVL doesn't currently have.

---

## 3. New risks found in our rework (beads filed)
- **`nrdrbranchdebug-9jz` (P1):** M1 vLeadK escape-hatch **under-brake** risk on hard-decel leads (`long_mpc.py:385-389`, default ON, unverified on-car). Under-braking is worse than over-braking — **verify on-car.**
- **`nrdrbranchdebug-993` (P2):** M3 `lead_b_eff` cross-track-swap latch → spurious over-brake for ~0.5s on a lead swap (`long_mpc.py:467-496`). Gated OFF at default (`m3_b_eff_max=2.5`); fix before enabling M3 or porting.
- **`nrdrbranchdebug-87o` (P3):** M1 crossfade silently stuck on raw vLead under `radarTrackId` churn (`long_mpc.py:401-407`) → nullifies KF-anchor benefit invisibly.
- **`nrdrbranchdebug-599` (P2):** MVL compare + port plan + rlog disambiguation (this analysis).
- **`nrdrbranchdebug-g05` (in_progress):** the plannerd fix itself — awaiting on-car engage+brake validation.

---

## 4. Next steps
1. **On-car validation** (parked → low speed, ready to take over): confirm op-long engages and brakes. Log `radarState` + `longitudinalPlan` — watch **both** over-braking (phantom/mistune) and **under-braking** (M1 escape hatch on hard-decel leads).
2. **Get MVL rlogs** → correlate over-brake events with `radarState.leadOne` to settle phantom-vs-mistune.
3. **Lateral "pid-tune-only" not active on device:** `NrdrLearnSteerRatio/Stiffness/AngleOffset` + `HondaTorqueLowPassFilter` read `=1` (PERSISTENT params already set; a new default doesn't overwrite). Needs `params.put` on device to take effect. Fresh installs *do* get the pid-tune-only defaults.

## 5. Device/branch facts
- Device "mici" @ `192.168.50.74`, on `radar-experimental`, running file content = `b3c3958` (long_mpc patched in place); `prebuilt` marker **present** (instant boot; rebuilt artifacts intact — do NOT `git reset` the device tree).
- Device python with capnp: `/usr/local/venv/bin/python` (`PYTHONPATH=/data:/data/openpilot`). rlogs: `/data/media/0/realdata/<route>--N`.
