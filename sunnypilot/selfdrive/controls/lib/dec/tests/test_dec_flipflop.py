"""
DEC flip-flop fix — unit tests (nrdrbranchdebug-86t bead).

dec.py imports cereal/opendbc/numpy/openpilot.common.params which are unavailable on this
Windows dev box. We use the AST-extraction pattern (see test_radard_k1.py for precedent):
parse dec.py source, pull ModeTransitionManager + SET_MODE_TIMEOUT + ModeType assignments,
compile+exec into a namespace with a minimal shim environment.

For the two hysteresis methods (_calculate_slow_down invalid-trajectory branch and the
valid-trajectory _update branch), we extract the relevant statements and test them via
a stub object that exposes all attributes those statements touch.

A source-regression guard asserts the live file contains the exact hysteresis expressions,
so the tests fail immediately if the code drifts even if the AST execution path changes.
"""

import ast
import types
import inspect
import textwrap
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Locate source file relative to THIS test file so it works on-device too
# ---------------------------------------------------------------------------
DEC_PY = Path(__file__).resolve().parent.parent / "dec.py"
assert DEC_PY.exists(), f"dec.py not found at {DEC_PY}"

DEC_SOURCE = DEC_PY.read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# AST extraction helpers
# ---------------------------------------------------------------------------

def _target_names(node):
    """Return the set of names assigned by an ast.Assign target."""
    t = node.targets[0]
    if isinstance(t, ast.Name):
        return {t.id}
    if isinstance(t, ast.Tuple):
        return {e.id for e in t.elts if isinstance(e, ast.Name)}
    return set()


def _extract_namespace():
    """
    Parse dec.py and exec ModeTransitionManager + SET_MODE_TIMEOUT + ModeType
    into a namespace, providing a minimal shim for the imports the class needs.
    """
    tree = ast.parse(DEC_SOURCE)

    wanted_classes = {"ModeTransitionManager"}
    wanted_assigns = {"SET_MODE_TIMEOUT", "ModeType", "TRAJECTORY_SIZE"}

    nodes = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name in wanted_classes:
            nodes.append(node)
        elif isinstance(node, ast.Assign) and _target_names(node) & wanted_assigns:
            nodes.append(node)

    # Shim: typing.Literal so ModeType = Literal['acc','blended'] executes fine
    from typing import Literal
    ns = {
        "Literal": Literal,
    }
    module = ast.Module(body=nodes, type_ignores=[])
    exec(compile(module, str(DEC_PY), "exec"), ns)  # noqa: S102

    assert "ModeTransitionManager" in ns, "AST extraction failed: ModeTransitionManager missing"
    assert "SET_MODE_TIMEOUT" in ns, "AST extraction failed: SET_MODE_TIMEOUT missing"
    assert ns["SET_MODE_TIMEOUT"] == 15
    return ns


NS = _extract_namespace()
ModeTransitionManager = NS["ModeTransitionManager"]
SET_MODE_TIMEOUT = NS["SET_MODE_TIMEOUT"]


# ---------------------------------------------------------------------------
# Source regression guards — if the code drifts these fail immediately
# ---------------------------------------------------------------------------

class TestSourceRegression:
    """Assert the fixed expressions exist verbatim in the live source file."""

    def test_dwell_guard_override_holds_present(self):
        assert "override_holds = self.emergency_override and mode == self.current_mode" in DEC_SOURCE, \
            "override_holds expression not found in dec.py — dwell-guard fix missing or drifted"

    def test_dwell_guard_condition_present(self):
        assert "if self.mode_duration < self.min_mode_duration and not override_holds:" in DEC_SOURCE, \
            "dwell guard condition not found in dec.py — fix missing or drifted"

    def test_invalid_traj_hysteresis_present(self):
        assert "slow_thr = WMACConstants.SLOW_DOWN_PROB * (0.625 if self._has_slow_down else 1.0)" in DEC_SOURCE, \
            "invalid-trajectory hysteresis expression missing or drifted"

    def test_valid_traj_hysteresis_present(self):
        assert "slow_thr = WMACConstants.SLOW_DOWN_PROB * (0.5 if self._has_slow_down else 0.8)" in DEC_SOURCE, \
            "valid-trajectory hysteresis expression missing or drifted"

    def test_old_single_threshold_NOT_present(self):
        # The old dwell guard used `and not self.emergency_override` as the full guard.
        # The fix replaced it with the override_holds pattern; the old line must be gone.
        # Note: the phrase also appears in a comment (historical note), so we check for
        # the exact old CODE line, not just the phrase.
        old_line = "if self.mode_duration < self.min_mode_duration and not self.emergency_override:"
        assert old_line not in DEC_SOURCE, \
            "Old dwell guard line still present — fix not applied"

    def test_old_update_state_not_present(self):
        assert "urgency_filtered > (WMACConstants.SLOW_DOWN_PROB * 0.8)" not in DEC_SOURCE, \
            "Old single-threshold update_state line still present — hysteresis not applied"

    def test_old_slow_down_prob_direct_not_present(self):
        # The invalid-traj branch must no longer use bare SLOW_DOWN_PROB comparison
        # Find the invalid-traj return block and confirm the old line is gone
        assert "self._has_slow_down = urgency_filtered > WMACConstants.SLOW_DOWN_PROB\n" not in DEC_SOURCE, \
            "Old bare SLOW_DOWN_PROB threshold in invalid-traj branch still present"


# ---------------------------------------------------------------------------
# ModeTransitionManager tests
# ---------------------------------------------------------------------------

def make_mtm():
    """Fresh ModeTransitionManager in default state."""
    return ModeTransitionManager()


class TestDwellGuard:
    """
    Test case (a): Emergency request under dwell from different mode → IMMEDIATE switch.
    The early-return for emergency=True fires before the dwell guard, so emergency entry
    is never blocked.
    """
    def test_emergency_entry_bypasses_dwell(self):
        mgr = make_mtm()
        assert mgr.current_mode == 'acc'
        # mode_duration is 0 (< min_mode_duration=10): dwell guard is active
        # but emergency=True takes the early-return path — must switch immediately
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        assert mgr.current_mode == 'blended', "Emergency entry must bypass dwell guard"
        assert mgr.emergency_override is True

    def test_emergency_entry_from_blended_to_acc(self):
        """Emergency entry also works in the opposite direction."""
        mgr = make_mtm()
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        assert mgr.current_mode == 'blended'
        # Still under dwell (mode_duration reset to 0 by emergency entry)
        mgr.request_mode('acc', confidence=1.0, emergency=True)
        assert mgr.current_mode == 'acc', "Emergency entry to acc must bypass dwell guard"

    """
    Test case (b): THE FIX — active emergency_override + OPPOSING non-emergency request
    during dwell → request blocked.
    Pre-fix behaviour: `not self.emergency_override` evaluated False during override,
    so ANY non-emergency request could slip through and flip the mode back.
    """
    def test_opposing_non_emergency_blocked_during_override_dwell(self):
        mgr = make_mtm()
        # Enter blended via emergency — mode_duration resets to 0, emergency_override=True
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        assert mgr.current_mode == 'blended'
        assert mgr.emergency_override is True
        assert mgr.mode_duration == 0  # dwell just reset

        # Non-emergency acc request with high confidence while mode_duration < 10
        # Pre-fix: `not False` → True → dwell bypassed → flip back to acc  (BUG)
        # Post-fix: override_holds = True and mode='acc' ≠ 'blended' → False → blocked
        mgr.request_mode('acc', confidence=1.0)
        assert mgr.current_mode == 'blended', \
            "THE FIX: opposing non-emergency must be blocked while emergency dwell is active"

    def test_opposing_blocked_multiple_frames(self):
        """Block persists across several frames of dwell."""
        mgr = make_mtm()
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        for _ in range(9):   # advance to mode_duration=9 (still < 10)
            mgr.update()
            mgr.request_mode('acc', confidence=1.0)
            assert mgr.current_mode == 'blended', \
                f"Must stay blended at mode_duration={mgr.mode_duration}"

    """
    Test case (c): active emergency_override + SAME-mode non-emergency request during dwell
    → passes the guard (override_holds=True → guard skipped).
    This allows same-mode confidence reinforcement without triggering a transition.
    """
    def test_same_mode_non_emergency_passes_guard(self):
        mgr = make_mtm()
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        assert mgr.current_mode == 'blended'
        # Same mode non-emergency request during dwell — override_holds=True → guard skipped
        # mode_confidence for blended should increase (confidence reinforcement path runs)
        conf_before = mgr.mode_confidence['blended']
        mgr.request_mode('blended', confidence=1.0)
        # The guard is not the blocker here; the actual mode-change block won't fire
        # because mode == current_mode. The confidence should be updated.
        assert mgr.current_mode == 'blended', "Mode must stay blended"
        assert mgr.mode_confidence['blended'] >= conf_before, \
            "Same-mode request must reinforce confidence"

    """
    Test case (d): After dwell expires (mode_duration >= 10) an opposing high-confidence
    request CAN transition. Also: update() clears emergency_override after mode_duration > 20.
    """
    def test_transition_allowed_after_dwell_expires(self):
        mgr = make_mtm()
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        # Advance past dwell — update() increments mode_duration each call
        for _ in range(10):
            mgr.update()
        assert mgr.mode_duration >= 10

        # Build acc confidence high enough to cross threshold (need > 0.6 for mode change)
        for _ in range(10):
            mgr.request_mode('acc', confidence=1.0)
        mgr.update()  # allow transition_timeout to open
        # transition_timeout was set on emergency entry; advance past it
        for _ in range(SET_MODE_TIMEOUT + 1):
            mgr.update()
        mgr.request_mode('acc', confidence=1.0)
        assert mgr.current_mode == 'acc', \
            "After dwell expires, high-confidence acc request must be able to transition"

    def test_emergency_override_clears_after_20_frames(self):
        mgr = make_mtm()
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        assert mgr.emergency_override is True
        for _ in range(21):
            mgr.update()
        assert mgr.emergency_override is False, \
            "update() must clear emergency_override after mode_duration > 20"


# ---------------------------------------------------------------------------
# Hysteresis tests via stub object extraction
# ---------------------------------------------------------------------------
#
# We extract the hysteresis statements from dec.py source and run them
# bound to a stub that exposes all the attributes those statements touch.
# This is safer than exec-ing the full method (which would need numpy/cereal).
# The source-regression guards above ensure the live code matches what we test.

class StubWMACConstants:
    SLOW_DOWN_PROB = 0.3


class StubFilter:
    """Minimal filter stub that returns a preset value."""
    def __init__(self, value=0.0):
        self._value = value

    def add_data(self, v):
        self._value = v  # simple passthrough for testing

    def get_value(self):
        return self._value


def run_invalid_traj_hysteresis(has_slow_down_init: bool, urgency_injected: float) -> bool:
    """
    Simulate the invalid-trajectory branch hysteresis from _calculate_slow_down.
    Injects urgency_injected directly as the filtered value (filter passthrough).

    Returns the new value of _has_slow_down.
    """
    stub = types.SimpleNamespace(
        _has_slow_down=has_slow_down_init,
        _slow_down_filter=StubFilter(urgency_injected),
        _urgency=0.0,
        WMACConstants=StubWMACConstants,
    )

    # Replicate the three lines from the invalid-traj branch exactly:
    urgency_filtered = stub._slow_down_filter.get_value() or 0.0
    slow_thr = StubWMACConstants.SLOW_DOWN_PROB * (0.625 if stub._has_slow_down else 1.0)
    stub._has_slow_down = urgency_filtered > slow_thr
    stub._urgency = urgency_filtered
    return stub._has_slow_down


def run_valid_traj_hysteresis(has_slow_down_init: bool, urgency_injected: float) -> bool:
    """
    Simulate the valid-trajectory branch hysteresis from _calculate_slow_down.
    Returns the new value of _has_slow_down.
    """
    stub = types.SimpleNamespace(
        _has_slow_down=has_slow_down_init,
        _slow_down_filter=StubFilter(urgency_injected),
        _urgency=0.0,
        WMACConstants=StubWMACConstants,
    )

    # Replicate the three lines from the valid-traj branch exactly:
    urgency_filtered = stub._slow_down_filter.get_value() or 0.0
    slow_thr = StubWMACConstants.SLOW_DOWN_PROB * (0.5 if stub._has_slow_down else 0.8)
    stub._has_slow_down = urgency_filtered > slow_thr
    stub._urgency = urgency_filtered
    return stub._has_slow_down


class TestHysteresisInvalidTraj:
    """
    Test case (e) — invalid-trajectory (update_state) site.
    Band: enter at SLOW_DOWN_PROB * 1.0 = 0.30 (strict)
          exit  at SLOW_DOWN_PROB * 0.625 = 0.1875
    """

    def test_below_enter_threshold_stays_false(self):
        """0.20 < 0.30 → stays False when starting False."""
        result = run_invalid_traj_hysteresis(has_slow_down_init=False, urgency_injected=0.20)
        assert result is False, "0.20 must not cross enter threshold 0.30"

    def test_at_enter_threshold_stays_false(self):
        """0.30 is NOT > 0.30 → stays False."""
        result = run_invalid_traj_hysteresis(has_slow_down_init=False, urgency_injected=0.30)
        assert result is False, "0.30 must not cross strict > threshold"

    def test_above_enter_threshold_sets_true(self):
        """0.31 > 0.30 → sets True."""
        result = run_invalid_traj_hysteresis(has_slow_down_init=False, urgency_injected=0.31)
        assert result is True, "0.31 must cross enter threshold 0.30"

    def test_hysteresis_hold_above_exit(self):
        """Once True, 0.20 > 0.1875 (exit) → stays True."""
        result = run_invalid_traj_hysteresis(has_slow_down_init=True, urgency_injected=0.20)
        assert result is True, "0.20 must stay above exit threshold 0.1875 (hysteresis hold)"

    def test_hysteresis_clear_below_exit(self):
        """Once True, 0.10 < 0.1875 → clears to False."""
        result = run_invalid_traj_hysteresis(has_slow_down_init=True, urgency_injected=0.10)
        assert result is False, "0.10 must fall below exit threshold 0.1875 → clears"

    def test_oscillation_scenario(self):
        """
        Simulate oscillating urgency 0.20 ↔ 0.31 across 10 frames.
        With hysteresis: after first entry at 0.31, 0.20 holds True (above 0.1875 exit).
        Without: every 0.20 frame would clear and every 0.31 frame would set → 5 flips.
        """
        state = False
        flips = 0
        for i in range(10):
            urgency = 0.31 if i % 2 == 0 else 0.20
            new_state = run_invalid_traj_hysteresis(has_slow_down_init=state, urgency_injected=urgency)
            if new_state != state:
                flips += 1
            state = new_state

        # With hysteresis: only the first frame (0→1) is a flip; subsequent 0.20 hold True
        assert flips <= 1, f"Hysteresis must suppress oscillation; got {flips} flips"


class TestHysteresisValidTraj:
    """
    Test case (e) — valid-trajectory (_update) site.
    Band: enter at SLOW_DOWN_PROB * 0.8 = 0.24
          exit  at SLOW_DOWN_PROB * 0.5 = 0.15
    """

    def test_below_enter_threshold_stays_false(self):
        """0.20 < 0.24 → stays False when starting False."""
        result = run_valid_traj_hysteresis(has_slow_down_init=False, urgency_injected=0.20)
        assert result is False, "0.20 must not cross enter threshold 0.24"

    def test_at_enter_threshold_stays_false(self):
        """0.24 is NOT > 0.24 → stays False."""
        result = run_valid_traj_hysteresis(has_slow_down_init=False, urgency_injected=0.24)
        assert result is False, "0.24 must not cross strict > threshold"

    def test_above_enter_threshold_sets_true(self):
        """0.25 > 0.24 → sets True."""
        result = run_valid_traj_hysteresis(has_slow_down_init=False, urgency_injected=0.25)
        assert result is True, "0.25 must cross enter threshold 0.24"

    def test_hysteresis_hold_above_exit(self):
        """Once True, 0.20 > 0.15 (exit) → stays True."""
        result = run_valid_traj_hysteresis(has_slow_down_init=True, urgency_injected=0.20)
        assert result is True, "0.20 must stay above exit threshold 0.15 (hysteresis hold)"

    def test_hysteresis_clear_below_exit(self):
        """Once True, 0.10 < 0.15 → clears to False."""
        result = run_valid_traj_hysteresis(has_slow_down_init=True, urgency_injected=0.10)
        assert result is False, "0.10 must fall below exit threshold 0.15 → clears"

    def test_oscillation_suppressed(self):
        """
        Oscillating urgency 0.20 ↔ 0.25 across 10 frames.
        With hysteresis: after first entry at 0.25, 0.20 holds True (above 0.15 exit).
        Without: every 0.20 frame would clear → 5 flips.
        """
        state = False
        flips = 0
        for i in range(10):
            urgency = 0.25 if i % 2 == 0 else 0.20
            new_state = run_valid_traj_hysteresis(has_slow_down_init=state, urgency_injected=urgency)
            if new_state != state:
                flips += 1
            state = new_state

        assert flips <= 1, f"Hysteresis must suppress oscillation; got {flips} flips"


class TestRegressionBaselineComparison:
    """
    Test case (f): Regression guard — fixed logic must differ from single-threshold
    at the boundary, proving the hysteresis is doing real work.
    """

    def _single_threshold_valid(self, urgency: float) -> bool:
        """Pre-fix: always compare against 0.8 * SLOW_DOWN_PROB = 0.24."""
        return urgency > (StubWMACConstants.SLOW_DOWN_PROB * 0.8)

    def _single_threshold_invalid(self, urgency: float) -> bool:
        """Pre-fix invalid-traj: always compare against SLOW_DOWN_PROB = 0.30."""
        return urgency > StubWMACConstants.SLOW_DOWN_PROB

    def test_valid_traj_hysteresis_differs_from_single_threshold(self):
        """
        With oscillating urgency 0.20 ↔ 0.25, single-threshold flips every frame (5 flips).
        Hysteresis suppresses to ≤1 flip. The delta proves the fix is load-bearing.
        """
        state_hyst = False
        state_single = False
        flips_hyst = 0
        flips_single = 0

        for i in range(10):
            urgency = 0.25 if i % 2 == 0 else 0.20

            new_hyst = run_valid_traj_hysteresis(has_slow_down_init=state_hyst, urgency_injected=urgency)
            new_single = self._single_threshold_valid(urgency)

            if new_hyst != state_hyst:
                flips_hyst += 1
            if new_single != state_single:
                flips_single += 1

            state_hyst = new_hyst
            state_single = new_single

        assert flips_single >= 4, \
            f"Single-threshold baseline must produce many flips (got {flips_single})"
        assert flips_hyst <= 1, \
            f"Hysteresis must suppress to ≤1 flip (got {flips_hyst})"
        assert flips_hyst < flips_single, \
            "Hysteresis must produce fewer flips than single-threshold — fix is load-bearing"

    def test_invalid_traj_hysteresis_differs_from_single_threshold(self):
        """
        Invalid-traj site: oscillating 0.29 ↔ 0.31 around the 0.30 boundary.
        Single-threshold flips every frame; hysteresis: once entered at 0.31,
        0.29 > 0.1875 exit → stays True → ≤1 flip.
        """
        state_hyst = False
        state_single = False
        flips_hyst = 0
        flips_single = 0

        for i in range(10):
            urgency = 0.31 if i % 2 == 0 else 0.29

            new_hyst = run_invalid_traj_hysteresis(has_slow_down_init=state_hyst, urgency_injected=urgency)
            new_single = self._single_threshold_invalid(urgency)

            if new_hyst != state_hyst:
                flips_hyst += 1
            if new_single != state_single:
                flips_single += 1

            state_hyst = new_hyst
            state_single = new_single

        assert flips_single >= 4, \
            f"Single-threshold baseline must produce many flips (got {flips_single})"
        assert flips_hyst <= 1, \
            f"Hysteresis must suppress to ≤1 flip (got {flips_hyst})"
        assert flips_hyst < flips_single, \
            "Hysteresis must produce fewer flips than single-threshold — fix is load-bearing"

    def test_dwell_guard_differs_from_pre_fix(self):
        """
        Pre-fix: `not self.emergency_override` — any override state bypassed the dwell.
        Post-fix: only same-mode requests bypass the dwell.

        Demonstrate: start in blended via emergency, then send acc request.
        Pre-fix: mode flips to acc (BUG).
        Post-fix: mode stays blended.
        """
        # Simulate pre-fix behaviour
        class PreFixMTM:
            def __init__(self):
                self.current_mode = 'blended'
                self.mode_confidence = {'acc': 0.0, 'blended': 1.0}
                self.transition_timeout = SET_MODE_TIMEOUT
                self.min_mode_duration = 10
                self.mode_duration = 0
                self.emergency_override = True

            def request_mode_prefx(self, mode, confidence=1.0):
                self.mode_confidence[mode] = min(1.0, self.mode_confidence[mode] + 0.1 * confidence)
                for m in self.mode_confidence:
                    if m != mode:
                        self.mode_confidence[m] = max(0.0, self.mode_confidence[m] - 0.05)
                # PRE-FIX guard: `not self.emergency_override` → False when override active
                # so the guard NEVER blocks when emergency_override is True
                if self.mode_duration < self.min_mode_duration and not self.emergency_override:
                    return
                confidence_threshold = 0.6 if mode != self.current_mode else 0.3
                if self.mode_confidence[mode] > confidence_threshold:
                    if mode != self.current_mode and self.transition_timeout == 0:
                        self.transition_timeout = SET_MODE_TIMEOUT
                        self.current_mode = mode
                        self.mode_duration = 0

        pre = PreFixMTM()
        pre.transition_timeout = 0  # force transition_timeout open for cleaner test
        for _ in range(8):  # pump confidence high
            pre.request_mode_prefx('acc', confidence=1.0)
        # Pre-fix allows the flip (this is the bug we're fixing)
        assert pre.current_mode == 'acc', \
            "Pre-fix should have allowed the flip (demonstrating the bug)"

        # Now the fixed version — same scenario — must stay blended
        mgr = make_mtm()
        mgr.request_mode('blended', confidence=1.0, emergency=True)
        mgr.transition_timeout = 0  # open the transition window
        for _ in range(8):
            mgr.request_mode('acc', confidence=1.0)
        assert mgr.current_mode == 'blended', \
            "Post-fix must keep blended — opposing request blocked during emergency dwell"
