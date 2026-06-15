from carkit_behavior.behavior_logic import (
    RoadRuleStateMachine,
    StopSignPhase,
    in_trigger_zone,
    stopping_distance,
)


def make_state_machine():
    return RoadRuleStateMachine(
        stop_speed_threshold=0.05,
        stop_hold_seconds=3.0,
        stop_cooldown_seconds=5.0,
        stop_rearm_absence_seconds=1.0,
    )


def test_red_stops_immediately_and_green_releases():
    state = make_state_machine()
    state.observe(True, False, False, False, 0.0)
    assert state.stop_active
    assert state.state_name == "RED_LIGHT_STOP"

    state.observe(False, True, False, False, 0.1)
    assert not state.stop_active
    assert state.state_name == "CLEAR"


def test_stop_sign_brakes_holds_and_enters_cooldown():
    state = make_state_machine()
    state.observe(False, False, True, True, 0.0)
    assert state.stop_sign_phase == StopSignPhase.BRAKING
    assert state.stop_active

    state.update(0.04, 1.0)
    assert state.stop_sign_phase == StopSignPhase.HOLD
    state.update(0.0, 3.9)
    assert state.stop_active
    state.update(0.0, 4.0)
    assert state.stop_sign_phase == StopSignPhase.COOLDOWN
    assert not state.stop_active


def test_stop_sign_requires_cooldown_and_absence_to_rearm():
    state = make_state_machine()
    state.observe(False, False, True, True, 0.0)
    state.update(0.0, 0.1)
    state.update(0.0, 3.1)
    state.observe(False, False, False, False, 4.0)
    state.update(0.0, 8.0)
    assert state.stop_sign_phase == StopSignPhase.COOLDOWN
    state.update(0.0, 8.1)
    assert state.stop_sign_phase == StopSignPhase.IDLE


def test_reset_clears_all_stops():
    state = make_state_machine()
    state.observe(True, False, True, True, 0.0)
    state.reset()
    assert not state.stop_active
    assert state.stop_sign_phase == StopSignPhase.IDLE


def test_stopping_distance_uses_reaction_braking_and_margin():
    assert stopping_distance(1.0, 0.3, 1.0, 0.6) == 1.4


def test_stopping_distance_rejects_nonpositive_deceleration():
    try:
        stopping_distance(1.0, 0.3, 0.0, 0.6)
    except ValueError:
        return
    raise AssertionError("nonpositive deceleration must be rejected")


def test_invalid_position_cannot_enter_trigger_zone():
    assert not in_trigger_zone(False, 0.0, 0.5, 1.0, 1.0)
    assert in_trigger_zone(True, 0.0, 0.5, 1.0, 1.0)
