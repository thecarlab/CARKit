# Copyright 2026 CARKit maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum


class StopSignPhase(Enum):
    IDLE = "CLEAR"
    BRAKING = "STOP_SIGN_BRAKING"
    HOLD = "STOP_SIGN_HOLD"
    COOLDOWN = "STOP_SIGN_COOLDOWN"


class RoadRuleStateMachine:
    def __init__(
        self,
        stop_speed_threshold: float,
        stop_hold_seconds: float,
        stop_cooldown_seconds: float,
        stop_rearm_absence_seconds: float,
    ) -> None:
        self.stop_speed_threshold = stop_speed_threshold
        self.stop_hold_seconds = stop_hold_seconds
        self.stop_cooldown_seconds = stop_cooldown_seconds
        self.stop_rearm_absence_seconds = stop_rearm_absence_seconds
        self.reset()

    def reset(self) -> None:
        self.red_latched = False
        self.stop_sign_phase = StopSignPhase.IDLE
        self.hold_started = None
        self.cooldown_until = None
        self.stop_sign_absent_since = None

    def observe(
        self,
        red_light: bool,
        green_light: bool,
        stop_sign: bool,
        stop_sign_visible: bool,
        now: float,
    ) -> None:
        if green_light:
            self.red_latched = False
        if red_light:
            self.red_latched = True

        if stop_sign_visible:
            self.stop_sign_absent_since = None
        elif self.stop_sign_absent_since is None:
            self.stop_sign_absent_since = now

        if stop_sign and self.stop_sign_phase == StopSignPhase.IDLE:
            self.stop_sign_phase = StopSignPhase.BRAKING
            self.hold_started = None

    def update(self, speed: float, now: float) -> None:
        if (
            self.stop_sign_phase == StopSignPhase.BRAKING
            and abs(speed) <= self.stop_speed_threshold
        ):
            self.stop_sign_phase = StopSignPhase.HOLD
            self.hold_started = now

        if (
            self.stop_sign_phase == StopSignPhase.HOLD
            and self.hold_started is not None
            and now - self.hold_started >= self.stop_hold_seconds
        ):
            self.stop_sign_phase = StopSignPhase.COOLDOWN
            self.cooldown_until = now + self.stop_cooldown_seconds

        absence_satisfied = (
            self.stop_sign_absent_since is not None
            and now - self.stop_sign_absent_since
            >= self.stop_rearm_absence_seconds
        )
        if (
            self.stop_sign_phase == StopSignPhase.COOLDOWN
            and self.cooldown_until is not None
            and now >= self.cooldown_until
            and absence_satisfied
        ):
            self.stop_sign_phase = StopSignPhase.IDLE
            self.cooldown_until = None

    @property
    def stop_active(self) -> bool:
        return self.red_latched or self.stop_sign_phase in (
            StopSignPhase.BRAKING,
            StopSignPhase.HOLD,
        )

    @property
    def state_name(self) -> str:
        if self.red_latched:
            return "RED_LIGHT_STOP"
        return self.stop_sign_phase.value


def stopping_distance(
    speed: float,
    reaction_seconds: float,
    deceleration: float,
    margin: float,
) -> float:
    if deceleration <= 0.0:
        raise ValueError("deceleration must be greater than zero")
    forward_speed = max(0.0, speed)
    reaction_distance = forward_speed * reaction_seconds
    braking_distance = forward_speed * forward_speed / (2.0 * deceleration)
    return reaction_distance + braking_distance + margin


def in_trigger_zone(
    position_valid: bool,
    x: float,
    z: float,
    max_lateral_offset: float,
    trigger_distance: float,
) -> bool:
    return (
        position_valid
        and 0.0 < z <= trigger_distance
        and abs(x) <= max_lateral_offset
    )
