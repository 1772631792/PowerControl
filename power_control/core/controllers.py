"""Control blocks for the averaged four-switch buck-boost simulation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class PIController:
    """Discrete PI controller with per-sample Ki and anti-windup.

    Ki is already the integral gain for one control update. This mirrors common
    embedded code: integral += Ki * error, with no runtime dt multiplication.
    """

    kp: float
    ki: float
    out_min: float
    out_max: float
    integral: float = 0.0

    def reset(self, output: float = 0.0) -> None:
        self.integral = clamp(output, self.out_min, self.out_max)

    def update(self, error: float, feedforward: float = 0.0) -> float:
        proportional = self.kp * error
        candidate = proportional + self.integral + feedforward
        saturated = clamp(candidate, self.out_min, self.out_max)

        # Only integrate when it moves a saturated output back toward its range.
        integrate = (
            candidate == saturated
            or (candidate > self.out_max and error < 0.0)
            or (candidate < self.out_min and error > 0.0)
        )
        if integrate:
            self.integral += self.ki * error
            self.integral = clamp(
                self.integral,
                self.out_min - feedforward - proportional,
                self.out_max - feedforward - proportional,
            )
        return clamp(proportional + self.integral + feedforward, self.out_min, self.out_max)


def ratio_to_duties(ratio: float, duty_min: float, duty_max: float, blend: float = 0.08) -> tuple[float, float]:
    """Map conversion ratio to buck and boost PWM duties.

    Averaged four-switch relation: M = D_buck / (1 - D_boost).
    Around M=1 both legs participate, avoiding an abrupt mode jump.
    """

    ratio = max(0.001, ratio)
    lo = min(1.0 - blend, duty_max)
    hi = 1.0 + blend
    if ratio <= lo:
        buck = ratio
        boost = 0.0
    elif ratio >= hi:
        # In boost mode the buck leg is statically on; 100% here is not a PWM
        # command and therefore is not constrained by duty_max.
        buck = 1.0
        boost = 1.0 - 1.0 / ratio
    else:
        # Raise the buck leg smoothly to static-on and solve boost duty so that
        # D_buck/(1-D_boost) remains the requested ratio throughout the blend.
        alpha = (ratio - lo) / (hi - lo)
        buck = lo + alpha * (1.0 - lo)
        boost = 1.0 - buck / ratio
    buck_limit = duty_max if ratio <= lo else 1.0
    return clamp(buck, duty_min, buck_limit), clamp(boost, 0.0, duty_max)


PowerMode = Literal["buck", "transition", "boost"]


@dataclass
class ModeStateMachine:
    """Select FSBB mode from measured voltage ratio, never from PID output.

    Separate enter/exit thresholds provide hysteresis. A minimum dwell time
    prevents noisy voltage samples from changing topology every control cycle.
    """

    minimum_dwell: float = 0.002
    buck_enter: float = 0.86
    buck_exit: float = 0.90
    boost_exit: float = 1.06
    boost_enter: float = 1.10
    mode: PowerMode | None = None
    last_change: float = -1e9

    def update(self, measured_ratio: float, now: float) -> tuple[PowerMode, bool]:
        if self.mode is None:
            if measured_ratio <= self.buck_exit:
                self.mode = "buck"
            elif measured_ratio >= self.boost_enter:
                self.mode = "boost"
            else:
                self.mode = "transition"
            self.last_change = now
            return self.mode, True

        if now - self.last_change < self.minimum_dwell:
            return self.mode, False

        new_mode = self.mode
        if self.mode == "buck" and measured_ratio >= self.buck_exit:
            new_mode = "transition"
        elif self.mode == "boost" and measured_ratio <= self.boost_exit:
            new_mode = "transition"
        elif self.mode == "transition":
            if measured_ratio <= self.buck_enter:
                new_mode = "buck"
            elif measured_ratio >= self.boost_enter:
                new_mode = "boost"

        changed = new_mode != self.mode
        if changed:
            self.mode = new_mode
            self.last_change = now
        return self.mode, changed

    def constrain_ratio(self, command: float, duty_max: float) -> float:
        """Keep a faulty or saturated PID command inside the selected mode."""

        low, high = self.ratio_limits(duty_max)
        return clamp(command, low, high)

    def ratio_limits(self, duty_max: float) -> tuple[float, float]:
        if self.mode == "buck":
            # Leave enough headroom above the exit threshold so the measured
            # voltage can actually cross it; otherwise startup can deadlock in
            # Buck mode just below the boundary.
            return 0.02, duty_max
        if self.mode == "boost":
            return max(1.08, self.boost_exit), 5.0
        # Extra command headroom lets the measured ratio cross the enter
        # threshold despite conduction loss, avoiding a transition deadlock.
        return max(0.02, self.buck_enter), self.boost_enter + 0.04

