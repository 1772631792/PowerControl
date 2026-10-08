"""Averaged CCM model and digital control for a non-inverting buck-boost."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from .controllers import ModeStateMachine, PIController, clamp, ratio_to_duties


ControlMode = Literal["cascade", "single"]


@dataclass
class PlantConfig:
    vin: float = 24.0
    vref: float = 36.0
    inductance: float = 150e-6
    capacitance: float = 470e-6
    resistance: float = 12.0
    inductor_resistance: float = 0.035
    capacitor_esr: float = 0.025
    current_limit: float = 20.0
    duty_min: float = 0.0
    duty_max: float = 0.92


@dataclass
class ControlConfig:
    outer_kp: float = 0.30
    outer_ki: float = 0.025
    inner_kp: float = 0.0025
    inner_ki: float = 0.0005
    single_kp: float = 0.003
    single_ki: float = 0.0007
    single_kd: float = 0.0
    control_frequency: float = 1_000.0
    soft_start: float = 0.10


@dataclass
class ScenarioConfig:
    duration: float = 0.65
    dt: float = 5e-6
    load_step_time: float = 0.28
    load_after: float = 6.0
    input_step_time: float = 0.48
    input_after: float = 20.0


@dataclass
class PowerScenarioConfig:
    """RoboMaster-style bus load and supercapacitor scenario."""

    duration: float = 2.5
    dt: float = 10e-6
    power_limit: float = 80.0
    vehicle_power_initial: float = 40.0
    vehicle_step_time: float = 0.8
    vehicle_power_after: float = 160.0
    cap_voltage_initial: float = 18.0
    cap_voltage_min: float = 8.0
    cap_voltage_max: float = 23.5
    cap_capacitance: float = 5.0
    vehicle_time_constant: float = 0.060
    power_filter_time_constant: float = 0.006
    vehicle_power_ripple: float = 6.0
    vehicle_ripple_frequency: float = 2.0
    current_slew_rate: float = 250.0


def _validate(p: PlantConfig, c: ControlConfig, s: ScenarioConfig) -> None:
    positive = {
        "输入电压": p.vin,
        "目标电压": p.vref,
        "电感": p.inductance,
        "电容": p.capacitance,
        "负载": p.resistance,
        "限流": p.current_limit,
        "控制频率": c.control_frequency,
        "仿真时间": s.duration,
        "仿真步长": s.dt,
        "阶跃后负载": s.load_after,
        "阶跃后输入": s.input_after,
    }
    bad = [name for name, value in positive.items() if value <= 0]
    if bad:
        raise ValueError(f"这些参数必须大于 0：{', '.join(bad)}")
    if s.dt > 1.0 / c.control_frequency:
        raise ValueError("仿真步长应不大于控制周期")
    if not 0.0 < p.duty_max < 1.0:
        raise ValueError("最大占空比必须在 0 和 1 之间")


def run_simulation(
    plant: PlantConfig,
    control: ControlConfig,
    scenario: ScenarioConfig,
    mode: ControlMode = "cascade",
) -> dict[str, np.ndarray]:
    """Run the averaged converter model and return uniformly sampled traces."""

    _validate(plant, control, scenario)
    steps = int(round(scenario.duration / scenario.dt)) + 1
    control_period = 1.0 / control.control_frequency
    control_steps = max(1, int(round(control_period / scenario.dt)))

    # Store no more than about 8000 points so the GUI remains responsive.
    stride = max(1, steps // 8000)
    count = (steps - 1) // stride + 1
    traces = {name: np.empty(count) for name in (
        "time", "vout", "vref", "il", "iref", "vin", "load",
        "buck_duty", "boost_duty", "ratio", "power",
    )}

    outer = PIController(control.outer_kp, control.outer_ki, 0.0, plant.current_limit)
    inner = PIController(control.inner_kp, control.inner_ki, -2.5, 2.5)
    mode_selector = ModeStateMachine()
    single_i = 0.0
    previous_error = 0.0

    il = 0.0
    vc = 0.0
    buck_duty = 0.0
    boost_duty = 0.0
    ratio_cmd = 0.02
    current_ref = 0.0
    sample_index = 0

    for k in range(steps):
        t = k * scenario.dt
        vin = plant.vin if t < scenario.input_step_time else scenario.input_after
        load = plant.resistance if t < scenario.load_step_time else scenario.load_after
        ramp = 1.0 if control.soft_start <= 0 else min(1.0, t / control.soft_start)
        voltage_ref = plant.vref * ramp

        # Terminal voltage includes a small ESR approximation.
        load_current = max(vc, 0.0) / load
        output_factor = 1.0 - boost_duty
        capacitor_current = output_factor * max(il, 0.0) - load_current
        vout = max(0.0, vc + plant.capacitor_esr * capacitor_current)

        if k % control_steps == 0:
            ctrl_dt = control_steps * scenario.dt
            voltage_error = voltage_ref - vout
            ratio_ff = clamp(voltage_ref / max(vin, 0.1), 0.02, 5.0)
            measured_ratio = vout / max(vin, 0.1)
            _power_mode, mode_changed = mode_selector.update(measured_ratio, t)

            if mode == "cascade":
                current_ref = outer.update(voltage_error)
                current_error = current_ref - il
                if mode_changed:
                    previous_correction = ratio_cmd - ratio_ff
                    inner.reset(previous_correction - inner.kp * current_error)
                correction = inner.update(current_error)
                ratio_cmd = mode_selector.constrain_ratio(
                    ratio_ff + correction, plant.duty_max
                )
            else:
                derivative = voltage_error - previous_error
                if mode_changed:
                    single_i = ratio_cmd - ratio_ff - control.single_kp * voltage_error
                unsaturated = (
                    ratio_ff
                    + control.single_kp * voltage_error
                    + single_i
                    + control.single_kd * derivative
                )
                low, high = mode_selector.ratio_limits(plant.duty_max)
                ratio_cmd = clamp(unsaturated, low, high)
                if (
                    unsaturated == ratio_cmd
                    or (unsaturated > high and voltage_error < 0.0)
                    or (unsaturated < low and voltage_error > 0.0)
                ):
                    single_i += control.single_ki * voltage_error
                current_ref = np.nan
                previous_error = voltage_error

            buck_duty, boost_duty = ratio_to_duties(
                ratio_cmd, plant.duty_min, plant.duty_max
            )

        if k % stride == 0:
            traces["time"][sample_index] = t
            traces["vout"][sample_index] = vout
            traces["vref"][sample_index] = voltage_ref
            traces["il"][sample_index] = il
            traces["iref"][sample_index] = current_ref
            traces["vin"][sample_index] = vin
            traces["load"][sample_index] = load
            traces["buck_duty"][sample_index] = buck_duty
            traces["boost_duty"][sample_index] = boost_duty
            traces["ratio"][sample_index] = ratio_cmd
            traces["power"][sample_index] = vout * vout / load
            sample_index += 1

        if k == steps - 1:
            break

        # CCM averaged four-switch power stage:
        # L di/dt = D_buck*Vin - (1-D_boost)*Vout - i*R_L
        # C dv/dt = (1-D_boost)*i - Vout/R_load
        di = (
            buck_duty * vin
            - (1.0 - boost_duty) * vc
            - plant.inductor_resistance * il
        ) / plant.inductance
        dv = ((1.0 - boost_duty) * il - vc / load) / plant.capacitance
        il = max(0.0, il + di * scenario.dt)  # diode/CCM boundary approximation
        vc = max(0.0, vc + dv * scenario.dt)

    return traces


def metrics(data: dict[str, np.ndarray], final_ref: float) -> dict[str, float]:
    """Calculate a few practical performance indicators."""

    t = data["time"]
    v = data["vout"]
    settled_region = t >= 0.8 * t[-1]
    final_mean = float(np.mean(v[settled_region]))
    overshoot = max(0.0, (float(np.max(v)) - final_ref) / final_ref * 100.0)
    ripple = float(np.max(v[settled_region]) - np.min(v[settled_region]))
    current_peak = float(np.max(data["il"]))
    error = final_ref - final_mean
    return {
        "final_voltage": final_mean,
        "steady_error": error,
        "overshoot_percent": overshoot,
        "ripple_pp": ripple,
        "peak_current": current_peak,
    }


def run_power_simulation(
    plant: PlantConfig,
    control: ControlConfig,
    scenario: PowerScenarioConfig,
) -> dict[str, np.ndarray]:
    """Simulate a bidirectional RoboMaster-style supercapacitor power buffer.

    Positive inductor current charges the supercapacitor. Negative current
    discharges it into the chassis bus. The outer power calculation generates
    the current reference; a fast current PI controls the FSBB voltage ratio.
    """

    values = {
        "仿真时间": scenario.duration,
        "仿真步长": scenario.dt,
        "功率上限": scenario.power_limit,
        "初始车辆功率": scenario.vehicle_power_initial,
        "阶跃后车辆功率": scenario.vehicle_power_after,
        "超级电容初始电压": scenario.cap_voltage_initial,
        "超级电容最小电压": scenario.cap_voltage_min,
        "超级电容最大电压": scenario.cap_voltage_max,
        "超级电容容量": scenario.cap_capacitance,
        "车辆响应时间常数": scenario.vehicle_time_constant,
        "功率滤波时间常数": scenario.power_filter_time_constant,
        "负载波动频率": scenario.vehicle_ripple_frequency,
        "电流参考变化率": scenario.current_slew_rate,
    }
    bad = [name for name, value in values.items() if value <= 0]
    if bad:
        raise ValueError(f"这些参数必须大于 0：{', '.join(bad)}")
    if scenario.cap_voltage_min >= scenario.cap_voltage_max:
        raise ValueError("超级电容最小电压必须小于最大电压")
    if not scenario.cap_voltage_min <= scenario.cap_voltage_initial <= scenario.cap_voltage_max:
        raise ValueError("超级电容初始电压必须位于最小值和最大值之间")
    if scenario.vehicle_power_ripple < 0:
        raise ValueError("车辆功率波动幅值不能小于 0")
    if scenario.dt > 1.0 / control.control_frequency:
        raise ValueError("仿真步长应不大于控制周期")

    steps = int(round(scenario.duration / scenario.dt)) + 1
    control_steps = max(1, int(round((1.0 / control.control_frequency) / scenario.dt)))
    stride = max(1, steps // 8000)
    count = (steps - 1) // stride + 1
    names = (
        "time", "vin", "vcap", "il", "iref", "vehicle_power",
        "vehicle_power_command", "measured_vehicle_power",
        "input_power", "power_limit", "converter_power", "power_command",
        "buck_duty", "boost_duty", "ratio",
    )
    traces = {name: np.empty(count) for name in names}

    current_pi = PIController(control.inner_kp, control.inner_ki, -2.5, 2.5)
    mode_selector = ModeStateMachine()
    il = 0.0
    vcap = scenario.cap_voltage_initial
    current_ref = 0.0
    buck_duty = 0.0
    boost_duty = 0.0
    ratio_cmd = max(0.02, vcap / plant.vin)
    power_command = 0.0
    vehicle_power = scenario.vehicle_power_initial
    measured_vehicle_power = vehicle_power
    sample_index = 0

    for k in range(steps):
        t = k * scenario.dt
        vin = plant.vin
        vehicle_power_base = (
            scenario.vehicle_power_initial
            if t < scenario.vehicle_step_time
            else scenario.vehicle_power_after
        )
        vehicle_power_command = vehicle_power_base + scenario.vehicle_power_ripple * np.sin(
            2.0 * np.pi * scenario.vehicle_ripple_frequency * t
        )
        load_alpha = 1.0 - np.exp(-scenario.dt / scenario.vehicle_time_constant)
        vehicle_power += (vehicle_power_command - vehicle_power) * load_alpha

        if k % control_steps == 0:
            ctrl_dt = control_steps * scenario.dt
            filter_alpha = 1.0 - np.exp(
                -ctrl_dt / scenario.power_filter_time_constant
            )
            measured_vehicle_power += (
                vehicle_power - measured_vehicle_power
            ) * filter_alpha
            ratio_ff = clamp(vcap / max(vin, 0.1), 0.02, 5.0)
            _power_mode, mode_changed = mode_selector.update(ratio_ff, t)
            ff_buck, ff_boost = ratio_to_duties(
                ratio_ff, plant.duty_min, plant.duty_max
            )
            # Positive power charges the capacitor; negative power supports the bus.
            power_command = scenario.power_limit - measured_vehicle_power
            if vcap >= scenario.cap_voltage_max and power_command > 0.0:
                power_command = 0.0
            if vcap <= scenario.cap_voltage_min and power_command < 0.0:
                power_command = 0.0
            max_power = plant.current_limit * vin * max(ff_buck, 0.05)
            power_command = clamp(power_command, -max_power, max_power)
            # Bus-side average current is D_left * iL. Recalculate from the
            # latest applied duty so the regulated quantity is power, not just
            # inductor current at the feed-forward operating point.
            duty_for_power = buck_duty if k > 0 else ff_buck
            raw_current_ref = clamp(
                power_command / (vin * max(duty_for_power, 0.05)),
                -plant.current_limit,
                plant.current_limit,
            )
            max_current_change = scenario.current_slew_rate * ctrl_dt
            current_ref = clamp(
                raw_current_ref,
                current_ref - max_current_change,
                current_ref + max_current_change,
            )
            current_error = current_ref - il
            if mode_changed:
                previous_correction = ratio_cmd - ratio_ff
                current_pi.reset(previous_correction - current_pi.kp * current_error)
            correction = current_pi.update(current_error)
            ratio_cmd = mode_selector.constrain_ratio(
                ratio_ff + correction, plant.duty_max
            )
            buck_duty, boost_duty = ratio_to_duties(
                ratio_cmd, plant.duty_min, plant.duty_max
            )

        right_upper_duty = 1.0 - boost_duty
        converter_power = vin * buck_duty * il
        input_power = vehicle_power + converter_power

        if k % stride == 0:
            values_now = (
                t, vin, vcap, il, current_ref, vehicle_power,
                vehicle_power_command, measured_vehicle_power, input_power,
                scenario.power_limit, converter_power, power_command,
                buck_duty, boost_duty, ratio_cmd,
            )
            for name, value in zip(names, values_now):
                traces[name][sample_index] = value
            sample_index += 1

        if k == steps - 1:
            break

        di = (
            buck_duty * vin
            - right_upper_duty * vcap
            - plant.inductor_resistance * il
        ) / plant.inductance
        dv_cap = right_upper_duty * il / scenario.cap_capacitance
        il += di * scenario.dt
        vcap = clamp(
            vcap + dv_cap * scenario.dt,
            scenario.cap_voltage_min,
            scenario.cap_voltage_max,
        )

    return traces


def power_metrics(data: dict[str, np.ndarray]) -> dict[str, float]:
    """Performance indicators for the supercapacitor power-control mode."""

    t = data["time"]
    region = t >= 0.8 * t[-1]
    power_error = data["input_power"][region] - data["power_limit"][region]
    return {
        "input_power": float(np.mean(data["input_power"][region])),
        "power_error": float(np.mean(power_error)),
        "power_error_peak": float(np.max(np.abs(power_error))),
        "cap_voltage": float(data["vcap"][-1]),
        "peak_current": float(np.max(np.abs(data["il"]))),
    }

