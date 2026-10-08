import unittest

import numpy as np

from power_control.core.controllers import ModeStateMachine, PIController, ratio_to_duties
from power_control.core.simulation import (
    ControlConfig,
    PlantConfig,
    PowerScenarioConfig,
    ScenarioConfig,
    metrics,
    power_metrics,
    run_power_simulation,
    run_simulation,
)


class SimulationTests(unittest.TestCase):
    def test_pi_uses_per_sample_integral_gain_without_dt(self):
        controller = PIController(kp=1.0, ki=0.1, out_min=-10.0, out_max=10.0)
        self.assertAlmostEqual(controller.update(2.0), 2.2)
        self.assertAlmostEqual(controller.update(2.0), 2.4)

    def test_mode_guard_blocks_wrong_boost_command_in_buck(self):
        selector = ModeStateMachine()
        mode, _ = selector.update(0.5, 0.0)
        guarded = selector.constrain_ratio(3.0, 0.92)
        buck, boost = ratio_to_duties(guarded, 0.0, 0.92)
        self.assertEqual(mode, "buck")
        self.assertLessEqual(buck, 0.92)
        self.assertEqual(boost, 0.0)

    def test_mode_hysteresis_prevents_chatter(self):
        selector = ModeStateMachine(minimum_dwell=0.0)
        self.assertEqual(selector.update(0.5, 0.0)[0], "buck")
        self.assertEqual(selector.update(0.91, 0.1)[0], "transition")
        self.assertEqual(selector.update(0.88, 0.2)[0], "transition")
        self.assertEqual(selector.update(0.85, 0.3)[0], "buck")

    def test_modulator_matches_requested_ratio(self):
        for ratio in (0.5, 0.9, 1.0, 1.1, 1.8, 3.0):
            db, ds = ratio_to_duties(ratio, 0.0, 0.98)
            actual = db / (1.0 - ds)
            self.assertAlmostEqual(actual, ratio, delta=0.025)

    def test_cascade_reaches_target(self):
        p = PlantConfig()
        s = ScenarioConfig(duration=0.35, load_step_time=1.0, input_step_time=1.0)
        data = run_simulation(p, ControlConfig(), s, "cascade")
        result = metrics(data, p.vref)
        self.assertTrue(np.all(np.isfinite(data["vout"])))
        self.assertLess(abs(result["steady_error"]), 1.0)
        # Buck leg is allowed to be statically on in boost mode.
        self.assertLessEqual(np.max(data["buck_duty"]), 1.0)
        self.assertLessEqual(np.max(data["boost_duty"]), p.duty_max)
        self.assertLessEqual(np.nanmax(data["iref"]), p.current_limit)

    def test_power_mode_holds_referee_input_limit(self):
        p = PlantConfig(vin=24.0, current_limit=20.0)
        s = PowerScenarioConfig(duration=1.5, vehicle_step_time=0.5)
        data = run_power_simulation(p, ControlConfig(), s)
        result = power_metrics(data)
        self.assertLess(abs(result["power_error"]), 1.0)
        self.assertGreater(data["converter_power"][1000], 0.0)  # charging
        self.assertLess(data["converter_power"][-1], 0.0)  # discharging
        self.assertLessEqual(np.max(np.abs(data["iref"])), p.current_limit)


if __name__ == "__main__":
    unittest.main()

