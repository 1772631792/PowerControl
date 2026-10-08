"""Steady-state voltage and duty-cycle relationship plots."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from power_control.core.controllers import ratio_to_duties


class SteadyCurvePanel(ttk.Frame):
    """Show ideal steady voltage curves and the continuous FSBB modulator map."""

    def __init__(self, master, fields: dict[str, tk.StringVar], **kwargs) -> None:
        super().__init__(master, **kwargs)
        self.fields = fields
        self.application_mode = "voltage"
        self._state: tuple[float, float, float, float] | None = None
        self._draw_job = None
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)

        self.note = ttk.Label(
            self,
            text="曲线采用理想 CCM 伏秒平衡。功率模式下它表示当前电压对应的前馈工作点，不表示 Vcap 是闭环稳压目标。",
            foreground="#5d6d7e",
            wraplength=900,
            justify="left",
        )
        self.note.grid(row=0, column=0, sticky="ew", padx=8, pady=(6, 2))

        self.figure = Figure(figsize=(9, 5), dpi=100, constrained_layout=True, facecolor="#fbfbfd")
        self.axes = self.figure.subplots(1, 2)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        toolbar = NavigationToolbar2Tk(self.canvas, self, pack_toolbar=False)
        toolbar.update()
        toolbar.grid(row=2, column=0, sticky="ew")

        for key in ("vin", "vref", "duty_max", "cap_voltage_initial"):
            fields[key].trace_add("write", self._field_changed)
        self.after_idle(self.draw)

    def _number(self, key: str, fallback: float = 0.0) -> float:
        try:
            return float(self.fields[key].get())
        except ValueError:
            return fallback

    def _field_changed(self, *_args) -> None:
        self._state = None
        self._schedule_draw()

    def _schedule_draw(self) -> None:
        if self._draw_job is not None:
            self.after_cancel(self._draw_job)
        self._draw_job = self.after_idle(self.draw)

    def set_application_mode(self, mode: str) -> None:
        self.application_mode = mode
        self._state = None
        self._schedule_draw()

    def set_operating_state(self, vin: float, voltage: float, buck_duty: float, boost_duty: float) -> None:
        self._state = (vin, voltage, buck_duty, boost_duty)
        self._schedule_draw()

    def draw(self) -> None:
        self._draw_job = None
        if self._state is None:
            vin = max(self._number("vin", 24.0), 0.1)
            target_key = "cap_voltage_initial" if self.application_mode == "power" else "vref"
            voltage = max(self._number(target_key, vin), 0.0)
            ratio = voltage / vin
            duty_max = min(max(self._number("duty_max", 0.92), 0.05), 0.99)
            buck_duty, boost_duty = ratio_to_duties(ratio, 0.0, duty_max)
        else:
            vin, voltage, buck_duty, boost_duty = self._state
            duty_max = min(max(self._number("duty_max", 0.92), 0.05), 0.99)

        ax_relation, ax_map = self.axes
        ax_relation.clear()
        ax_map.clear()
        for axis in self.axes:
            axis.set_facecolor("#ffffff")
            axis.grid(True, color="#d2d2d7", alpha=0.48, linewidth=0.8)
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)
            axis.spines["left"].set_color("#d2d2d7")
            axis.spines["bottom"].set_color("#d2d2d7")
            axis.tick_params(colors="#515154", labelsize=9)

        duty = np.linspace(0.0, duty_max, 500)
        buck_voltage = vin * duty
        boost_voltage = vin / np.maximum(1.0 - duty, 1e-4)
        y_max = max(vin * 2.2, voltage * 1.35, 30.0)

        ax_relation.plot(duty * 100.0, buck_voltage, label="Buck：V=D·Vin", color="#0071e3", linewidth=2)
        ax_relation.plot(duty * 100.0, boost_voltage, label="Boost：V=Vin/(1-D)", color="#ff9f0a", linewidth=2)
        ax_relation.axhline(voltage, color="#ff3b30", linestyle="--", linewidth=1.2, label=f"当前电压 {voltage:.2f} V")
        if voltage <= vin and 0.0 <= voltage / vin <= duty_max:
            ax_relation.scatter([voltage / vin * 100.0], [voltage], color="#ff3b30", s=48, zorder=5)
        if voltage >= vin:
            required_boost = 1.0 - vin / max(voltage, 0.1)
            if required_boost <= duty_max:
                ax_relation.scatter([required_boost * 100.0], [voltage], color="#ff3b30", s=48, zorder=5)
        ax_relation.set_xlim(0.0, duty_max * 100.0)
        ax_relation.set_ylim(0.0, y_max)
        ax_relation.set_xlabel("PWM 占空比 / %")
        ax_relation.set_ylabel("理想稳态电压 / V")
        ax_relation.set_title(f"单模式稳态关系（Vin={vin:.2f} V）")
        ax_relation.legend(loc="best", fontsize=8)

        max_voltage = max(y_max, vin * 1.2)
        voltage_axis = np.linspace(max(0.05 * vin, 0.1), max_voltage, 600)
        buck_values = np.empty_like(voltage_axis)
        boost_values = np.empty_like(voltage_axis)
        for index, value in enumerate(voltage_axis):
            buck_values[index], boost_values[index] = ratio_to_duties(
                value / vin, 0.0, duty_max
            )
        ax_map.plot(voltage_axis, buck_values * 100.0, label="左桥 Buck duty (Q1)", color="#0071e3", linewidth=2)
        ax_map.plot(voltage_axis, boost_values * 100.0, label="右桥 Boost duty (Q3)", color="#ff9f0a", linewidth=2)
        ax_map.axvline(voltage, color="#ff3b30", linestyle="--", linewidth=1.2)
        ax_map.scatter([voltage, voltage], [buck_duty * 100.0, boost_duty * 100.0], color="#ff3b30", s=45, zorder=5)
        ax_map.axvspan(0.92 * vin, 1.08 * vin, color="#ffd98a", alpha=0.25, label="双桥过渡区")
        ax_map.set_xlim(voltage_axis[0], max_voltage)
        ax_map.set_ylim(-2.0, 102.0)
        ax_map.set_xlabel("稳定电压 / V")
        ax_map.set_ylabel("所需占空比 / %")
        ax_map.set_title(
            f"连续调制工作点：Buck={buck_duty*100:.1f}%  Boost={boost_duty*100:.1f}%"
        )
        ax_map.legend(loc="best", fontsize=8)
        self.canvas.draw_idle()

