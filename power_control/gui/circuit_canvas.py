"""Interactive schematic used by the Buck-Boost GUI."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, simpledialog

from power_control.core.controllers import ratio_to_duties


class CircuitDiagram(tk.Canvas):
    """Draw a four-switch non-inverting Buck-Boost schematic.

    Components tagged with a field key can be double-clicked to edit the same
    StringVar that is used by the parameter notebook.
    """

    EDITABLE = {
        "vin": ("输入电压 Vin", "V"),
        "vref": ("目标电压 Vref", "V"),
        "l_uh": ("电感 L", "µH"),
        "c_uf": ("输出电容 C", "µF"),
        "resistance": ("负载电阻 R", "Ω"),
        "duty_max": ("PWM 最大占空比", ""),
        "current_limit": ("电感限流", "A"),
        "vehicle_power_initial": ("车辆初始功率", "W"),
        "power_limit": ("裁判系统功率上限", "W"),
        "cap_voltage_initial": ("超级电容初始电压", "V"),
        "cap_capacitance": ("超级电容容量", "F"),
    }

    def __init__(self, master, fields: dict[str, tk.StringVar], **kwargs) -> None:
        super().__init__(
            master,
            background="#ffffff",
            highlightthickness=1,
            highlightbackground="#d2d2d7",
            cursor="arrow",
            **kwargs,
        )
        self.fields = fields
        self.application_mode = "voltage"
        self._redraw_job = None
        self._state: tuple[float, float, float, float] | None = None
        self.bind("<Configure>", self._schedule_redraw)
        for variable in fields.values():
            variable.trace_add("write", self._value_changed)

    def _value_changed(self, *_args) -> None:
        self._state = None
        self._schedule_redraw()

    def _schedule_redraw(self, _event=None) -> None:
        if self._redraw_job is not None:
            self.after_cancel(self._redraw_job)
        self._redraw_job = self.after_idle(self.draw)

    def draw(self) -> None:
        self._redraw_job = None
        self.delete("all")
        width = max(self.winfo_width(), 700)
        height = max(self.winfo_height(), 260)
        sx = width / 1000.0
        sy = height / 300.0
        x = lambda value: value * sx
        y = lambda value: value * sy
        wire = {"fill": "#34495e", "width": 2}
        active = "#087e8b"
        on_color = "#2a9d68"
        pwm_color = "#ed8b23"
        off_color = "#9aa7b4"

        vin = self._number("vin")
        target = self._number("cap_voltage_initial") if self.application_mode == "power" else self._number("vref")
        if self._state is None:
            ratio_request = target / max(vin, 0.1)
            buck_duty, boost_duty = ratio_to_duties(
                ratio_request, 0.0, min(max(self._number("duty_max"), 0.1), 0.99)
            )
            shown_vin, shown_vout = vin, target
        else:
            buck_duty, boost_duty, shown_vin, shown_vout = self._state
        ratio = shown_vout / max(shown_vin, 0.1)
        if boost_duty <= 1e-6 and buck_duty < 0.999:
            mode = "BUCK 降压"
            q1, q2, q4, q3 = pwm_color, pwm_color, on_color, off_color
            state_text = f"Q1/Q2 互补 PWM {buck_duty * 100:.1f}%    Q4 常导通    Q3 关断"
            formula = f"Vout ≈ D·Vin = {buck_duty:.3f} × {shown_vin:.1f} V = {shown_vout:.1f} V"
        elif buck_duty >= 0.999 and boost_duty > 1e-6:
            mode = "BOOST 升压"
            q1, q2, q4, q3 = on_color, off_color, pwm_color, pwm_color
            state_text = f"Q1 常导通    Q2 关断    Q3/Q4 互补 PWM {boost_duty * 100:.1f}%"
            formula = f"Vout ≈ Vin/(1-D) = {shown_vin:.1f} V/(1-{boost_duty:.3f}) = {shown_vout:.1f} V"
        else:
            mode = "BUCK-BOOST 过渡区"
            q1 = q2 = q4 = q3 = pwm_color
            state_text = f"左右半桥同时 PWM：Dleft={buck_duty * 100:.1f}%  Dright={(1-boost_duty) * 100:.1f}%"
            formula = "Vright/Vleft = Dleft/Dright"

        self.create_text(x(22), y(13), anchor="nw", text="四开关双向 Buck-Boost：电感连接两个半桥中点", fill="#415267", font=("Microsoft YaHei UI", 11, "bold"))
        self.create_text(x(978), y(15), anchor="ne", text=f"当前：{mode}", fill="#b85c00", font=("Microsoft YaHei UI", 11, "bold"))
        subtitle = (
            "RoboMaster：电池/裁判系统与车辆共母线，变换器控制超级电容双向充放电功率"
            if self.application_mode == "power"
            else "稳压模型：左端直流源 → FSBB → 右端输出电容与阻性负载"
        )
        self.create_text(x(500), y(40), text=subtitle, fill="#77552a", font=("Microsoft YaHei UI", 9))

        # Shared ground, left DC source, and separate DC rails.
        self.create_line(x(55), y(245), x(945), y(245), **wire)
        self.create_line(x(90), y(55), x(250), y(55), **wire)
        self.create_line(x(620), y(55), x(945), y(55), **wire)
        self.create_line(x(90), y(55), x(90), y(96), **wire)
        self.create_oval(x(66), y(96), x(114), y(158), outline=active, width=3, tags=("editable", "vin"))
        self.create_text(x(90), y(112), text="+", fill=active, font=("Arial", 12, "bold"), tags=("editable", "vin"))
        self.create_text(x(90), y(143), text="−", fill=active, font=("Arial", 12, "bold"), tags=("editable", "vin"))
        self.create_line(x(90), y(158), x(90), y(245), **wire)
        self._component_label(x(90), y(184), "vin", "左端 Vin")

        if self.application_mode == "power":
            # The chassis is in parallel with the source on the left bus.
            self.create_line(x(155), y(55), x(155), y(77), **wire)
            self.create_rectangle(x(122), y(77), x(188), y(137), outline="#c65d37", width=2, fill="#fff3eb", tags=("editable", "vehicle_power_initial"))
            self.create_text(x(155), y(96), text="车辆", fill="#a74727", font=("Microsoft YaHei UI", 9, "bold"), tags=("editable", "vehicle_power_initial"))
            self.create_text(x(155), y(118), text=f"{self.fields['vehicle_power_initial'].get()} W", fill="#a74727", font=("Consolas", 9), tags=("editable", "vehicle_power_initial"))
            self.create_line(x(155), y(137), x(155), y(245), **wire)
            self.create_text(x(155), y(159), text=f"上限 {self.fields['power_limit'].get()} W", fill="#76511d", font=("Microsoft YaHei UI", 8), tags=("editable", "power_limit"))

        # Two vertical half bridges. Their switching nodes are connected by L.
        self.create_line(x(250), y(55), x(250), y(70), **wire)
        self._mosfet(x(250), y(86), "Q1", q1, "duty_max")
        self.create_line(x(250), y(102), x(250), y(130), **wire)
        self.create_oval(x(244), y(124), x(256), y(136), fill="#34495e", outline="")
        self.create_line(x(250), y(130), x(250), y(158), **wire)
        self._mosfet(x(250), y(174), "Q2", q2, "duty_max")
        self.create_line(x(250), y(190), x(250), y(245), **wire)

        self.create_line(x(620), y(55), x(620), y(70), **wire)
        self._mosfet(x(620), y(86), "Q4", q4, "duty_max")
        self.create_line(x(620), y(102), x(620), y(130), **wire)
        self.create_oval(x(614), y(124), x(626), y(136), fill="#34495e", outline="")
        self.create_line(x(620), y(130), x(620), y(158), **wire)
        self._mosfet(x(620), y(174), "Q3", q3, "duty_max")
        self.create_line(x(620), y(190), x(620), y(245), **wire)

        # Inductor is explicitly between SW_L and SW_R.
        self.create_line(x(250), y(130), x(318), y(130), **wire)
        points = [x(318), y(130)]
        for index in range(9):
            points.extend((x(334 + index * 30), y(130 + (-12 if index % 2 == 0 else 12))))
        points.extend((x(588), y(130), x(620), y(130)))
        self.create_line(*points, fill=active, width=3, smooth=True, tags=("editable", "l_uh"))
        self._component_label(x(435), y(101), "l_uh", "L")
        self.create_text(x(435), y(163), text=self._value("current_limit", "A") + " 限流", fill="#8a5a00", font=("Microsoft YaHei UI", 9), tags=("editable", "current_limit"))
        self.create_line(x(350), y(188), x(515), y(188), fill="#de6b35", width=2, arrow="last")
        self.create_text(x(432), y(204), text="正电流 / 充电方向", fill="#b75324", font=("Microsoft YaHei UI", 9))

        # Right-side capacitor. In power mode this is the supercapacitor bank.
        self.create_line(x(735), y(55), x(735), y(112), **wire)
        self.create_line(x(710), y(112), x(760), y(112), fill=active, width=3, tags=("editable", "c_uf"))
        self.create_line(x(710), y(126), x(760), y(126), fill=active, width=3, tags=("editable", "c_uf"))
        self.create_line(x(735), y(126), x(735), y(245), **wire)
        if self.application_mode == "power":
            self.create_text(x(735), y(153), text=f"Csuper = {self.fields['cap_capacitance'].get()} F", fill=active, font=("Microsoft YaHei UI", 9, "bold"), tags=("editable", "cap_capacitance"))
            self.create_text(x(735), y(178), text=f"Vcap = {target:g} V", fill=active, font=("Microsoft YaHei UI", 9, "bold"), tags=("editable", "cap_voltage_initial"))
            self.create_text(x(858), y(122), text="超级电容组", fill="#4b5968", font=("Microsoft YaHei UI", 10, "bold"))
            self.create_text(x(858), y(145), text="充电：iL > 0\n放电：iL < 0", fill="#687888", font=("Microsoft YaHei UI", 8), justify="center")
        else:
            self._component_label(x(735), y(153), "c_uf", "Cout")
            self.create_line(x(850), y(55), x(850), y(92), **wire)
            resistor = [x(850), y(92)]
            for index in range(8):
                resistor.extend((x(837 if index % 2 == 0 else 863), y(100 + index * 12)))
            resistor.extend((x(850), y(202)))
            self.create_line(*resistor, fill=active, width=3, tags=("editable", "resistance"))
            self.create_line(x(850), y(202), x(850), y(245), **wire)
            self._component_label(x(902), y(148), "resistance", "Rload")
            self.create_oval(x(928), y(46), x(946), y(64), fill="#ff5a5f", outline="", tags=("editable", "vref"))
            self._component_label(x(925), y(82), "vref", "右端 Vout*")
        self.create_text(x(490), y(263), text=state_text, fill="#4b5968", font=("Microsoft YaHei UI", 9, "bold"), tags=("editable", "duty_max"))
        self.create_text(x(490), y(284), text=formula, fill="#087e8b", font=("Consolas", 9, "bold"))

        for key in self.EDITABLE:
            self.tag_bind(key, "<Double-Button-1>", lambda _event, field=key: self.edit(field))
            self.tag_bind(key, "<Enter>", lambda _event: self.configure(cursor="hand2"))
            self.tag_bind(key, "<Leave>", lambda _event: self.configure(cursor="arrow"))

    def _mosfet(self, px: float, py: float, name: str, color: str, tag: str) -> None:
        self.create_rectangle(px - 27, py - 16, px + 27, py + 16, outline=color, width=3, fill="#f7f9fb", tags=("editable", tag))
        self.create_text(px, py, text=name, fill=color, font=("Arial", 9, "bold"), tags=("editable", tag))

    def _component_label(self, px: float, py: float, key: str, symbol: str) -> None:
        label, unit = self.EDITABLE[key]
        del label
        text = f"{symbol} = {self.fields[key].get()} {unit}".rstrip()
        self.create_text(px, py, text=text, fill="#087e8b", font=("Microsoft YaHei UI", 9, "bold"), tags=("editable", key))

    def _value(self, key: str, unit: str) -> str:
        return f"{self.fields[key].get()} {unit}".strip()

    def _number(self, key: str) -> float:
        try:
            return float(self.fields[key].get())
        except ValueError:
            return 0.0

    def set_operating_state(self, buck_duty: float, boost_duty: float, vin: float, vout: float) -> None:
        """Update mode and switch-state annotations from simulation results."""
        self._state = (buck_duty, boost_duty, vin, vout)
        self._schedule_redraw()

    def set_application_mode(self, mode: str) -> None:
        self.application_mode = mode
        self._state = None
        self._schedule_redraw()

    def edit(self, key: str) -> None:
        title, unit = self.EDITABLE[key]
        try:
            initial = float(self.fields[key].get())
        except ValueError:
            initial = 0.0
        value = simpledialog.askfloat(
            "修改电路参数",
            f"{title}{f' / {unit}' if unit else ''}",
            initialvalue=initial,
            parent=self.winfo_toplevel(),
        )
        if value is None:
            return
        if value <= 0:
            messagebox.showerror("参数错误", "参数必须大于 0。", parent=self.winfo_toplevel())
            return
        if key == "duty_max" and value >= 1:
            messagebox.showerror("参数错误", "PWM 最大占空比必须小于 1。", parent=self.winfo_toplevel())
            return
        self.fields[key].set(f"{value:g}")

