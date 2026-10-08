"""Formula and mode-transition reference panel for the GUI."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from power_control.core.controllers import ratio_to_duties


class FormulaPanel(ttk.Frame):
    """Scrollable formula reference with a live operating-point marker."""

    TRANSITION_HIGH = 1.08

    def __init__(self, master, fields: dict[str, tk.StringVar], **kwargs) -> None:
        super().__init__(master, **kwargs)
        self.fields = fields
        self.application_mode = "voltage"
        self._state: tuple[float, float, float, float] | None = None
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        ttk.Label(self, text="Buck / Boost 模式切换", font=("Microsoft YaHei UI", 11, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 5)
        )
        self.mode_canvas = tk.Canvas(self, height=105, background="#ffffff", highlightthickness=1, highlightbackground="#d2d2d7")
        self.mode_canvas.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        self.mode_canvas.bind("<Configure>", lambda _event: self._draw_mode_bar())

        text_frame = ttk.Frame(self)
        text_frame.grid(row=2, column=0, sticky="nsew")
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)
        self.text = tk.Text(
            text_frame,
            wrap="word",
            borderwidth=0,
            background="#ffffff",
            foreground="#283747",
            font=("Microsoft YaHei UI", 9),
            padx=8,
            pady=7,
            cursor="arrow",
        )
        scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=self.text.yview)
        self.text.configure(yscrollcommand=scrollbar.set)
        self.text.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.text.tag_configure("title", font=("Microsoft YaHei UI", 10, "bold"), foreground="#145a73", spacing1=8, spacing3=3)
        self.text.tag_configure("formula", font=("Consolas", 10, "bold"), foreground="#087e8b", lmargin1=8, lmargin2=8, spacing1=2, spacing3=5)
        self.text.tag_configure("body", spacing3=4)
        self._fill_reference()
        self.text.configure(state="disabled")

        for key in ("vin", "vref", "duty_max", "cap_voltage_initial"):
            fields[key].trace_add("write", self._field_changed)

    def _fill_reference(self) -> None:
        sections = [
            ("title", "1. 符号定义\n"),
            ("body", "Dₗ：左半桥上管 Q1 占空比\nDᵣ：右半桥上管 Q4 占空比\nGUI 中 Boost 占空比表示 Q3 下管占空比，即 1−Dᵣ。\n"),
            ("title", "2. 功率级平均模型\n"),
            ("formula", "L·diₗ/dt = Dₗ·Vin − Dᵣ·Vout − Rₗ·iₗ\n"),
            ("formula", "C·dVout/dt = Dᵣ·iₗ − Vout/Rload\n"),
            ("body", "稳态时电感平均电压为零，因此：\n"),
            ("formula", "M = Vout/Vin = Dₗ/Dᵣ\n"),
            ("title", "3. Buck 降压区\n"),
            ("formula", "Dᵣ = 1，Dₗ = M，Vout = Dₗ·Vin\n"),
            ("body", "Q1/Q2 互补 PWM；Q4 常导通；Q3 关断。\n"),
            ("title", "4. Boost 升压区\n"),
            ("formula", "Dₗ = 1，Dᵣ = 1/M\nQ3 duty = 1 − Dᵣ\n"),
            ("body", "Q1 常导通；Q2 关断；Q3/Q4 互补 PWM。\n"),
            ("title", "5. 升降压过渡区\n"),
            ("body", "默认在 M=0.92～1.08 内让两个半桥同时参与调制。Dₗ 从 0.92 连续增加到 1，并实时计算：\n"),
            ("formula", "Dᵣ = Dₗ/M\n"),
            ("body", "这样始终满足 Dₗ/Dᵣ=M，占空比不会在 Vin≈Vout 时突然跳变。此处是连续混合区，不是机械式切换。\n"),
            ("title", "6. 安全模式状态机\n"),
            ("formula", "Mmeasure = Vright/Vleft\n"),
            ("body", "状态机使用实测电压比，而不是 PID 输出判定模式。Buck→过渡：M≥0.90；过渡→Buck：M≤0.86；过渡→Boost：M≥1.10；Boost→过渡：M≤1.06。阈值滞回并带 2 ms 最小驻留时间。\n"),
            ("formula", "Mcmd = clamp(MPID, Mmin(mode), Mmax(mode))\n"),
            ("body", "Buck 状态下即使 PID 瞬间产生 Boost 指令，也会被限制在 Buck 合法范围；切换时同步积分器，实现无扰切换。\n"),
            ("title", "7. 稳压级联控制器（普通模式）\n"),
            ("formula", "I[k] = I[k−1] + Ki·e[k]\nu[k] = Kp·e[k] + I[k]\n"),
            ("formula", "eᵥ[k] = Vref − Vout\nIref[k] = sat(Kpv·eᵥ[k] + Iv[k])\n"),
            ("formula", "eᵢ[k] = Iref[k] − iₗ[k]\nMcmd[k] = Vref/Vin + Kpi·eᵢ[k] + Ii[k]\n"),
            ("body", "默认控制频率为 1000 Hz。Ki 是每次控制循环的离散增益，代码直接累加 Ki·e，不再乘 dt。外电压环决定电流参考，快速电流内环调整变换比。\n"),
            ("title", "8. 车辆功率控制（默认模式）\n"),
            ("formula", "Pconv* = Plimit − Pvehicle\n"),
            ("formula", "iL* = Pconv*/(Vbus·Dₗ)\n"),
            ("formula", "ei[k] = iL*[k] − iL[k]\nIi[k] = Ii[k−1] + Kii·ei[k]\nMcmd[k] = Vcap/Vbus + Kpi·ei[k] + Ii[k]\n"),
            ("body", "Pconv*>0 时给超级电容充电；Pconv*<0 时超级电容放电支援车辆。最终目标是让裁判系统看到的电源功率接近 Plimit。\n"),
            ("title", "9. 超级电容安全边界\n"),
            ("body", "到达最高电压后禁止继续充电，到达最低电压后禁止继续放电；电流参考同时受 ±Imax 限制。能量耗尽后无法只靠提高 PI 增益维持功率上限。\n"),
        ]
        for tag, content in sections:
            self.text.insert("end", content, tag)

    def _number(self, key: str) -> float:
        try:
            return float(self.fields[key].get())
        except ValueError:
            return 0.0

    def _field_changed(self, *_args) -> None:
        self._state = None
        self.after_idle(self._draw_mode_bar)

    def set_operating_state(self, vin: float, vout: float, buck_duty: float, boost_duty: float) -> None:
        self._state = (vin, vout, buck_duty, boost_duty)
        self._draw_mode_bar()

    def set_application_mode(self, mode: str) -> None:
        self.application_mode = mode
        self._state = None
        self._draw_mode_bar()

    def _draw_mode_bar(self) -> None:
        canvas = self.mode_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 270)
        left, right = 14, width - 14
        top, bottom = 24, 56
        max_ratio = 1.6
        duty_max = min(max(self._number("duty_max"), 0.1), 0.99)
        low = min(0.90, duty_max)
        high = 1.10
        px = lambda ratio: left + (right - left) * min(max(ratio, 0.0), max_ratio) / max_ratio

        canvas.create_rectangle(px(0), top, px(low), bottom, fill="#bfe4f0", outline="")
        canvas.create_rectangle(px(low), top, px(high), bottom, fill="#ffe0a6", outline="")
        canvas.create_rectangle(px(high), top, px(max_ratio), bottom, fill="#c9e8d3", outline="")
        canvas.create_text((px(0) + px(low)) / 2, 40, text="BUCK", fill="#155f75", font=("Arial", 8, "bold"))
        canvas.create_text((px(low) + px(high)) / 2, 40, text="混合", fill="#8a5a00", font=("Microsoft YaHei UI", 8, "bold"))
        canvas.create_text((px(high) + px(max_ratio)) / 2, 40, text="BOOST", fill="#26734d", font=("Arial", 8, "bold"))

        if self._state is None:
            vin = self._number("vin")
            vout_key = "cap_voltage_initial" if self.application_mode == "power" else "vref"
            vout = self._number(vout_key)
            ratio = vout / max(vin, 0.1)
            buck, boost = ratio_to_duties(ratio, 0.0, duty_max)
        else:
            vin, vout, buck, boost = self._state
            ratio = vout / max(vin, 0.1)
        marker = px(ratio)
        canvas.create_line(marker, 16, marker, 65, fill="#d94f4f", width=2)
        canvas.create_polygon(marker, 65, marker - 5, 58, marker + 5, 58, fill="#d94f4f", outline="")
        canvas.create_text(left, 73, anchor="w", text="M=0", fill="#667788", font=("Arial", 8))
        canvas.create_text(right, 73, anchor="e", text=f"M≥{max_ratio:g}", fill="#667788", font=("Arial", 8))
        canvas.create_text(
            width / 2,
            91,
            text=f"当前 M={ratio:.3f}   Dₗ={buck:.3f}   Dᵣ={1.0-boost:.3f}",
            fill="#a13d3d",
            font=("Consolas", 9, "bold"),
        )

