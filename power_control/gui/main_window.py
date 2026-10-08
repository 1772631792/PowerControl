"""Tk GUI for comparing single-loop and cascaded buck-boost control."""

from __future__ import annotations

import csv
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
matplotlib.rcParams["font.sans-serif"] = [
    "Microsoft YaHei",
    "SimHei",
    "Arial Unicode MS",
    "DejaVu Sans",
]
matplotlib.rcParams["axes.unicode_minus"] = False
matplotlib.rcParams["legend.frameon"] = False
matplotlib.rcParams["axes.titleweight"] = "medium"
matplotlib.rcParams["axes.labelcolor"] = "#515154"
matplotlib.rcParams["text.color"] = "#1d1d1f"
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from power_control.gui.circuit_canvas import CircuitDiagram
from power_control.gui.formula_panel import FormulaPanel
from power_control.gui.steady_curve_panel import SteadyCurvePanel
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


class BuckBoostApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Buck-Boost 双闭环控制仿真")
        self.geometry("1480x920")
        self.minsize(1180, 760)
        self.data = None
        self._fields: dict[str, tk.StringVar] = {}
        self._build_style()
        self._build_ui()
        self.after(150, self.run)

    def _build_style(self) -> None:
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        self.configure(background="#f5f5f7")
        style.configure(".", font=("Microsoft YaHei UI", 9), foreground="#1d1d1f")
        style.configure("App.TFrame", background="#f5f5f7")
        style.configure("Header.TFrame", background="#ffffff")
        style.configure("Tab.TFrame", background="#ffffff")
        style.configure("Card.TFrame", background="#ffffff", relief="flat")
        style.configure("TLabel", background="#ffffff", foreground="#1d1d1f")
        style.configure("Title.TLabel", background="#ffffff", foreground="#1d1d1f", font=("Microsoft YaHei UI", 17, "bold"))
        style.configure("Subtitle.TLabel", background="#ffffff", foreground="#6e6e73", font=("Microsoft YaHei UI", 9))
        style.configure("Badge.TLabel", background="#e8f2ff", foreground="#0071e3", padding=(10, 5), font=("Microsoft YaHei UI", 9, "bold"))
        style.configure("Metric.TLabel", background="#ffffff", foreground="#424245", font=("Consolas", 10), padding=8)
        style.configure("TLabelframe", background="#ffffff", bordercolor="#d2d2d7", lightcolor="#d2d2d7", darkcolor="#d2d2d7", relief="solid")
        style.configure("TLabelframe.Label", background="#ffffff", foreground="#1d1d1f", font=("Microsoft YaHei UI", 9, "bold"))
        style.configure("TNotebook", background="#f5f5f7", borderwidth=0, tabmargins=(0, 0, 0, 5))
        style.configure("TNotebook.Tab", background="#e8e8ed", foreground="#6e6e73", padding=(13, 8), borderwidth=0, font=("Microsoft YaHei UI", 9))
        style.map("TNotebook.Tab", background=[("selected", "#ffffff"), ("active", "#f2f2f7")], foreground=[("selected", "#1d1d1f")])
        style.configure("TEntry", fieldbackground="#ffffff", foreground="#1d1d1f", bordercolor="#c7c7cc", lightcolor="#c7c7cc", darkcolor="#c7c7cc", padding=(7, 5))
        style.map("TEntry", bordercolor=[("focus", "#0071e3")], lightcolor=[("focus", "#0071e3")], darkcolor=[("focus", "#0071e3")])
        style.configure("TRadiobutton", background="#ffffff", foreground="#1d1d1f", padding=(2, 3))
        style.map("TRadiobutton", background=[("active", "#ffffff")], indicatorcolor=[("selected", "#0071e3")])
        style.configure("Accent.TButton", background="#0071e3", foreground="#ffffff", borderwidth=0, padding=(14, 8), font=("Microsoft YaHei UI", 9, "bold"))
        style.map("Accent.TButton", background=[("active", "#0077ed"), ("pressed", "#0068d1"), ("disabled", "#a9cef5")])
        style.configure("Secondary.TButton", background="#e8e8ed", foreground="#1d1d1f", borderwidth=0, padding=(14, 8))
        style.map("Secondary.TButton", background=[("active", "#dedee3"), ("pressed", "#d1d1d6")])

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=14, style="App.TFrame")
        root.pack(fill="both", expand=True)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(1, weight=1)

        header = ttk.Frame(root, style="Header.TFrame", padding=(14, 10))
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="Power Control Lab", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, text="四开关双向 Buck-Boost · 超级电容功率控制仿真", style="Subtitle.TLabel").grid(row=1, column=0, sticky="w", pady=(2, 0))
        ttk.Label(header, text="FPWM · 双向功率", style="Badge.TLabel").grid(row=0, column=1, rowspan=2, sticky="e")

        controls = ttk.Frame(root, width=380, style="App.TFrame")
        controls.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        controls.grid_propagate(False)
        controls.rowconfigure(1, weight=1)
        controls.columnconfigure(0, weight=1)
        content = ttk.Frame(root, style="App.TFrame")
        content.grid(row=1, column=1, sticky="nsew")
        content.rowconfigure(1, weight=1)
        content.columnconfigure(0, weight=1)

        application_box = ttk.LabelFrame(controls, text="仿真应用", padding=7)
        application_box.grid(row=0, column=0, sticky="ew", pady=(0, 7))
        self.application_mode = tk.StringVar(value="power")
        ttk.Radiobutton(
            application_box, text="RoboMaster 车辆功率控制", variable=self.application_mode,
            value="power", command=self._update_application_ui,
        ).pack(anchor="w")
        ttk.Radiobutton(
            application_box, text="普通 Buck-Boost 输出稳压", variable=self.application_mode,
            value="voltage", command=self._update_application_ui,
        ).pack(anchor="w")

        notebook = ttk.Notebook(controls)
        notebook.grid(row=1, column=0, sticky="nsew")
        plant_tab = ttk.Frame(notebook, padding=12, style="Tab.TFrame")
        control_tab = ttk.Frame(notebook, padding=12, style="Tab.TFrame")
        scenario_tab = ttk.Frame(notebook, padding=12, style="Tab.TFrame")
        formula_tab = ttk.Frame(notebook, padding=12, style="Tab.TFrame")
        notebook.add(plant_tab, text="电路")
        notebook.add(control_tab, text="控制器")
        notebook.add(scenario_tab, text="车辆负载")
        notebook.add(formula_tab, text="公式")
        formula_tab.rowconfigure(0, weight=1)
        formula_tab.columnconfigure(0, weight=1)

        self.mode = tk.StringVar(value="cascade")
        self.mode_box = ttk.LabelFrame(control_tab, text="稳压控制策略", padding=8)
        self.mode_box.pack(fill="x", pady=(0, 7))
        ttk.Radiobutton(self.mode_box, text="级联：电压 PI + 电流 PI", variable=self.mode, value="cascade").pack(anchor="w")
        ttk.Radiobutton(self.mode_box, text="单环：电压 PID（对照）", variable=self.mode, value="single").pack(anchor="w")
        self.power_control_box = ttk.LabelFrame(control_tab, text="功率控制结构", padding=8)
        ttk.Label(
            self.power_control_box,
            text="车辆功率差 → 双向电流参考 → 电流 PI → 四开关调制",
            wraplength=300,
            justify="left",
        ).pack(anchor="w")

        plant_box = ttk.LabelFrame(plant_tab, text="四开关功率级", padding=8)
        plant_box.pack(fill="x")
        for key, label, value in [
            ("vin", "输入电压 Vin / V", "24"),
            ("l_uh", "电感 L / µH", "150"),
            ("current_limit", "电感限流 / A", "20"),
            ("duty_max", "PWM 最大占空比", "0.92"),
        ]:
            self._entry(plant_box, key, label, value)

        self.voltage_plant_box = ttk.LabelFrame(plant_tab, text="稳压模式专用", padding=8)
        self.voltage_plant_box.pack(fill="x", pady=7)
        for key, label, value in [
            ("vref", "目标电压 Vref / V", "36"),
            ("c_uf", "输出电容 C / µF", "470"),
            ("resistance", "初始负载 R / Ω", "12"),
        ]:
            self._entry(self.voltage_plant_box, key, label, value)

        current_gains_box = ttk.LabelFrame(control_tab, text="电流内环 PI", padding=8)
        current_gains_box.pack(fill="x", pady=7)
        for key, label, value in [
            ("inner_kp", "内环 Kp", "0.055"),
            ("inner_ki", "内环 Ki", "220"),
        ]:
            self._entry(current_gains_box, key, label, value)
        self.voltage_gains_box = ttk.LabelFrame(control_tab, text="稳压模式专用参数", padding=8)
        self.voltage_gains_box.pack(fill="x", pady=7)
        for key, label, value in [
            ("outer_kp", "外环 Kp / A·V⁻¹", "0.45"),
            ("outer_ki", "外环 Ki / A·V⁻¹·s⁻¹", "80"),
            ("single_kp", "单环 Kp", "0.035"),
            ("single_ki", "单环 Ki", "18"),
            ("single_kd", "单环 Kd", "0.00002"),
        ]:
            self._entry(self.voltage_gains_box, key, label, value)

        self.voltage_scenario_box = ttk.LabelFrame(scenario_tab, text="稳压模式扰动", padding=8)
        self.voltage_scenario_box.pack(fill="x")
        for key, label, value in [
            ("duration", "仿真时间 / s", "0.65"),
            ("load_step_time", "负载阶跃时刻 / s", "0.28"),
            ("load_after", "阶跃后负载 / Ω", "6"),
            ("input_step_time", "输入阶跃时刻 / s", "0.48"),
            ("input_after", "阶跃后输入 / V", "20"),
        ]:
            self._entry(self.voltage_scenario_box, key, label, value)

        self.power_scenario_box = ttk.LabelFrame(scenario_tab, text="车辆功率与超级电容", padding=8)
        for key, label, value in [
            ("power_duration", "仿真时间 / s", "2.5"),
            ("power_limit", "裁判系统功率上限 / W", "80"),
            ("vehicle_power_initial", "车辆初始功率 / W", "40"),
            ("vehicle_step_time", "车辆功率阶跃时刻 / s", "0.8"),
            ("vehicle_power_after", "阶跃后车辆功率 / W", "160"),
            ("cap_voltage_initial", "超电初始电压 / V", "18"),
            ("cap_voltage_min", "超电最低电压 / V", "8"),
            ("cap_voltage_max", "超电最高电压 / V", "23.5"),
            ("cap_capacitance", "超级电容容量 / F", "5"),
            ("vehicle_time_constant_ms", "车辆响应时间常数 / ms", "60"),
            ("power_filter_ms", "功率采样滤波 / ms", "6"),
            ("vehicle_power_ripple", "车辆动态波动幅值 / W", "6"),
            ("vehicle_ripple_frequency", "车辆动态波动频率 / Hz", "2"),
            ("current_slew_rate", "电流参考斜率 / A·s⁻¹", "250"),
        ]:
            self._entry(self.power_scenario_box, key, label, value)

        self.formula_panel = FormulaPanel(formula_tab, self._fields)
        self.formula_panel.grid(row=0, column=0, sticky="nsew")

        ttk.Label(
            plant_tab,
            text="提示：也可以在右侧电路图中双击彩色元件修改参数。",
            foreground="#52667a",
            wraplength=270,
            justify="left",
        ).pack(fill="x", pady=12)

        bottom = ttk.Frame(controls, style="Card.TFrame", padding=8)
        bottom.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        buttons = ttk.Frame(bottom, style="Card.TFrame")
        buttons.pack(fill="x")
        self.run_button = ttk.Button(buttons, text="运行仿真", command=self.run, style="Accent.TButton")
        self.run_button.pack(side="left", expand=True, fill="x", padx=(0, 4))
        ttk.Button(buttons, text="导出 CSV", command=self.export_csv, style="Secondary.TButton").pack(side="left", expand=True, fill="x")

        self.status = tk.StringVar(value="就绪")
        ttk.Label(bottom, textvariable=self.status, style="Metric.TLabel", justify="left").pack(fill="x", pady=8)

        circuit_box = ttk.LabelFrame(content, text="交互式功率级电路图", padding=5)
        circuit_box.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        circuit_box.columnconfigure(0, weight=1)
        self.circuit = CircuitDiagram(circuit_box, self._fields, height=285)
        self.circuit.grid(row=0, column=0, sticky="ew")

        result_notebook = ttk.Notebook(content)
        result_notebook.grid(row=1, column=0, sticky="nsew")
        plot_frame = ttk.Frame(result_notebook, padding=4, style="Tab.TFrame")
        curve_frame = ttk.Frame(result_notebook, padding=4, style="Tab.TFrame")
        result_notebook.add(plot_frame, text="时域波形")
        result_notebook.add(curve_frame, text="稳态电压－占空比")
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)
        self.figure = Figure(figsize=(10, 6), dpi=100, constrained_layout=True, facecolor="#fbfbfd")
        self.axes = self.figure.subplots(3, 1, sharex=True)
        self._style_axes(self.axes)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame, pack_toolbar=False)
        toolbar.update()
        toolbar.grid(row=1, column=0, sticky="ew")
        curve_frame.rowconfigure(0, weight=1)
        curve_frame.columnconfigure(0, weight=1)
        self.steady_curve_panel = SteadyCurvePanel(curve_frame, self._fields)
        self.steady_curve_panel.grid(row=0, column=0, sticky="nsew")
        self._update_application_ui()

    @staticmethod
    def _style_axes(axes) -> None:
        for axis in axes:
            axis.set_facecolor("#ffffff")
            axis.grid(True, color="#d2d2d7", alpha=0.48, linewidth=0.8)
            axis.tick_params(colors="#515154", labelsize=9)
            axis.spines["top"].set_visible(False)
            axis.spines["right"].set_visible(False)
            axis.spines["left"].set_color("#d2d2d7")
            axis.spines["bottom"].set_color("#d2d2d7")

    def _entry(self, parent, key: str, label: str, default: str) -> None:
        row = len(parent.grid_slaves()) // 2
        var = tk.StringVar(value=default)
        self._fields[key] = var
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 7), pady=2)
        ttk.Entry(parent, textvariable=var, width=10).grid(row=row, column=1, sticky="e", pady=2)

    def _configs(self):
        value = lambda key: float(self._fields[key].get())
        plant = PlantConfig(
            vin=value("vin"), vref=value("vref"),
            inductance=value("l_uh") * 1e-6, capacitance=value("c_uf") * 1e-6,
            resistance=value("resistance"), current_limit=value("current_limit"),
            duty_max=value("duty_max"),
        )
        control = ControlConfig(
            outer_kp=value("outer_kp"), outer_ki=value("outer_ki"),
            inner_kp=value("inner_kp"), inner_ki=value("inner_ki"),
            single_kp=value("single_kp"), single_ki=value("single_ki"),
            single_kd=value("single_kd"),
        )
        if self.application_mode.get() == "power":
            scenario = PowerScenarioConfig(
                duration=value("power_duration"), power_limit=value("power_limit"),
                vehicle_power_initial=value("vehicle_power_initial"),
                vehicle_step_time=value("vehicle_step_time"),
                vehicle_power_after=value("vehicle_power_after"),
                cap_voltage_initial=value("cap_voltage_initial"),
                cap_voltage_min=value("cap_voltage_min"),
                cap_voltage_max=value("cap_voltage_max"),
                cap_capacitance=value("cap_capacitance"),
                vehicle_time_constant=value("vehicle_time_constant_ms") * 1e-3,
                power_filter_time_constant=value("power_filter_ms") * 1e-3,
                vehicle_power_ripple=value("vehicle_power_ripple"),
                vehicle_ripple_frequency=value("vehicle_ripple_frequency"),
                current_slew_rate=value("current_slew_rate"),
            )
        else:
            scenario = ScenarioConfig(
                duration=value("duration"), load_step_time=value("load_step_time"),
                load_after=value("load_after"), input_step_time=value("input_step_time"),
                input_after=value("input_after"),
            )
        return plant, control, scenario

    def _update_application_ui(self) -> None:
        if not hasattr(self, "power_scenario_box"):
            return
        if self.application_mode.get() == "power":
            self.voltage_scenario_box.pack_forget()
            self.power_scenario_box.pack(fill="x")
            self.voltage_plant_box.pack_forget()
            self.mode_box.pack_forget()
            self.voltage_gains_box.pack_forget()
            self.power_control_box.pack(fill="x", pady=(0, 7))
        else:
            self.power_scenario_box.pack_forget()
            self.voltage_scenario_box.pack(fill="x")
            self.voltage_plant_box.pack(fill="x", pady=7)
            self.power_control_box.pack_forget()
            self.mode_box.pack(fill="x", pady=(0, 7))
            self.voltage_gains_box.pack(fill="x", pady=7)
        if hasattr(self, "circuit"):
            self.circuit.set_application_mode(self.application_mode.get())
        if hasattr(self, "formula_panel"):
            self.formula_panel.set_application_mode(self.application_mode.get())
        if hasattr(self, "steady_curve_panel"):
            self.steady_curve_panel.set_application_mode(self.application_mode.get())

    def run(self) -> None:
        try:
            configs = self._configs()
        except ValueError:
            messagebox.showerror("参数错误", "请输入有效的数字。")
            return
        self.run_button.configure(state="disabled")
        self.status.set("正在计算…")
        threading.Thread(
            target=self._worker,
            args=(*configs, self.application_mode.get(), self.mode.get()),
            daemon=True,
        ).start()

    def _worker(self, plant, control, scenario, application, mode) -> None:
        try:
            if application == "power":
                data = run_power_simulation(plant, control, scenario)
                result_metrics = power_metrics(data)
            else:
                data = run_simulation(plant, control, scenario, mode)
                result_metrics = metrics(data, plant.vref)
            self.after(0, lambda: self._show_result(data, result_metrics, application, mode))
        except Exception as exc:
            self.after(0, lambda: self._show_error(str(exc)))

    def _show_error(self, message: str) -> None:
        self.run_button.configure(state="normal")
        self.status.set("仿真失败")
        messagebox.showerror("仿真失败", message)

    def _show_result(self, data, result_metrics, application: str, mode: str) -> None:
        self.data = data
        self.figure.clear()
        self.axes = self.figure.subplots(3, 1, sharex=True)
        ax_v, ax_i, ax_d = self.axes
        self._style_axes(self.axes)

        if application == "power":
            ax_v.plot(data["time"], data["vehicle_power_command"], ":", label="驾驶功率指令", color="#c7a1a1", linewidth=1.0)
            ax_v.plot(data["time"], data["vehicle_power"], label="车辆功率", color="#ff3b30", linewidth=1.35)
            ax_v.plot(data["time"], data["input_power"], label="电源输入功率", color="#0071e3", linewidth=1.7)
            ax_v.plot(data["time"], data["power_limit"], "--", label="功率上限", color="#1d1d1f", linewidth=1.1)
            ax_v.set_ylabel("功率 / W")
            ax_v.legend(loc="best", ncols=3)
        else:
            ax_v.plot(data["time"], data["vout"], label="Vout", color="#0071e3", linewidth=1.7)
            ax_v.plot(data["time"], data["vref"], "--", label="Vref", color="#ff3b30", linewidth=1.1)
            ax_v.plot(data["time"], data["vin"], ":", label="Vin", color="#86868b")
            ax_v.set_ylabel("电压 / V")
            ax_v.legend(loc="best", ncols=3)

        ax_i.plot(data["time"], data["il"], label="电感电流", color="#ff9f0a", linewidth=1.6)
        if application == "power" or mode == "cascade":
            ax_i.plot(data["time"], data["iref"], "--", label="电流参考", color="#0071e3", linewidth=1.1)
        ax_i.set_ylabel("电流 / A")
        ax_i.legend(loc="best")
        if application == "power":
            cap_axis = ax_i.twinx()
            cap_axis.plot(data["time"], data["vcap"], label="超级电容电压", color="#af52de", alpha=0.88, linewidth=1.5)
            cap_axis.set_ylabel("超电电压 / V", color="#af52de")
            cap_axis.tick_params(axis="y", labelcolor="#af52de")
            cap_axis.spines["top"].set_visible(False)
            cap_axis.spines["right"].set_color("#d2d2d7")

        ax_d.plot(data["time"], data["buck_duty"] * 100, label="Buck 占空比", color="#0071e3", linewidth=1.6)
        ax_d.plot(data["time"], data["boost_duty"] * 100, label="Boost 占空比", color="#ff9f0a", linewidth=1.6)
        ax_d.set_ylabel("占空比 / %")
        ax_d.set_xlabel("时间 / s")
        ax_d.set_ylim(-3, 103)
        ax_d.legend(loc="best", ncols=2)
        self.canvas.draw_idle()
        shown_vout = float(data["vcap"][-1]) if application == "power" else float(data["vout"][-1])
        self.circuit.set_application_mode(application)
        self.circuit.set_operating_state(
            float(data["buck_duty"][-1]),
            float(data["boost_duty"][-1]),
            float(data["vin"][-1]),
            shown_vout,
        )
        self.formula_panel.set_operating_state(
            float(data["vin"][-1]),
            shown_vout,
            float(data["buck_duty"][-1]),
            float(data["boost_duty"][-1]),
        )
        self.steady_curve_panel.set_operating_state(
            float(data["vin"][-1]),
            shown_vout,
            float(data["buck_duty"][-1]),
            float(data["boost_duty"][-1]),
        )

        if application == "power":
            self.status.set(
                "车辆功率控制 + 电流内环\n"
                f"电源侧功率  {result_metrics['input_power']:.2f} W\n"
                f"功率误差  {result_metrics['power_error']:+.3f} W\n"
                f"超电末电压  {result_metrics['cap_voltage']:.2f} V\n"
                f"峰值电流  {result_metrics['peak_current']:.2f} A"
            )
        else:
            strategy = "级联双环" if mode == "cascade" else "单电压 PID"
            self.status.set(
                f"{strategy}\n"
                f"稳态电压  {result_metrics['final_voltage']:.3f} V\n"
                f"稳态误差  {result_metrics['steady_error']:+.3f} V\n"
                f"最大超调  {result_metrics['overshoot_percent']:.2f} %\n"
                f"后段纹波  {result_metrics['ripple_pp']:.3f} Vpp\n"
                f"峰值电流  {result_metrics['peak_current']:.2f} A"
            )
        self.run_button.configure(state="normal")

    def export_csv(self) -> None:
        if self.data is None:
            messagebox.showinfo("暂无数据", "请先运行一次仿真。")
            return
        path = filedialog.asksaveasfilename(
            title="导出仿真数据", defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")], initialfile="buckboost_simulation.csv",
        )
        if not path:
            return
        names = list(self.data.keys())
        with open(path, "w", newline="", encoding="utf-8-sig") as handle:
            writer = csv.writer(handle)
            writer.writerow(names)
            writer.writerows(zip(*(self.data[name] for name in names)))
        self.status.set(self.status.get() + f"\n已导出：{path}")



