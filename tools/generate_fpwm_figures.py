"""Generate the diagrams embedded in the FPWM tutorial."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "docs" / "images"
OUTPUT.mkdir(parents=True, exist_ok=True)

try:
    font_path = font_manager.findfont("Microsoft YaHei", fallback_to_default=False)
    font_manager.fontManager.addfont(font_path)
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
except ValueError:
    plt.rcParams["font.sans-serif"] = ["SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def save_mapping() -> None:
    x = np.linspace(0.001, 0.99, 900)
    d_left = np.where(x <= 0.5, 2.0 * x, 1.0)
    d_right = np.where(x <= 0.5, 1.0, 2.0 - 2.0 * x)
    ratio = d_left / np.maximum(d_right, 1e-6)

    fig, axes = plt.subplots(2, 1, figsize=(9.2, 6.4), sharex=True, constrained_layout=True)
    axes[0].plot(x, d_left * 100, linewidth=2.4, label="左上管占空比 $D_L$")
    axes[0].plot(x, d_right * 100, linewidth=2.4, label="右上管占空比 $D_R$")
    axes[0].axvline(0.5, color="#555555", linestyle="--", linewidth=1)
    axes[0].text(0.25, 51, "Buck 区", ha="center", va="center", fontsize=12)
    axes[0].text(0.75, 51, "Boost 区", ha="center", va="center", fontsize=12)
    axes[0].set_ylabel("上管占空比 / %")
    axes[0].set_ylim(-3, 105)
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc="lower center", ncols=2)

    shown = ratio <= 4.0
    axes[1].plot(x[shown], ratio[shown], color="#8e5ab5", linewidth=2.4)
    axes[1].axvline(0.5, color="#555555", linestyle="--", linewidth=1)
    axes[1].axhline(1.0, color="#777777", linestyle=":", linewidth=1)
    axes[1].scatter([0.375], [0.75], color="#d94f4f", s=55, zorder=5)
    axes[1].annotate("24 V 与 18 V 的平衡点\nx=0.375，M=0.75", (0.375, 0.75), xytext=(0.10, 2.0), arrowprops={"arrowstyle": "->"})
    axes[1].set_xlabel("统一控制量 x")
    axes[1].set_ylabel("电压变换比 M")
    axes[1].set_ylim(0, 4)
    axes[1].grid(alpha=0.25)
    fig.suptitle("一个连续控制量同时生成左右半桥占空比", fontsize=15)
    fig.savefig(OUTPUT / "fpwm-unified-mapping.png", dpi=170)
    plt.close(fig)


def save_inductor_voltage() -> None:
    x = np.linspace(0.05, 0.95, 900)
    vin, vcap, inductance = 24.0, 18.0, 150e-6
    d_left = np.where(x <= 0.5, 2.0 * x, 1.0)
    d_right = np.where(x <= 0.5, 1.0, 2.0 - 2.0 * x)
    v_inductor = d_left * vin - d_right * vcap
    di_dt = v_inductor / inductance / 1000.0
    x_ff = (vcap / vin) / 2.0

    fig, ax = plt.subplots(figsize=(9.2, 4.8), constrained_layout=True)
    ax.plot(x, v_inductor, color="#087e8b", linewidth=2.6, label="电感平均电压")
    ax.axhline(0, color="#444444", linewidth=1)
    ax.axvline(x_ff, color="#d94f4f", linestyle="--", linewidth=1.4, label="电压比前馈平衡点")
    ax.fill_between(x, v_inductor, 0, where=v_inductor >= 0, color="#58b87a", alpha=0.18)
    ax.fill_between(x, v_inductor, 0, where=v_inductor < 0, color="#e36a6a", alpha=0.16)
    ax.annotate("x 增大：电流上升", (0.43, 2.64), xytext=(0.54, 8), arrowprops={"arrowstyle": "->"})
    ax.annotate("x 减小：电流下降", (0.32, -2.64), xytext=(0.10, -10), arrowprops={"arrowstyle": "->"})
    ax.annotate("x=0.375\n平均电感电压为 0", (x_ff, 0), xytext=(0.43, -7), arrowprops={"arrowstyle": "->"})
    ax.set_xlabel("统一控制量 x")
    ax.set_ylabel("电感平均电压 / V")
    ax.set_title("统一控制量本质上控制电感平均电压（Vin=24 V，Vcap=18 V）")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper left")
    second = ax.secondary_yaxis("right", functions=(lambda value: value / inductance / 1000.0, lambda value: value * inductance * 1000.0))
    second.set_ylabel("电流变化率 / kA/s")
    fig.savefig(OUTPUT / "fpwm-inductor-voltage.png", dpi=170)
    plt.close(fig)


def save_control_flow() -> None:
    fig, ax = plt.subplots(figsize=(12.5, 4.2), constrained_layout=True)
    ax.set_xlim(0, 12.5)
    ax.set_ylim(0, 4.2)
    ax.axis("off")
    boxes = [
        (0.25, "同步采样", "Vbus, Vcap\niL, Pvehicle"),
        (2.35, "功率计算", "Pconv*=Plimit\n−Pvehicle"),
        (4.45, "安全包络", "电压边界\n±电流限幅"),
        (6.55, "电流 PI", "iref → Δx\n抗积分饱和"),
        (8.65, "FPWM 映射", "x → DL, DR\n唯一映射器"),
        (10.75, "定时器更新", "互补 PWM\n死区/关断"),
    ]
    for xpos, title, detail in boxes:
        patch = FancyBboxPatch((xpos, 1.35), 1.55, 1.45, boxstyle="round,pad=0.04,rounding_size=0.08", facecolor="#edf5f7", edgecolor="#3f6973", linewidth=1.6)
        ax.add_patch(patch)
        ax.text(xpos + 0.775, 2.38, title, ha="center", va="center", fontsize=11, fontweight="bold")
        ax.text(xpos + 0.775, 1.78, detail, ha="center", va="center", fontsize=9)
    for index in range(len(boxes) - 1):
        start = boxes[index][0] + 1.58
        end = boxes[index + 1][0] - 0.05
        ax.add_patch(FancyArrowPatch((start, 2.08), (end, 2.08), arrowstyle="-|>", mutation_scale=13, linewidth=1.4, color="#555555"))
    ax.text(6.25, 3.55, "每个 PWM/控制周期按固定顺序执行", ha="center", fontsize=15)
    ax.text(9.45, 0.72, "PID 永远不直接操作 Q1～Q4", ha="center", fontsize=11, color="#b64b3a")
    ax.add_patch(FancyArrowPatch((9.45, 1.3), (9.45, 1.02), arrowstyle="-|>", mutation_scale=13, color="#b64b3a"))
    fig.savefig(OUTPUT / "fpwm-control-flow.png", dpi=170, transparent=False)
    plt.close(fig)


if __name__ == "__main__":
    save_mapping()
    save_inductor_voltage()
    save_control_flow()
    print(f"Generated FPWM figures in {OUTPUT}")
