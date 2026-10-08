"""Buck-Boost 仿真器的唯一启动入口。"""

from power_control.gui.main_window import BuckBoostApp


def main() -> None:
    BuckBoostApp().mainloop()


if __name__ == "__main__":
    main()

