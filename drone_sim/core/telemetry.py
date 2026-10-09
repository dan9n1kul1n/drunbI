import csv
from pathlib import Path
import numpy as np


class Telemetry:
    """Сбор и экспорт телеметрии полёта.

    Хранит историю параметров в памяти, умеет сохранять в CSV
    и строить графики (matplotlib) для отчёта.
    """

    FIELDS = [
        "time",
        "pos_x", "pos_y", "pos_z",
        "vel_x", "vel_y", "vel_z",
        "roll_deg", "pitch_deg", "yaw_deg",
        "p", "q", "r",
        "throttle", "roll_cmd", "pitch_cmd", "yaw_cmd",
        "wind_x", "wind_y",
        "battery_pct", "motor_temp_c",
        "gps_valid",
    ]

    def __init__(self):
        self.rows = []
        self.events = []  # лог событий (срабатывание автопилота, потеря GPS и т.д.)

    def log(self, state, commands, wind, battery_pct, motor_temp_c, gps_valid):
        """Записать один кадр телеметрии."""
        throttle, roll_cmd, pitch_cmd, yaw_cmd = commands
        self.rows.append({
            "time": state.time,
            "pos_x": state.position[0],
            "pos_y": state.position[1],
            "pos_z": state.position[2],
            "vel_x": state.velocity[0],
            "vel_y": state.velocity[1],
            "vel_z": state.velocity[2],
            "roll_deg": np.degrees(state.attitude[0]),
            "pitch_deg": np.degrees(state.attitude[1]),
            "yaw_deg": np.degrees(state.attitude[2]),
            "p": state.angular_velocity[0],
            "q": state.angular_velocity[1],
            "r": state.angular_velocity[2],
            "throttle": throttle,
            "roll_cmd": roll_cmd,
            "pitch_cmd": pitch_cmd,
            "yaw_cmd": yaw_cmd,
            "wind_x": wind[0],
            "wind_y": wind[1],
            "battery_pct": battery_pct,
            "motor_temp_c": motor_temp_c,
            "gps_valid": int(gps_valid),
        })

    def add_event(self, time: float, text: str):
        """Записать событие (например, «потеря GPS», «автопилот включён»)."""
        self.events.append({"time": time, "event": text})

    def to_csv(self, path: str | Path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"Телеметрия сохранена: {path}")

    def to_events_log(self, path: str | Path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            for ev in self.events:
                f.write(f"[{ev['time']:.2f} с] {ev['event']}\n")
        print(f"Лог событий сохранён: {path}")

    def to_plots(self, path: str | Path):
        """Построить графики: высота, скорости, углы, газ."""
        try:
            import matplotlib
            matplotlib.use("Agg")  # без окна — просто сохранить в файл
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib не установлен — пропускаю построение графиков")
            return

        if not self.rows:
            return

        t = [r["time"] for r in self.rows]
        z = [r["pos_z"] for r in self.rows]
        vx = [r["vel_x"] for r in self.rows]
        vy = [r["vel_y"] for r in self.rows]
        vz = [r["vel_z"] for r in self.rows]
        roll = [r["roll_deg"] for r in self.rows]
        pitch = [r["pitch_deg"] for r in self.rows]
        yaw = [r["yaw_deg"] for r in self.rows]
        thr = [r["throttle"] for r in self.rows]
        bat = [r["battery_pct"] for r in self.rows]

        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

        axes[0].plot(t, z, label="Высота Z")
        axes[0].set_ylabel("Высота, м")
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(t, vx, label="Vx")
        axes[1].plot(t, vy, label="Vy")
        axes[1].plot(t, vz, label="Vz")
        axes[1].set_ylabel("Скорость, м/с")
        axes[1].legend()
        axes[1].grid(True)

        axes[2].plot(t, roll, label="Крен")
        axes[2].plot(t, pitch, label="Тангаж")
        axes[2].plot(t, yaw, label="Рыскание")
        axes[2].set_ylabel("Углы, °")
        axes[2].legend()
        axes[2].grid(True)

        axes[3].plot(t, thr, label="Газ", color="black")
        axes[3].plot(t, [b / 100 for b in bat], label="Батарея", color="green")
        axes[3].set_ylabel("Газ / Батарея")
        axes[3].set_xlabel("Время, с")
        axes[3].legend()
        axes[3].grid(True)

        fig.tight_layout()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=100)
        plt.close(fig)
        print(f"Графики сохранены: {path}")
