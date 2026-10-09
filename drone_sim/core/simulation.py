from pathlib import Path
import numpy as np

from .drone_profile import DroneProfile
from .state import DroneState
from .physics import PhysicsEngine
from .mixer import MotorMixer
from .weather import Weather
from .sensors import SensorSuite
from .autopilot import Autopilot
from .battery import Battery, MotorTemperature
from .telemetry import Telemetry
from .forces import GRAVITY


class Simulation:
    """Главный цикл симуляции БПЛА."""

    def __init__(self,
                 profile: DroneProfile,
                 weather: Weather,
                 dt: float = 0.005,
                 duration: float = 30.0,
                 seed: int | None = None):
        self.profile = profile
        self.weather = weather
        self.dt = dt
        self.duration = duration

        self.physics = PhysicsEngine(profile)
        self.mixer = MotorMixer(max_thrust_per_motor=profile.data["motors"]["max_thrust_n"])
        self.sensors = SensorSuite(seed=seed)
        self.autopilot = Autopilot()

        self.battery = Battery(
            capacity_mah=profile.data["battery"]["capacity_mah"],
            voltage_v=profile.data["battery"]["voltage_v"],
            max_current_a=profile.data["battery"]["max_current_a"],
        )
        self.motors = MotorTemperature(ambient_c=weather.temperature_c)

        self.telemetry = Telemetry()
        self.state = DroneState()
        self._motor_thrusts = np.zeros(4)

        # Команды пилота (по умолчанию — ничего).
        self.pilot_cmd = {
            "throttle": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

    def set_pilot_command(self, throttle: float = 0.0, roll: float = 0.0,
                          pitch: float = 0.0, yaw: float = 0.0):
        self.pilot_cmd = {"throttle": throttle, "roll": roll,
                          "pitch": pitch, "yaw": yaw}

    def run(self, target_altitude: float = 10.0):
        """Запустить симуляцию на заданное время."""
        self.state = DroneState(position=np.array([0.0, 0.0, 0.0]))
        self.autopilot.set_mode(Autopilot.MODE_ALTITUDE)
        self.autopilot.set_target_altitude(target_altitude)
        self.autopilot.set_target_position(0.0, 0.0)  # держимся в точке старта

        print(f"=== Симуляция: {self.profile.data['name']} ===")
        print(f"Погода: {self.weather}")
        print(f"Целевая высота: {target_altitude} м, длительность: {self.duration} с")
        print(f"Шаг: {self.dt} с, шагов: {int(self.duration / self.dt)}")
        print()

        self.telemetry.add_event(0.0, "Симуляция запущена")

        n_steps = int(self.duration / self.dt)
        last_gps_valid = False

        for step in range(n_steps):
            # 1. Опрос сенсоров (для будущей оценки состояния и логов).
            sf = self.physics.specific_force(
                self.state,
                self._motor_thrusts,
                self.weather.wind_vector(),
                self.weather.thrust_multiplier(),
                self.weather.drag_multiplier(),
            )
            measurements = self.sensors.update(
                self.state, sf, self.dt, self._motor_thrusts
            )

            gps_valid = measurements["gps"] is not None
            if gps_valid != last_gps_valid:
                self.telemetry.add_event(
                    self.state.time,
                    "GPS восстановлен" if gps_valid else "GPS потерян"
                )
                last_gps_valid = gps_valid

            # 2. Автопилот.
            throttle, roll, pitch, yaw = self.autopilot.update(
                throttle_cmd=self.pilot_cmd["throttle"],
                roll_cmd=self.pilot_cmd["roll"],
                pitch_cmd=self.pilot_cmd["pitch"],
                yaw_cmd=self.pilot_cmd["yaw"],
                attitude=self.state.attitude,
                angular_velocity=self.state.angular_velocity,
                altitude=self.state.position[2],
                dt=self.dt,
                position=self.state.position[:2],   # x, y для Position Hold
            )

            # 3. Миксер: команды → тяги моторов.
            throttle_n = throttle * self.profile.data["motors"]["max_thrust_n"]
            thrusts = self.mixer.mix(throttle_n, roll, pitch, yaw)
            self._motor_thrusts = thrusts

            # 4. Батарея и температура моторов.
            total_thrust_n = float(np.sum(thrusts))
            hover_total = self.profile.mass * GRAVITY
            current = self.battery.consume(total_thrust_n, hover_total, self.dt)
            motor_temp = self.motors.update(
                current,
                self.profile.data["battery"]["max_current_a"],
                self.dt,
            )

            # 5. Физика: шаг вперёд.
            self.weather.update(self.dt)
            self.state = self.physics.step_rk4(
                self.state, thrusts,
                self.weather.wind_vector(), self.dt,
                drag_multiplier=self.weather.drag_multiplier(),
                thrust_multiplier=self.weather.thrust_multiplier(),
            )

            # 6. Телеметрия.
            self.telemetry.log(
                self.state,
                (throttle, roll, pitch, yaw),
                self.weather.wind_vector(),
                self.battery.percent,
                motor_temp,
                gps_valid,
            )

            # Печать раз в секунду.
            if step % int(1.0 / self.dt) == 0:
                print(f"[{self.state.time:6.2f} с] "
                      f"pos=({self.state.position[0]:7.2f},{self.state.position[1]:7.2f},{self.state.position[2]:6.2f}) "
                      f"roll={np.degrees(self.state.attitude[0]):6.2f}° "
                      f"pitch={np.degrees(self.state.attitude[1]):6.2f}° "
                      f"bat={self.battery.percent:5.1f}% "
                      f"T={motor_temp:5.1f}°C")

        print()
        print("=== Симуляция завершена ===")

    def save_report(self, out_dir: str | Path):
        out_dir = Path(out_dir)
        self.telemetry.to_csv(out_dir / "telemetry.csv")
        self.telemetry.to_events_log(out_dir / "events.log")
        self.telemetry.to_plots(out_dir / "plots.png")
