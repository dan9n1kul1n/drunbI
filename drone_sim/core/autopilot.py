
import numpy as np
from .pid import PID


class Autopilot:
    """Автопилот: стабилизация углов + удержание высоты + удержание позиции."""

    MODE_RATE = "rate"
    MODE_ANGLE = "angle"
    MODE_ALTITUDE = "altitude"
    MODE_POSITION_HOLD = "position_hold"

    def __init__(self,
                 max_roll_deg: float = 15.0,
                 max_pitch_deg: float = 15.0,
                 max_yaw_rate_deg: float = 90.0,
                 drone_mass_kg: float = 1.2):
        self.mode = self.MODE_ANGLE
        self.mass = drone_mass_kg

        self.max_roll = np.radians(max_roll_deg)
        self.max_pitch = np.radians(max_pitch_deg)
        self.max_yaw_rate = np.radians(max_yaw_rate_deg)

        self.pid_roll_rate = PID(kp=0.5, ki=0.05, kd=0.005)
        self.pid_pitch_rate = PID(kp=0.5, ki=0.05, kd=0.005)
        self.pid_yaw_rate = PID(kp=0.8, ki=0.05, kd=0.0)

        self.pid_roll_angle = PID(kp=6.0, ki=0.0, kd=1.5)
        self.pid_pitch_angle = PID(kp=6.0, ki=0.0, kd=1.5)

        self.pid_altitude = PID(kp=0.4, ki=0.05, kd=0.3,
                                output_min=-0.5, output_max=0.5,
                                integral_min=-1.0, integral_max=1.0)

        # Коэффициенты по позиции — масштабируются по массе.
        # Умножаем на 0.5, чтобы дрон НЕ компенсировал ветер на 100% —
        # ветер должен сносить его с курса.
        mass_scale = (1.2 / self.mass) ** 0.35 * 0.5

        self.pid_pos_x = PID(kp=0.15 * mass_scale,
                             ki=0.005 * mass_scale,
                             kd=0.0,
                             output_min=-1.0, output_max=1.0,
                             integral_min=-0.3, integral_max=0.3)
        self.pid_pos_y = PID(kp=0.15 * mass_scale,
                             ki=0.005 * mass_scale,
                             kd=0.0,
                             output_min=-1.0, output_max=1.0,
                             integral_min=-0.3, integral_max=0.3)

        self.target_altitude = 0.0
        self.target_position = np.zeros(2)

    def set_mode(self, mode: str):
        self.mode = mode
        for pid in [self.pid_roll_rate, self.pid_pitch_rate, self.pid_yaw_rate,
                    self.pid_roll_angle, self.pid_pitch_angle,
                    self.pid_altitude, self.pid_pos_x, self.pid_pos_y]:
            pid.reset()

    def set_target_altitude(self, altitude_m: float):
        self.target_altitude = altitude_m

    def set_target_position(self, x: float, y: float):
        self.target_position = np.array([x, y])

    def update(self,
               throttle_cmd: float,
               roll_cmd: float,
               pitch_cmd: float,
               yaw_cmd: float,
               attitude: np.ndarray,
               angular_velocity: np.ndarray,
               altitude: float,
               dt: float,
               position: np.ndarray | None = None) -> tuple[float, float, float, float]:
        roll, pitch, yaw = attitude
        p, q, r = angular_velocity

        if self.mode == self.MODE_RATE:
            target_p = roll_cmd * self.max_yaw_rate
            target_q = pitch_cmd * self.max_yaw_rate
            target_r = yaw_cmd * self.max_yaw_rate
            roll_out = self.pid_roll_rate.update(target_p - p, dt)
            pitch_out = self.pid_pitch_rate.update(target_q - q, dt)
            yaw_out = self.pid_yaw_rate.update(target_r - r, dt)
            throttle_out = throttle_cmd

        elif self.mode == self.MODE_ANGLE:
            target_roll = roll_cmd * self.max_roll
            target_pitch = pitch_cmd * self.max_pitch
            target_p = self.pid_roll_angle.update(target_roll - roll, dt)
            target_q = self.pid_pitch_angle.update(target_pitch - pitch, dt)
            target_r = yaw_cmd * self.max_yaw_rate
            roll_out = self.pid_roll_rate.update(target_p - p, dt)
            pitch_out = self.pid_pitch_rate.update(target_q - q, dt)
            yaw_out = self.pid_yaw_rate.update(target_r - r, dt)
            throttle_out = throttle_cmd

        elif self.mode == self.MODE_ALTITUDE:
            target_roll = roll_cmd * self.max_roll
            target_pitch = pitch_cmd * self.max_pitch
            target_p = self.pid_roll_angle.update(target_roll - roll, dt)
            target_q = self.pid_pitch_angle.update(target_pitch - pitch, dt)
            target_r = yaw_cmd * self.max_yaw_rate
            roll_out = self.pid_roll_rate.update(target_p - p, dt)
            pitch_out = self.pid_pitch_rate.update(target_q - q, dt)
            yaw_out = self.pid_yaw_rate.update(target_r - r, dt)
            hover_throttle = 0.5
            alt_correction = self.pid_altitude.update(self.target_altitude - altitude, dt)
            throttle_out = hover_throttle + alt_correction + throttle_cmd * 0.3
            throttle_out = max(0.0, min(1.0, throttle_out))

        elif self.mode == self.MODE_POSITION_HOLD:
            if position is None:
                raise ValueError("Position Hold требует position=(x, y)")

            err_x = self.target_position[0] - position[0]
            err_y = self.target_position[1] - position[1]

            # Ограничиваем автономный наклон 6° — дрон наклоняется против ветра,
            # но НЕ настолько, чтобы полностью компенсировать снос.
            max_auto_angle = np.radians(6.0)

            target_pitch_auto = np.clip(
                self.pid_pos_x.update(err_x, dt) * self.max_pitch,
                -max_auto_angle, max_auto_angle
            )
            target_roll_auto = np.clip(
                -self.pid_pos_y.update(err_y, dt) * self.max_roll,
                -max_auto_angle, max_auto_angle
            )

            target_roll = np.clip(
                target_roll_auto + roll_cmd * self.max_roll,
                -np.radians(10.0), np.radians(10.0)
            )
            target_pitch = np.clip(
                target_pitch_auto + pitch_cmd * self.max_pitch,
                -np.radians(10.0), np.radians(10.0)
            )

            target_p = self.pid_roll_angle.update(target_roll - roll, dt)
            target_q = self.pid_pitch_angle.update(target_pitch - pitch, dt)
            target_r = yaw_cmd * self.max_yaw_rate

            roll_out = self.pid_roll_rate.update(target_p - p, dt)
            pitch_out = self.pid_pitch_rate.update(target_q - q, dt)
            yaw_out = self.pid_yaw_rate.update(target_r - r, dt)

            hover_throttle = 0.5
            alt_correction = self.pid_altitude.update(self.target_altitude - altitude, dt)
            throttle_out = hover_throttle + alt_correction + throttle_cmd * 0.3
            throttle_out = max(0.0, min(1.0, throttle_out))

        else:
            raise ValueError(f"Неизвестный режим автопилота: {self.mode}")

        roll_out = max(-1.0, min(1.0, roll_out))
        pitch_out = max(-1.0, min(1.0, pitch_out))
        yaw_out = max(-1.0, min(1.0, yaw_out))

        return throttle_out, roll_out, pitch_out, yaw_out
