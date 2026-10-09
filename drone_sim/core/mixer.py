import numpy as np


class MotorMixer:
    """Преобразует команды управления (throttle, roll, pitch, yaw)
    в тяги отдельных моторов для X-конфигурации квадрокоптера.

    Соглашение:
      - throttle: общая тяга, от 0 до max_thrust_per_motor
      - roll, pitch, yaw: добавки к тяге, обычно в диапазоне [-1, +1]
        (масштабируются коэффициентом authority)
      - результат ограничивается снизу нулём и сверху max_thrust_per_motor

    Конфигурация моторов (вид сверху, нос — вверх):

           T1(CCW)   T2(CW)
              \\     /
               \\   /
                [X]
               /   \\
              /     \\
           T4(CW)   T3(CCW)

    Знаки подобраны так, чтобы положительный roll = крен вправо,
    положительный pitch = нос вверх, положительный yaw = поворот вправо.
    """

    def __init__(self, max_thrust_per_motor: float,
                 roll_authority: float = 0.5,
                 pitch_authority: float = 0.5,
                 yaw_authority: float = 0.2):
        self.max_thrust = max_thrust_per_motor
        self.roll_authority = roll_authority
        self.pitch_authority = pitch_authority
        self.yaw_authority = yaw_authority

    def mix(self, throttle: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Вернуть массив тяг четырёх моторов [T1, T2, T3, T4] в ньютонах."""
        r = roll * self.roll_authority * self.max_thrust
        p = pitch * self.pitch_authority * self.max_thrust
        y = yaw * self.yaw_authority * self.max_thrust

        thrusts = np.array([
            throttle + r + p - y,   # T1 — передний левый
            throttle - r + p + y,   # T2 — передний правый
            throttle - r - p - y,   # T3 — задний правый
            throttle + r - p + y,   # T4 — задний левый
        ])

        # Ограничиваем физическими пределами моторов.
        thrusts = np.clip(thrusts, 0.0, self.max_thrust)
        return thrusts