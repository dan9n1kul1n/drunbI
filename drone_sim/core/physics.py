import numpy as np
from .state import DroneState
from .drone_profile import DroneProfile
from .rotation import rotation_matrix, euler_rates_matrix
from .forces import gravity_world, drag_world, thrust_local, motor_moments


class PhysicsEngine:
    """Ядро физики: расчёт производной состояния и интеграция методом RK4.

    Учитывает:
      - тягу моторов (масштабируется погодой: плотность воздуха, осадки)
      - гравитацию
      - аэродинамическое сопротивление относительно ветра
      - моменты от моторов (roll, pitch, yaw)
      - гироскопический эффект (omega × I·omega)

    Соглашение о системе координат:
      - мировая: X — восток, Y — север, Z — вверх
      - связанная: X — вперёд, Y — вправо, Z — вверх
      - тяга винтов направлена вдоль +Z связанной системы
      - гравитация: [0, 0, -mg]
    """

    def __init__(self, profile: DroneProfile):
        self.profile = profile
        self.mass = profile.mass
        self.inertia = np.diag(profile.inertia)  # матрица инерции (диагональная)
        self.inertia_inv = np.linalg.inv(self.inertia)

    def derivative(self, state: DroneState,
                   motor_thrusts: np.ndarray,
                   wind_world: np.ndarray,
                   drag_multiplier: float = 1.0,
                   thrust_multiplier: float = 1.0) -> np.ndarray:
        """Правая часть системы ОДУ: d(state)/dt.

        drag_multiplier и thrust_multiplier приходят из Weather:
        они моделируют влияние осадков и плотности воздуха.
        По умолчанию 1.0 — то есть без погодных поправок.
        """
        roll, pitch, yaw = state.attitude
        R = rotation_matrix(roll, pitch, yaw)

        # --- Линейное движение ---
        # Тяга масштабируется погодой (плотность воздуха, осадки).
        total_thrust = float(np.sum(motor_thrusts)) * thrust_multiplier
        T_local = thrust_local(total_thrust)
        T_world = R @ T_local

        F_gravity = gravity_world(self.mass)
        F_drag = drag_world(state.velocity, wind_world, self.profile) * drag_multiplier * 2.5

        F_total = T_world + F_gravity + F_drag
        accel = F_total / self.mass

        # --- Угловое движение ---
        M = motor_moments(motor_thrusts, self.profile)
        angular_accel = self.inertia_inv @ (M - np.cross(
            state.angular_velocity, self.inertia @ state.angular_velocity
        ))

        # --- Кинематика ориентации ---
        # Угловые скорости (p,q,r) — в связанной системе,
        # углы Эйлера (roll,pitch,yaw) — в мировой. Матрица E связывает их.
        E = euler_rates_matrix(roll, pitch)
        attitude_rates = E @ state.angular_velocity

        return np.concatenate([
            state.velocity,     # d(pos)/dt = vel
            accel,              # d(vel)/dt = a
            attitude_rates,     # d(att)/dt = E * omega
            angular_accel,      # d(omega)/dt = I⁻¹ * (M - ω×Iω)
        ])

    def step_rk4(self, state: DroneState, motor_thrusts: np.ndarray,
                 wind_world: np.ndarray, dt: float,
                 drag_multiplier: float = 1.0,
                 thrust_multiplier: float = 1.0) -> DroneState:
        """Один шаг интеграции методом Рунге-Кутты 4-го порядка.

        RK4 точнее и стабильнее метода Эйлера при том же шаге — критично
        для симуляции быстрых вращений.

        Параметры drag_multiplier и thrust_multiplier имеют значения по умолчанию,
        поэтому старые вызовы step_rk4(state, thrusts, wind, dt) продолжают работать.
        """
        y0 = state.to_vector()

        def f(y):
            s = DroneState.from_vector(y, state.time)
            return self.derivative(s, motor_thrusts, wind_world,
                                   drag_multiplier, thrust_multiplier)

        k1 = f(y0)
        k2 = f(y0 + 0.5 * dt * k1)
        k3 = f(y0 + 0.5 * dt * k2)
        k4 = f(y0 + dt * k3)

        y_new = y0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return DroneState.from_vector(y_new, state.time + dt)

    def specific_force(self, state: DroneState,
                       motor_thrusts: np.ndarray,
                       wind_world: np.ndarray,
                       thrust_multiplier: float = 1.0,
                       drag_multiplier: float = 1.0) -> np.ndarray:
        """Удельная (негравитационная) сила в МИРОВОЙ системе, м/с².

        Это то, что «чувствует» акселерометр: (тяга + сопротивление) / масса.
        Гравитацию НЕ включаем — акселерометр в свободном падении показывает 0,
        а в покое на столе — реакцию опоры (+g вверх).

        Используется сенсорами (IMU) для формирования реалистичных измерений.
        """
        roll, pitch, yaw = state.attitude
        R = rotation_matrix(roll, pitch, yaw)

        total_thrust = float(np.sum(motor_thrusts)) * thrust_multiplier
        T_world = R @ thrust_local(total_thrust)
        F_drag = drag_world(state.velocity, wind_world, self.profile) * drag_multiplier

        return (T_world + F_drag) / self.mass
