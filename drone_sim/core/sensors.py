import numpy as np
from .state import DroneState
from .rotation import rotation_matrix


GRAVITY = 9.81


class IMU:
    """Инерциальный измерительный блок: акселерометр + гироскоп.

    Модель шума:
      - белый шум (гауссов)
      - постоянное смещение (bias) — главная причина дрейфа
      - вибрация от винтов (высокочастотный шум)

    Акселерометр измеряет УДЕЛЬНУЮ силу (specific force) в СВЯЗАННОЙ системе.
    В покое на столе он покажет +g вверх (реакцию опоры), а в свободном
    падении — 0.
    """

    def __init__(self,
                 accel_noise_std: float = 0.05,       # м/с²
                 gyro_noise_std: float = 0.005,       # рад/с
                 accel_bias: np.ndarray | None = None,
                 gyro_bias: np.ndarray | None = None,
                 vibration_amplitude: float = 0.1,
                 seed: int | None = None):
        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std
        self.accel_bias = accel_bias if accel_bias is not None else np.zeros(3)
        self.gyro_bias = gyro_bias if gyro_bias is not None else np.zeros(3)
        self.vibration_amplitude = vibration_amplitude
        self._rng = np.random.default_rng(seed)
        self.time = 0.0

    def measure(self, state: DroneState,
                specific_force_world: np.ndarray,
                motor_thrusts: np.ndarray | None = None) -> dict:
        """Вернуть измерения IMU.

        specific_force_world — ускорение от негравитационных сил в мировой
        системе (тяга + сопротивление, делённые на массу). Его надо
        получить из PhysicsEngine (см. интеграцию ниже).
        """
        R = rotation_matrix(*state.attitude)
        # Переводим удельную силу в связанную систему.
        accel_body = R.T @ specific_force_world

        # Добавляем смещение, шум и вибрацию.
        accel_measured = accel_body + self.accel_bias
        accel_measured += self._rng.normal(0, self.accel_noise_std, 3)

        # Вибрация: амплитуда зависит от суммарной тяги моторов.
        if motor_thrusts is not None and np.sum(motor_thrusts) > 0:
            vib = self._rng.normal(0, 1, 3) * self.vibration_amplitude
            accel_measured += vib

        gyro_measured = state.angular_velocity + self.gyro_bias
        gyro_measured += self._rng.normal(0, self.gyro_noise_std, 3)

        return {
            "accel": accel_measured,       # м/с², связанная система
            "gyro": gyro_measured,         # рад/с, связанная система
            "time": self.time,
        }

    def update(self, dt: float) -> None:
        self.time += dt


class Magnetometer:
    """Магнитометр: измеряет вектор магнитного поля Земли в связанной системе.

    Даёт абсолютную ориентацию по курсу (yaw), в отличие от гироскопа,
    который измеряет только изменение угла.
    """

    def __init__(self, declination_deg: float = 0.0,
                 noise_std: float = 0.01,
                 seed: int | None = None):
        # Магнитное поле Земли в мировой системе (приближённо, для средних широт).
        # Направлено на север и вниз (инклинация ~60°).
        self.field_world = np.array([0.0, 0.5, 0.8])  # условные единицы
        self.declination = np.radians(declination_deg)
        self.noise_std = noise_std
        self._rng = np.random.default_rng(seed)

    def measure(self, state: DroneState) -> dict:
        R = rotation_matrix(*state.attitude)
        # Поле в связанной системе.
        field_body = R.T @ self.field_world
        # Добавляем шум.
        field_body += self._rng.normal(0, self.noise_std, 3)
        return {"mag": field_body}


class Barometer:
    """Барометр: измеряет высоту по атмосферному давлению.

    Точность высокая (±0.1 м), но есть медленный дрейф из-за изменения
    погоды. Обновляется часто (50+ Гц).
    """

    def __init__(self, noise_std: float = 0.1, drift_rate: float = 0.01,
                 seed: int | None = None):
        self.noise_std = noise_std
        self.drift_rate = drift_rate  # м/с
        self._rng = np.random.default_rng(seed)
        self._drift = 0.0

    def measure(self, state: DroneState, dt: float) -> dict:
        self._drift += self._rng.normal(0, self.drift_rate * np.sqrt(dt))
        altitude = state.position[2] + self._drift
        altitude += self._rng.normal(0, self.noise_std)
        return {"altitude": altitude}


class GPS:
    """GPS-приёмник: позиция и скорость в мировой системе.

    Особенности:
      - обновляется редко (обычно 5–10 Гц), а не каждый шаг физики
      - ошибка позиции ~1.5 м по горизонтали, ~3 м по вертикали
      - ошибка скорости ~0.1 м/с
    """

    def __init__(self, update_rate_hz: float = 10.0,
                 pos_noise_std: float = 1.5,
                 vel_noise_std: float = 0.1,
                 seed: int | None = None):
        self.period = 1.0 / update_rate_hz
        self.pos_noise_std = pos_noise_std
        self.vel_noise_std = vel_noise_std
        self._rng = np.random.default_rng(seed)
        self._time_since_update = 0.0
        self._last_measurement = None

    def measure(self, state: DroneState, dt: float) -> dict | None:
        """Вернуть измерение или None, если GPS ещё не обновился."""
        self._time_since_update += dt
        if self._time_since_update < self.period:
            return self._last_measurement

        self._time_since_update = 0.0
        pos = state.position + self._rng.normal(0, self.pos_noise_std, 3)
        # Вертикальная ошибка GPS обычно больше горизонтальной.
        pos[2] += self._rng.normal(0, self.pos_noise_std)
        vel = state.velocity + self._rng.normal(0, self.vel_noise_std, 3)

        self._last_measurement = {
            "position": pos,
            "velocity": vel,
            "time": state.time,
        }
        return self._last_measurement


class Rangefinder:
    """Дальномер: расстояние до земли вниз.

    Работает только на малой высоте (до ~10 м), с шумом и редкими выпадениями.
    """

    def __init__(self, max_range: float = 10.0,
                 noise_std: float = 0.02,
                 dropout_prob: float = 0.02,
                 seed: int | None = None):
        self.max_range = max_range
        self.noise_std = noise_std
        self.dropout_prob = dropout_prob
        self._rng = np.random.default_rng(seed)

    def measure(self, state: DroneState) -> dict | None:
        alt = state.position[2]
        if alt > self.max_range:
            return None  # вне диапазона
        if self._rng.random() < self.dropout_prob:
            return None  # случайный пропуск
        distance = alt + self._rng.normal(0, self.noise_std)
        return {"distance": max(0.0, distance)}


class SensorSuite:
    """Объединяет все сенсоры в один блок.

    Использование:
        suite = SensorSuite()
        ...
        measurements = suite.update(state, specific_force_world, dt, motor_thrusts)
    """

    def __init__(self, seed: int | None = None):
        self.imu = IMU(seed=seed)
        self.mag = Magnetometer(seed=seed)
        self.baro = Barometer(seed=seed)
        self.gps = GPS(seed=seed)
        self.rangefinder = Rangefinder(seed=seed)
        self.time = 0.0

    def update(self, state: DroneState,
               specific_force_world: np.ndarray,
               dt: float,
               motor_thrusts: np.ndarray | None = None) -> dict:
        """Опрос всех сенсоров. Возвращает словарь с измерениями."""
        measurements = {
            "imu": self.imu.measure(state, specific_force_world, motor_thrusts),
            "mag": self.mag.measure(state),
            "baro": self.baro.measure(state, dt),
            "gps": self.gps.measure(state, dt),        # может быть None
            "rangefinder": self.rangefinder.measure(state),  # может быть None
            "time": self.time,
        }
        self.imu.update(dt)
        self.time += dt
        return measurements
