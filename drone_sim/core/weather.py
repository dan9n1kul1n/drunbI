import numpy as np


class Weather:
    """Модель погодных условий.

    Соглашение о системе координат:
      - мировая система: X — восток, Y — север, Z — вверх
      - ветер задаётся вектором в мировой системе [wx, wy, 0]

    Реализовано:
      - постоянная составляющая ветра (скорость + направление)
      - порывы (плавные колебания через сумму синусоид)
      - турбулентность (случайный шум с низкочастотной фильтрацией)
      - осадки (дождь/снег/град) — влияют на аэродинамику
      - температура — влияет на плотность воздуха и тягу
    """

    # Плотность воздуха при 15 °C на уровне моря, кг/м³
    RHO_0 = 1.225

    def __init__(self,
                 wind_speed_ms: float = 0.0,
                 wind_direction_deg: float = 0.0,
                 gusts: bool = False,
                 turbulence: float = 0.0,
                 precipitation: str = "none",   # "none", "rain", "snow", "hail"
                 precipitation_intensity: float = 0.0,  # 0..1
                 temperature_c: float = 15.0,
                 seed: int | None = None):
        self.wind_speed = wind_speed_ms
        # Направление: 0° = ветер дует С СЕВЕРА (т.е. в сторону юга, -Y).
        # Метеорологический стандарт: direction — откуда дует.
        self.wind_direction_deg = wind_direction_deg
        self.gusts = gusts
        self.turbulence = turbulence
        self.precipitation = precipitation
        self.precipitation_intensity = precipitation_intensity
        self.temperature_c = temperature_c

        self._rng = np.random.default_rng(seed)
        self._turb_state = np.zeros(3)  # сглаженное состояние турбулентности
        self.time = 0.0

    @property
    def air_density(self) -> float:
        """Плотность воздуха с учётом температуры (упрощённая модель)."""
        # При повышении температуры плотность падает (p = const).
        # T в Кельвинах: T_K = T_C + 273.15
        T_k = self.temperature_c + 273.15
        T_0 = 288.15  # 15 °C
        return self.RHO_0 * (T_0 / T_k)

    def wind_vector(self) -> np.ndarray:
        """Вектор ветра в мировой системе [wx, wy, 0].

        Направление: метеорологическое (откуда дует).
        0° (север) → ветер дует с севера на юг → вектор [0, -w, 0].
        90° (восток) → ветер дует с востока на запад → [-w, 0, 0].
        """
        theta = np.radians(self.wind_direction_deg)
        # Единичный вектор «откуда дует» — навстречу направлению ветра.
        # Если ветер с севера, он дует на юг: [0, -1, 0].
        direction = np.array([-np.sin(theta), -np.cos(theta), 0.0])

        base = direction * self.wind_speed

        # Порывы: плавные колебания скорости ветра.
        if self.gusts:
            gust = (np.sin(self.time * 0.7) * 0.5 +
                    np.sin(self.time * 1.9) * 0.3) * 2.0  # до ±1.6 м/с
            base += direction * gust

        # Турбулентность: случайный шум, сглаженный по времени.
        if self.turbulence > 0:
            noise = self._rng.normal(0, 1, 3)
            # Экспоненциальное сглаживание — турбулентность не «дёргается» резко.
            alpha = 0.05
            self._turb_state = (1 - alpha) * self._turb_state + alpha * noise
            base += self._turb_state * self.turbulence

        return base

    def drag_multiplier(self) -> float:
        """Множитель сопротивления из-за осадков.

        Дождь/снег/град увеличивают эффективное сопротивление, потому что
        частицы сталкиваются с корпусом и винтами. Числа условные, но
        монотонные: град > дождь > снег (снег легче, но липнет).
        """
        coefficients = {"none": 0.0, "rain": 0.3, "snow": 0.15, "hail": 0.5}
        return 1.0 + coefficients.get(self.precipitation, 0.0) * self.precipitation_intensity

    def thrust_multiplier(self) -> float:
        """Множитель тяги из-за плотности воздуха и осадков.

        Тяга винта пропорциональна плотности воздуха.
        Осадки ухудшают эффективность винта (смачивание, обледенение).
        """
        density_factor = self.air_density / self.RHO_0

        precip_factor = 1.0
        if self.precipitation == "rain":
            precip_factor -= 0.10 * self.precipitation_intensity
        elif self.precipitation == "snow":
            precip_factor -= 0.15 * self.precipitation_intensity
        elif self.precipitation == "hail":
            precip_factor -= 0.25 * self.precipitation_intensity

        return density_factor * precip_factor

    def update(self, dt: float) -> None:
        """Продвинуть время погоды (для порывов и турбулентности)."""
        self.time += dt

    def __repr__(self):
        return (f"<Weather wind={self.wind_speed} м/с "
                f"dir={self.wind_direction_deg}° "
                f"precip={self.precipitation} "
                f"T={self.temperature_c}°C>")
