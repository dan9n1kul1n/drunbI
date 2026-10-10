class PID:
    """ПИД-регулятор с анти-виндапом и ограничением выхода."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_min: float = -1.0, integral_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def update(self, error: float, dt: float) -> float:
        """Один шаг регулятора с анти-виндапом.

        Анти-виндап: если выход уже уперся в потолок, а ошибка того же знака,
        интеграл НЕ накапливается. Это предотвращает «откат» после насыщения.
        """
        # Пропорциональная часть.
        p = self.kp * error

        # Дифференциальная часть.
        if self._first_call:
            d = 0.0
            self._first_call = False
        else:
            d = self.kd * (error - self._prev_error) / dt

        # Предварительный выход без интеграла.
        output_without_i = p + d

        # Проверка насыщения.
        with_i = output_without_i + self.ki * self._integral
        would_saturate_high = with_i >= self.output_max
        would_saturate_low = with_i <= self.output_min

        # Накапливаем интеграл только вне насыщения.
        if not ((would_saturate_high and error > 0) or (would_saturate_low and error < 0)):
            self._integral += error * dt
            self._integral = max(self.integral_min, min(self.integral_max, self._integral))

        i = self.ki * self._integral
        self._prev_error = error

        output = p + i + d
        return max(self.output_min, min(self.output_max, output))

    def __repr__(self):
        return f"<PID kp={self.kp} ki={self.ki} kd={self.kd}>"
