class PID:
    """ПИД-регулятор с анти-виндапом и ограничением выхода.

    Используется для:
      - стабилизации углов (roll, pitch, yaw)
      - удержания высоты (throttle)
      - удержания позиции (в перспективе)
    """

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
        """Один шаг регулятора.

        error — рассогласование (setpoint - measurement).
        dt — шаг по времени.
        Возвращает управляющее воздействие, ограниченное [output_min, output_max].
        """
        # Пропорциональная часть.
        p = self.kp * error

        # Интегральная часть с ограничением (анти-виндап).
        self._integral += error * dt
        self._integral = max(self.integral_min, min(self.integral_max, self._integral))
        i = self.ki * self._integral

        # Дифференциальная часть. На первом вызове пропускаем — иначе
        # получим огромный скачок из-за нулевого prev_error.
        if self._first_call:
            d = 0.0
            self._first_call = False
        else:
            d = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        output = p + i + d
        return max(self.output_min, min(self.output_max, output))

    def __repr__(self):
        return f"<PID kp={self.kp} ki={self.ki} kd={self.kd}>"
