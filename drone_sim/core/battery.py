class Battery:
    """Модель батареи и моторов.

    Не претендует на электрохимическую точность, но даёт монотонные
    и правдоподобные графики разряда и нагрева.
    """

    def __init__(self, capacity_mah: float, voltage_v: float, max_current_a: float):
        self.capacity_ah = capacity_mah / 1000.0
        self.voltage = voltage_v
        self.max_current = max_current_a

        self.charge_ah = self.capacity_ah  # сколько осталось
        self.initial_energy_wh = self.capacity_ah * voltage_v

    @property
    def percent(self) -> float:
        return 100.0 * self.charge_ah / self.capacity_ah

    def consume(self, total_thrust_n: float, hover_thrust_n: float, dt: float) -> float:
        """Потратить заряд за шаг dt. Возвращает текущий ток (А).

        Ток пропорционален тяге в степени 1.5 (грубо — винты при большей
        тяге крутятся быстрее и потребляют непропорционально больше).
        """
        if hover_thrust_n <= 0:
            ratio = 0.0
        else:
            ratio = total_thrust_n / (hover_thrust_n * 2)  # 2 — потолок газа

        current = self.max_current * (ratio ** 1.5)
        current = max(0.0, min(self.max_current, current))

        # Потраченный заряд: I * t / 3600 (А·с → А·ч)
        self.charge_ah -= current * dt / 3600.0
        self.charge_ah = max(0.0, self.charge_ah)
        return current


class MotorTemperature:
    """Упрощённая тепловая модель моторов.

    Нагрев пропорционален квадрату тока, охлаждение — разности
    с температурой окружающей среды.
    """

    def __init__(self, ambient_c: float = 25.0,
                 heat_rate: float = 0.5,
                 cooling_rate: float = 0.05):
        self.temperature = ambient_c
        self.ambient = ambient_c
        self.heat_rate = heat_rate
        self.cooling_rate = cooling_rate

    def update(self, current_a: float, max_current_a: float, dt: float) -> float:
        # Нагрев от тока (квадратично — как I²R).
        heat = self.heat_rate * (current_a / max_current_a) ** 2
        # Охлаждение — к температуре среды.
        cooling = self.cooling_rate * (self.temperature - self.ambient)
        self.temperature += (heat - cooling) * dt
        return self.temperature
