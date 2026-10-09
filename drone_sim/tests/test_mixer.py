import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from core.mixer import MotorMixer


def test_hover_mix():
    """При нулевых roll/pitch/yaw все моторы должны иметь равную тягу."""
    mixer = MotorMixer(max_thrust_per_motor=8.0)
    thrusts = mixer.mix(throttle=3.0, roll=0, pitch=0, yaw=0)
    print(f"Висение: {thrusts}")
    assert np.allclose(thrusts, 3.0), "Все моторы должны быть равны"
    print("✓ Тест висения миксера пройден")


def test_roll_right():
    """Положительный roll должен увеличить тягу левых моторов (T1, T4),
    чтобы дрон наклонился вправо."""
    mixer = MotorMixer(max_thrust_per_motor=8.0)
    thrusts = mixer.mix(throttle=3.0, roll=0.5, pitch=0, yaw=0)
    print(f"Крен вправо: {thrusts}")
    assert thrusts[0] > 3.0, "T1 (передний левый) должен усилиться"
    assert thrusts[3] > 3.0, "T4 (задний левый) должен усилиться"
    assert thrusts[1] < 3.0, "T2 (передний правый) должен ослабнуть"
    assert thrusts[2] < 3.0, "T3 (задний правый) должен ослабнуть"
    print("✓ Тест крена пройден")


def test_pitch_up():
    """Положительный pitch (нос вверх) — усиливаем задние моторы (T3, T4)."""
    mixer = MotorMixer(max_thrust_per_motor=8.0)
    thrusts = mixer.mix(throttle=3.0, roll=0, pitch=0.5, yaw=0)
    print(f"Тангаж вверх: {thrusts}")
    assert thrusts[0] > 3.0, "T1 (передний левый) должен усилиться"
    assert thrusts[1] > 3.0, "T2 (передний правый) должен усилиться"
    assert thrusts[2] < 3.0, "T3 (задний правый) должен ослабнуть"
    assert thrusts[3] < 3.0, "T4 (задний левый) должен ослабнуть"
    print("✓ Тест тангажа пройден")


def test_saturation():
    """Команды не должны выходить за пределы мотора."""
    mixer = MotorMixer(max_thrust_per_motor=8.0)
    thrusts = mixer.mix(throttle=7.0, roll=1.0, pitch=1.0, yaw=1.0)
    print(f"Насыщение: {thrusts}")
    assert np.all(thrusts >= 0.0), "Тяга не может быть отрицательной"
    assert np.all(thrusts <= 8.0), "Тяга не может превышать максимум мотора"
    print("✓ Тест насыщения пройден")


if __name__ == "__main__":
    test_hover_mix()
    test_roll_right()
    test_pitch_up()
    test_saturation()
    print("\n=== Все тесты миксера пройдены ===")
