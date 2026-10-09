import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from core.drone_profile import DroneProfile
from core.state import DroneState
from core.physics import PhysicsEngine
from core.sensors import SensorSuite
from core.forces import GRAVITY


def test_imu_at_rest():
    """Дрон висит неподвижно. Акселерометр должен показать ~+g вверх."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)
    sensors = SensorSuite(seed=42)

    state = DroneState(position=np.array([0, 0, 10.0]))
    hover_thrust = np.full(4, profile.mass * GRAVITY / 4)

    # В висении тяга уравновешивает гравитацию: specific_force = +g вверх.
    sf = engine.specific_force(state, hover_thrust, np.zeros(3))

    # Прогоняем 50 измерений, усредняем — шум должен уйти.
    accel_sum = np.zeros(3)
    for _ in range(50):
        m = sensors.imu.measure(state, sf, hover_thrust)
        accel_sum += m["accel"]
    accel_avg = accel_sum / 50

    print(f"Акселерометр (среднее за 50 замеров): {accel_avg.round(3)} м/с²")
    print(f"Ожидание: ~[0, 0, {GRAVITY}]")
    assert abs(accel_avg[2] - GRAVITY) < 0.2, "Вертикальная составляющая ~g"
    assert abs(accel_avg[0]) < 0.1, "Горизонтальные ~0"
    assert abs(accel_avg[1]) < 0.1
    print("✓ IMU в покое показывает ~g вверх")


def test_gyro_noise():
    """Гироскоп на неподвижном дроне должен показывать шум вокруг нуля."""
    sensors = SensorSuite(seed=1)
    profile_path = Path(__file__).parent.parent / "configs" / "drones" / "quadcopter_x.json"
    profile = DroneProfile.from_json(profile_path)
    engine = PhysicsEngine(profile)

    state = DroneState()
    sf = engine.specific_force(state, np.zeros(4), np.zeros(3))

    readings = []
    for _ in range(200):
        m = sensors.imu.measure(state, sf, np.zeros(4))
        readings.append(m["gyro"])
    readings = np.array(readings)

    std = readings.std(axis=0)
    mean = readings.mean(axis=0)
    print(f"Гироскоп: среднее {mean.round(4)}, стд {std.round(4)}")
    assert np.all(np.abs(mean) < 0.01), "Среднее ~0"
    assert np.all(std < 0.02), "Шум в разумных пределах"
    print("✓ Гироскоп шумит вокруг нуля")


def test_barometer():
    """Барометр измеряет высоту с шумом ~0.1 м."""
    sensors = SensorSuite(seed=2)
    state = DroneState(position=np.array([0, 0, 25.0]))

    readings = [sensors.baro.measure(state, 0.01)["altitude"] for _ in range(100)]
    readings = np.array(readings)

    print(f"Барометр: среднее {readings.mean():.3f}, стд {readings.std():.3f}")
    assert abs(readings.mean() - 25.0) < 0.5, "Среднее около истинной высоты"
    assert readings.std() < 0.5, "Шум разумный"
    print("✓ Барометр работает")


def test_gps_rate():
    """GPS должен обновляться реже, чем вызывается measure."""
    sensors = SensorSuite(seed=3)
    state = DroneState(position=np.array([10, 20, 5]))
    dt = 0.01

    updates = 0
    for _ in range(100):  # 1 секунда
        m = sensors.gps.measure(state, dt)
        if m is not None and m["time"] == state.time:
            updates += 1
        state.time += dt

    print(f"GPS обновился {updates} раз за 1 с (ожидается ~10)")
    assert 8 <= updates <= 12, "GPS работает на ~10 Гц"
    print("✓ GPS обновляется с правильной частотой")


def test_full_suite():
    """Полный цикл: физика + сенсоры, 1 секунда симуляции."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)
    sensors = SensorSuite(seed=7)

    state = DroneState(position=np.array([0, 0, 10.0]))
    hover = np.full(4, profile.mass * GRAVITY / 4)
    dt = 0.01

    for i in range(100):  # 1 секунда
        sf = engine.specific_force(state, hover, np.zeros(3))
        m = sensors.update(state, sf, dt, hover)
        state = engine.step_rk4(state, hover, np.zeros(3), dt)

    print(f"Финальное состояние: pos={state.position.round(2)}")
    print(f"Последнее измерение IMU accel: {m['imu']['accel'].round(3)}")
    print(f"Последнее измерение баро: {m['baro']['altitude']:.3f} м")
    assert m["gps"] is not None, "GPS должен был обновиться хотя бы раз"
    print("✓ Полный цикл сенсоров работает")


if __name__ == "__main__":
    test_imu_at_rest()
    test_gyro_noise()
    test_barometer()
    test_gps_rate()
    test_full_suite()
    print("\n=== Все тесты сенсоров пройдены ===")
