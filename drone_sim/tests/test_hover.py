import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from core.drone_profile import DroneProfile
from core.state import DroneState
from core.physics import PhysicsEngine
from core.forces import GRAVITY


def test_hover():
    """Если суммарная тяга = вес, дрон должен висеть без ускорения."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)

    # Стартуем с высоты 10 м, без ветра
    state = DroneState(position=np.array([0, 0, 10.0]))
    wind = np.zeros(3)

    # Тяга = вес (по 1/4 на мотор)
    hover_thrust_per_motor = profile.mass * GRAVITY / 4
    thrusts = np.full(4, hover_thrust_per_motor)

    dt = 0.001
    for i in range(2000):  # 2 секунды
        state = engine.step_rk4(state, thrusts, wind, dt)

    print(f"Через 2 сек: {state}")
    print(f"Высота: {state.position[2]:.4f} м (ожидалось 10.0)")
    print(f"Скорость vz: {state.velocity[2]:.6f} м/с (ожидалось 0)")

    assert abs(state.position[2] - 10.0) < 0.01, "Дрон должен висеть на месте"
    assert abs(state.velocity[2]) < 0.01, "Вертикальная скорость должна быть ~0"
    print("✓ Тест висения пройден")


def test_climb():
    """При тяге больше веса дрон должен набирать высоту."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)

    state = DroneState(position=np.array([0, 0, 10.0]))
    wind = np.zeros(3)

    # Тяга на 20% больше веса
    thrusts = np.full(4, profile.mass * GRAVITY * 1.2 / 4)

    dt = 0.001
    for i in range(2000):
        state = engine.step_rk4(state, thrusts, wind, dt)

    print(f"\nПри избытке тяги: {state}")
    print(f"Высота: {state.position[2]:.4f} м (должна вырасти)")
    print(f"Скорость vz: {state.velocity[2]:.4f} м/с (должна быть > 0)")

    assert state.position[2] > 10.5, "Дрон должен подняться"
    assert state.velocity[2] > 0, "Вертикальная скорость должна быть положительной"
    print("✓ Тест набора высоты пройден")


if __name__ == "__main__":
    test_hover()
    test_climb()
    print("\n=== Все тесты физики пройдены ===")