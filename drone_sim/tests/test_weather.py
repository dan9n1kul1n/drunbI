import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from core.weather import Weather
from core.drone_profile import DroneProfile
from core.state import DroneState
from core.physics import PhysicsEngine
from core.forces import GRAVITY


def test_wind_direction():
    """Ветер с севера (0°) должен дуть на юг: [0, -w, 0]."""
    w = Weather(wind_speed_ms=7.0, wind_direction_deg=0.0)
    v = w.wind_vector()
    print(f"Ветер с севера 7 м/с: {v}")
    assert v[0] == 0.0
    assert v[1] == -7.0
    assert v[2] == 0.0
    print("✓ Направление ветра корректно")


def test_wind_direction_west():
    """Ветер с запада (270°) дует на восток: [+w, 0, 0]."""
    w = Weather(wind_speed_ms=5.0, wind_direction_deg=270.0)
    v = w.wind_vector()
    print(f"Ветер с запада 5 м/с: {v.round(3)}")
    assert v[0] > 4.99
    assert abs(v[1]) < 0.01
    print("✓ Западный ветер корректен")


def test_air_density_temperature():
    """При нагреве плотность воздуха падает."""
    cold = Weather(temperature_c=-10.0)
    hot = Weather(temperature_c=40.0)
    print(f"Плотность при -10°C: {cold.air_density:.4f}")
    print(f"Плотность при +40°C: {hot.air_density:.4f}")
    assert cold.air_density > hot.air_density
    print("✓ Влияние температуры на плотность корректно")


def test_drift_with_wind():
    """Дрон висит без управления. Боковой ветер должен его сносить."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)

    weather = Weather(wind_speed_ms=7.0, wind_direction_deg=0.0)  # с севера
    state = DroneState(position=np.array([0, 0, 10.0]))
    hover_thrust = np.full(4, profile.mass * GRAVITY / 4)

    dt = 0.001
    for i in range(3000):  # 3 секунды
        weather.update(dt)
        state = engine.step_rk4(
            state, hover_thrust, weather.wind_vector(), dt,
            drag_multiplier=weather.drag_multiplier(),
            thrust_multiplier=weather.thrust_multiplier(),
        )

    print(f"После 3 с в ветре: pos = {state.position.round(2)}")
    print(f"Снос по Y (ожидается в минус): {state.position[1]:.2f} м")
    assert state.position[1] < -0.5, "Дрон должен снести на юг"
    print("✓ Снос ветром работает")


def test_temperature_reduces_thrust():
    """В жару при той же команде дрон должен подниматься медленнее."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)

    def climb_height(temp):
        weather = Weather(temperature_c=temp)
        state = DroneState(position=np.array([0, 0, 10.0]))
        thrust = np.full(4, profile.mass * GRAVITY * 1.2 / 4)
        dt = 0.001
        for i in range(2000):
            weather.update(dt)
            state = engine.step_rk4(
                state, thrust, np.zeros(3), dt,
                drag_multiplier=weather.drag_multiplier(),
                thrust_multiplier=weather.thrust_multiplier(),
            )
        return state.position[2]

    h_cold = climb_height(-10.0)
    h_hot = climb_height(40.0)
    print(f"Высота через 2 с при -10°C: {h_cold:.2f} м")
    print(f"Высота через 2 с при +40°C: {h_hot:.2f} м")
    assert h_cold > h_hot, "В жару подъём должен быть медленнее"
    print("✓ Температура влияет на тягу")


if __name__ == "__main__":
    test_wind_direction()
    test_wind_direction_west()
    test_air_density_temperature()
    test_drift_with_wind()
    test_temperature_reduces_thrust()
    print("\n=== Все тесты погоды пройдены ===")
