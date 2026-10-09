import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from core.drone_profile import DroneProfile
from core.state import DroneState
from core.physics import PhysicsEngine
from core.mixer import MotorMixer
from core.autopilot import Autopilot
from core.forces import GRAVITY


def test_angle_stabilization():
    """Дрон наклонён на 20° по крену, автопилот должен выровнять."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)
    mixer = MotorMixer(max_thrust_per_motor=profile.data["motors"]["max_thrust_n"])
    ap = Autopilot()
    ap.set_mode(Autopilot.MODE_ANGLE)

    state = DroneState(
        position=np.array([0, 0, 10.0]),
        attitude=np.array([np.radians(20), 0, 0]),  # крен 20° вправо
    )

    # Газ на уровне висения.
    hover_per_motor = profile.mass * GRAVITY / 4
    throttle_hover = hover_per_motor / profile.data["motors"]["max_thrust_n"]

    dt = 0.005
    max_roll_after = None

    for i in range(2000):  # 10 секунд
        # Пилот не даёт команд — цель: 0°.
        t, r, p, y = ap.update(
            throttle_cmd=0.0, roll_cmd=0.0, pitch_cmd=0.0, yaw_cmd=0.0,
            attitude=state.attitude,
            angular_velocity=state.angular_velocity,
            altitude=state.position[2],
            dt=dt,
        )
        # Газ: держим висение + автопилот может добавить.
        base_throttle = throttle_hover
        throttle_n = (base_throttle + t * 0.1) * profile.data["motors"]["max_thrust_n"]
        thrusts = mixer.mix(throttle_n, r, p, y)
        state = engine.step_rk4(state, thrusts, np.zeros(3), dt)

        if i == 1000:
            max_roll_after = abs(state.attitude[0])

    final_roll_deg = np.degrees(state.attitude[0])
    print(f"Крен через 10 с: {final_roll_deg:.3f}° (ожидается ~0)")
    assert abs(state.attitude[0]) < np.radians(3.0), "Крен должен быть погашен"
    print("✓ Стабилизация угла крена работает")


def test_altitude_hold():
    """Задаём целевую высоту 15 м — дрон должен туда подняться и держаться."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)
    mixer = MotorMixer(max_thrust_per_motor=profile.data["motors"]["max_thrust_n"])
    ap = Autopilot()
    ap.set_mode(Autopilot.MODE_ALTITUDE)
    ap.set_target_altitude(15.0)

    state = DroneState(position=np.array([0, 0, 5.0]))

    dt = 0.005
    for i in range(4000):  # 20 секунд
        t, r, p, y = ap.update(
            throttle_cmd=0.0, roll_cmd=0.0, pitch_cmd=0.0, yaw_cmd=0.0,
            attitude=state.attitude,
            angular_velocity=state.angular_velocity,
            altitude=state.position[2],
            dt=dt,
        )
        throttle_n = t * profile.data["motors"]["max_thrust_n"]
        thrusts = mixer.mix(throttle_n, r, p, y)
        state = engine.step_rk4(state, thrusts, np.zeros(3), dt)

    final_alt = state.position[2]
    print(f"Высота через 20 с: {final_alt:.2f} м (цель 15.0)")
    assert abs(final_alt - 15.0) < 1.0, "Должен держаться около 15 м"
    print("✓ Удержание высоты работает")


if __name__ == "__main__":
    test_angle_stabilization()
    test_altitude_hold()
    print("\n=== Все тесты автопилота пройдены ===")
