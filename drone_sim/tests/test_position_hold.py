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


def run_case(wind_speed, label):
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)
    mixer = MotorMixer(max_thrust_per_motor=profile.data["motors"]["max_thrust_n"])
    ap = Autopilot()
    ap.set_mode(Autopilot.MODE_POSITION_HOLD)
    ap.set_target_altitude(10.0)
    ap.set_target_position(0.0, 0.0)

    if wind_speed > 0:
        theta = np.radians(315.0)
        wind = np.array([-np.sin(theta), -np.cos(theta), 0.0]) * wind_speed
    else:
        wind = np.zeros(3)

    state = DroneState(position=np.array([0.0, 0.0, 0.0]))
    dt = 0.005

    print(f"\n=== {label} (ветер {wind_speed} м/с) ===")
    print(f"{'t':>5} | {'pos_x':>7} {'pos_y':>7} {'pos_z':>6} | "
          f"{'roll':>6} {'pitch':>6}")

    for step in range(int(10.0 / dt)):
        t = step * dt

        throttle, roll, pitch, yaw = ap.update(
            throttle_cmd=0.0, roll_cmd=0.0, pitch_cmd=0.0, yaw_cmd=0.0,
            attitude=state.attitude,
            angular_velocity=state.angular_velocity,
            altitude=state.position[2],
            dt=dt,
            position=state.position[:2],
        )

        throttle_n = throttle * profile.data["motors"]["max_thrust_n"]
        thrusts = mixer.mix(throttle_n, roll, pitch, yaw)
        state = engine.step_rk4(state, thrusts, wind, dt)

        if step % int(1.0 / dt) == 0 and step > 0:
            print(f"{t:5.1f} | "
                  f"{state.position[0]:7.2f} {state.position[1]:7.2f} {state.position[2]:6.2f} | "
                  f"{np.degrees(state.attitude[0]):5.1f}° {np.degrees(state.attitude[1]):5.1f}°")


if __name__ == "__main__":
    run_case(0.0, "БЕЗ ВЕТРА")
    run_case(7.0, "С ВЕТРОМ 7 м/с")
