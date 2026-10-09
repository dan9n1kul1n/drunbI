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


root = Path(__file__).parent.parent
profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
engine = PhysicsEngine(profile)
mixer = MotorMixer(max_thrust_per_motor=profile.data["motors"]["max_thrust_n"])
ap = Autopilot()
ap.set_mode(Autopilot.MODE_POSITION_HOLD)
ap.set_target_altitude(10.0)
ap.set_target_position(0.0, 0.0)

theta = np.radians(315.0)
wind = np.array([-np.sin(theta), -np.cos(theta), 0.0]) * 4.0

state = DroneState(position=np.array([0.0, 0.0, 0.0]))
dt = 0.005

print(f"{'t':>5} | {'pos_x':>7} {'pos_y':>7} | {'err_x':>7} {'err_y':>7} | "
      f"{'roll':>7} {'pitch':>7}")

for step in range(int(6.0 / dt)):  # 6 секунд — короче, чтобы не кувыркался
    t = step * dt

    pos_in = state.position[:2]
    err_x = ap.target_position[0] - pos_in[0]
    err_y = ap.target_position[1] - pos_in[1]

    throttle, roll, pitch, yaw = ap.update(
        throttle_cmd=0.0, roll_cmd=0.0, pitch_cmd=0.0, yaw_cmd=0.0,
        attitude=state.attitude,
        angular_velocity=state.angular_velocity,
        altitude=state.position[2],
        dt=dt,
        position=pos_in,
    )

    throttle_n = throttle * profile.data["motors"]["max_thrust_n"]
    thrusts = mixer.mix(throttle_n, roll, pitch, yaw)
    state = engine.step_rk4(state, thrusts, wind, dt)

    if step % int(1.0 / dt) == 0:
        # НЕ вызываем ПИД повторно! Только смотрим, что пришло и что вышло.
        print(f"{t:5.1f} | {pos_in[0]:7.2f} {pos_in[1]:7.2f} | "
              f"{err_x:7.2f} {err_y:7.2f} | "
              f"{np.degrees(roll):6.2f}° {np.degrees(pitch):6.2f}°")
