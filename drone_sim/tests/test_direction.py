import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from core.drone_profile import DroneProfile
from core.state import DroneState
from core.physics import PhysicsEngine
from core.mixer import MotorMixer
from core.forces import GRAVITY


def probe(roll_deg, pitch_deg, label):
    """Дать дрону фиксированный крен/тангаж, посмотреть куда он полетит."""
    root = Path(__file__).parent.parent
    profile = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    engine = PhysicsEngine(profile)
    mixer = MotorMixer(max_thrust_per_motor=profile.data["motors"]["max_thrust_n"])

    # Стартуем в воздухе, с заданной ориентацией, без скорости.
    state = DroneState(
        position=np.array([0.0, 0.0, 10.0]),
        attitude=np.array([np.radians(roll_deg), np.radians(pitch_deg), 0.0]),
    )

    # Газ на уровне висения.
    hover_per_motor = profile.mass * GRAVITY / 4
    throttle_n = hover_per_motor
    thrusts = mixer.mix(throttle_n, 0.0, 0.0, 0.0)

    dt = 0.005
    for _ in range(400):  # 2 секунды
        state = engine.step_rk4(state, thrusts, np.zeros(3), dt)

    print(f"{label:35s} → смещение dX={state.position[0]:+7.3f}, dY={state.position[1]:+7.3f}")


if __name__ == "__main__":
    print("=== Проверка направления движения при наклонах ===\n")
    probe(0, 0, "Ровно (эталон)")
    probe(+10, 0, "Крен вправо +10° (roll>0)")
    probe(-10, 0, "Крен влево -10° (roll<0)")
    probe(0, +10, "Нос вверх +10° (pitch>0)")
    probe(0, -10, "Нос вниз -10° (pitch<0)")
    print()
    print("=== Как читать результат ===")
    print("- Если при roll>0 дрон летит в +X → для полёта в +X нужен roll>0")
    print("- Если при pitch>0 дрон летит в -Y → для полёта в +Y нужен pitch<0")
    print("- И наоборот. Знаки target_roll_auto / target_pitch_auto подбираем по факту.")
