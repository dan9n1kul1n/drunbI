from dataclasses import dataclass, field
import numpy as np


@dataclass
class DroneState:
    """Полное состояние БПЛА в момент времени t.

    Позиция и скорость — в мировой (инерциальной) системе.
    Ориентация — углы Эйлера (roll, pitch, yaw) в радианах.
    Угловые скорости — в локальной (связанной) системе.
    """
    # Линейное движение (мировая система)
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z (м)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # vx, vy, vz (м/с)

    # Угловое движение
    attitude: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw (рад)
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # p, q, r (рад/с)

    # Время симуляции
    time: float = 0.0

    def to_vector(self) -> np.ndarray:
        """Упаковать состояние в один вектор из 12 компонент (для RK4)."""
        return np.concatenate([
            self.position, self.velocity,
            self.attitude, self.angular_velocity
        ])

    @classmethod
    def from_vector(cls, v: np.ndarray, time: float) -> "DroneState":
        """Распаковать вектор обратно в состояние."""
        return cls(
            position=v[0:3].copy(),
            velocity=v[3:6].copy(),
            attitude=v[6:9].copy(),
            angular_velocity=v[9:12].copy(),
            time=time,
        )

    def __repr__(self):
        return (f"<State t={self.time:.2f} "
                f"pos={self.position.round(2)} "
                f"vel={self.velocity.round(2)} "
                f"att={np.degrees(self.attitude).round(1)}°>")