import numpy as np
from .drone_profile import DroneProfile
from .rotation import rotation_matrix


GRAVITY = 9.81  # м/с²


def thrust_local(total_thrust_n: float) -> np.ndarray:
    """Вектор тяги в связанной системе.

    Соглашение о системе координат (принято для всего проекта):
      - мировая система: X — восток, Y — север, Z — ВВЕРХ
      - связанная система дрона: X — вперёд, Y — вправо, Z — ВВЕРХ
      - тяга винтов направлена вдоль +Z связанной системы (вверх)
      - гравитация в мировой системе: [0, 0, -mg]

    При нулевом повороте дрона тяга в мировой системе тоже направлена вверх (+Z).
    """
    return np.array([0.0, 0.0, total_thrust_n])


def gravity_world(mass: float) -> np.ndarray:
    """Сила тяжести в мировой системе (направлена вниз, -Z)."""
    return np.array([0.0, 0.0, -mass * GRAVITY])


def drag_world(velocity_world: np.ndarray, wind_world: np.ndarray,
               profile: DroneProfile) -> np.ndarray:
    """Сила аэродинамического сопротивления в мировой системе.

    Учитывает ветер: сопротивление зависит от скорости ОТНОСИТЕЛЬНО воздуха.
    v_rel = v_дрона - v_ветра. Если дрон летит точно со скоростью ветра,
    относительная скорость нулевая и сопротивления нет.
    """
    v_rel = velocity_world - wind_world
    speed = np.linalg.norm(v_rel)
    if speed < 1e-6:
        return np.zeros(3)

    d = profile.data["drag"]
    k1 = d["linear_coeff"]
    k2 = d["quadratic_coeff"]

    magnitude = k1 * speed + k2 * speed ** 2
    return -magnitude * (v_rel / speed)


def motor_moments(motor_thrusts: np.ndarray, profile: DroneProfile) -> np.ndarray:
    """Моменты (roll, pitch, yaw) от тяг моторов.

    motor_thrusts — массив тяг [Н] для каждого мотора.
    Позиции моторов берутся из профиля (в связанной системе).

    Момент от каждого мотора: M_i = r_i × F_i, где F_i = [0, 0, T_i].
    Суммарный момент — вектор (Mx, My, Mz) = (roll, pitch, yaw).

    Плюс реактивный момент винтов (yaw): для X-конфигурации чередование
    знаков [ + - + - ] даёт нулевой момент при равных тягах и ненулевой
    при разнице (это то, чем дрон управляет по рысканию).
    """
    positions = np.array(profile.data["motors"]["positions"])  # (N, 3)
    forces_local = np.zeros((len(motor_thrusts), 3))
    forces_local[:, 2] = motor_thrusts  # тяга вдоль +Z (вверх)

    # Момент = r × F. Суммируем по всем моторам.
    moments = np.zeros(3)
    for i in range(len(motor_thrusts)):
        moments += np.cross(positions[i], forces_local[i])

    # Реактивный момент от вращения винтов (упрощённая модель).
    # Знак чередуется: у X-конфигурации соседние винты крутятся в разные стороны.
    yaw_sign = np.array([+1, -1, +1, -1])[:len(motor_thrusts)]
    yaw_moment = np.sum(yaw_sign * motor_thrusts) * 0.02  # коэффициент подобрать
    moments[2] += yaw_moment

    return moments
