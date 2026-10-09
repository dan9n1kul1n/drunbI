import numpy as np


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Матрица поворота из локальной (связанной) системы БПЛА в мировую.

    Порядок поворотов: Z (yaw) → Y (pitch) → X (roll), т.е. R = Rz * Ry * Rx.
    Это стандарт для аэрокосмических приложений (Tait-Bryan angles).
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def euler_rates_matrix(roll: float, pitch: float) -> np.ndarray:
    """Матрица перехода от угловых скоростей (p,q,r) к производным углов Эйлера.

    Нужна, потому что p,q,r — в связанной системе, а roll,pitch,yaw — в мировой.
    Формула стандартная, есть в любом учебнике по динамике полёта.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    tp = np.tan(pitch)

    return np.array([
        [1, sr * tp, cr * tp],
        [0, cr,      -sr],
        [0, sr / cp, cr / cp],
    ])