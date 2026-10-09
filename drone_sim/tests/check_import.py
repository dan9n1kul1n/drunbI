import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from core import autopilot
from core.autopilot import Autopilot

print("Путь к модулю autopilot:")
print("  ", autopilot.__file__)
print()
print("Версия класса Autopilot (знак pid_pos_x в MODE_POSITION_HOLD):")

ap = Autopilot()
ap.set_mode(Autopilot.MODE_POSITION_HOLD)
ap.set_target_position(0.0, 0.0)

# Смотрим исходный код метода update — какая формула там реально.
import inspect
source = inspect.getsource(ap.update)
# Ищем строки, где определяется target_pitch_auto / target_roll_auto.
for line in source.splitlines():
    if "target_pitch_auto" in line or "target_roll_auto" in line:
        print("  ", line.strip())

print()
print("Файл main.py смотрит в:",
      Path(sys.argv[0]).resolve() if sys.argv and sys.argv[0] else "?")
print("sys.path[0]:", sys.path[0])
