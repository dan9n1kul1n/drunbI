import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from core import autopilot
import inspect

print("Файл:", autopilot.__file__)
print("=" * 70)

# Печатаем весь блок MODE_POSITION_HOLD из метода update.
ap = autopilot.Autopilot()
source = inspect.getsource(ap.update)

inside = False
for line in source.splitlines():
    if "MODE_POSITION_HOLD" in line:
        inside = True
    if inside:
        print(line)
    if inside and "target_r =" in line:
        break
