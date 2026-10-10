import subprocess
import sys
import os

def get_base():
    if getattr(sys, 'frozen', False) and hasattr(sys, '_MEIPASS'):
        return sys._MEIPASS
    return os.path.dirname(os.path.abspath(__file__))

if __name__ == "__main__":
    base = get_base()
    godot_exe = os.path.join(base, "godot-viewer.exe")
    print(f"Запуск: {godot_exe}")
    subprocess.run([godot_exe], cwd=base)
