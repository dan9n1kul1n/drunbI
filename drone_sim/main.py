import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from core.drone_profile import DroneProfile
from core.weather import Weather
from core.simulation import Simulation


def main():
    root = Path(__file__).parent

    drone_name = sys.argv[1] if len(sys.argv) > 1 else "quadcopter_x"
    weather_name = sys.argv[2] if len(sys.argv) > 2 else "wind4"

    drone_path = root / "configs" / "drones" / f"{drone_name}.json"
    drone = DroneProfile.from_json(drone_path)

    # --- ЖЁСТКАЯ РАЗНИЦА ПОГОДЫ ---
    if weather_name == "calm":
        wind = 0.0
    elif weather_name == "wind4":
        wind = 4.0
    elif weather_name == "wind7":
        wind = 7.0
    elif weather_name == "rain":
        wind = 5.0
    else:
        wind = 4.0

    print(f"!!! ЗАПУСК: дрон={drone_name}, погода={weather_name}, ветер={wind} м/с !!!")

    weather = Weather(
        wind_speed_ms=wind,
        wind_direction_deg=315.0,
        gusts=True,
        turbulence=0.5,
        precipitation="rain" if weather_name == "rain" else "none",
        precipitation_intensity=0.5 if weather_name == "rain" else 0.0,
        temperature_c=15.0,
        seed=None,   # БЕЗ seed — каждый запуск РАЗНЫЙ
    )

    sim = Simulation(
        profile=drone,
        weather=weather,
        dt=0.005,
        duration=30.0,
        seed=None,
    )

    sim.run(target_altitude=10.0)

    out_dir = root / "output" / "logs"
    sim.save_report(out_dir)

    import shutil
    godot_dir = root / "godot-viewer"
    if godot_dir.exists():
        shutil.copy(out_dir / "telemetry.csv", godot_dir / "telemetry.csv")
        print(f"CSV скопирован в Godot")

    print(f"!!! ЗАВЕРШЕНО: ветер={wind} м/с !!!")


if __name__ == "__main__":
    main()
