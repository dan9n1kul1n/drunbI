import sys
from pathlib import Path

# Добавляем корень проекта в путь поиска модулей,
# чтобы работал импорт "from core.xxx import ...".
sys.path.insert(0, str(Path(__file__).parent))

from core.drone_profile import DroneProfile
from core.weather import Weather
from core.simulation import Simulation


def main():
    root = Path(__file__).parent

    # --- Загрузка профиля дрона ---
    drone = DroneProfile.from_json(root / "configs" / "drones" / "quadcopter_x.json")
    print(f"Загружен профиль: {drone}")

    # --- Погода ---
    # Ветер 2 м/с с северо-запада (метеорологическое направление: откуда дует),
    # с порывами и умеренной турбулентностью, без осадков, +15 °C.
    weather = Weather(
        wind_speed_ms=2.0,
        wind_direction_deg=315.0,   # северо-запад
        gusts=True,
        turbulence=0.3,
        precipitation="none",
        temperature_c=15.0,
        seed=42,
    )

    # --- Симуляция ---
    sim = Simulation(
        profile=drone,
        weather=weather,
        dt=0.005,          # шаг физики 5 мс (200 Гц)
        duration=30.0,     # 30 секунд полёта
        seed=42,
    )

    sim.run(target_altitude=10.0)

    # --- Сохранение отчёта ---
    out_dir = root / "output" / "logs"
    sim.save_report(out_dir)


if __name__ == "__main__":
    main()
