import json
from pathlib import Path


class DroneProfile:
    """Цифровой профиль БПЛА: загрузка, валидация, доступ к параметрам."""

    REQUIRED_FIELDS = ["name", "type", "mass_kg", "motors", "inertia"]

    def __init__(self, data: dict):
        self._validate(data)
        self.data = data

    def _validate(self, data: dict):
        for field in self.REQUIRED_FIELDS:
            if field not in data:
                raise ValueError(f"В профиле БПЛА отсутствует поле: {field}")
        if data["mass_kg"] <= 0:
            raise ValueError("Масса должна быть положительной")

    @classmethod
    def from_json(cls, path: str | Path) -> "DroneProfile":
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Файл профиля не найден: {path}")
        with open(path, "r", encoding="utf-8") as f:
            return cls(json.load(f))

    # Удобные свойства для доступа из физики
    @property
    def mass(self) -> float:
        return self.data["mass_kg"]

    @property
    def inertia(self) -> tuple[float, float, float]:
        i = self.data["inertia"]
        return i["ixx"], i["iyy"], i["izz"]

    @property
    def max_total_thrust(self) -> float:
        m = self.data["motors"]
        return m["count"] * m["max_thrust_n"]

    def __repr__(self):
        return f"<DroneProfile {self.data['name']} ({self.data['type']}, {self.mass} кг)>"
