extends Node

var drones := ["quadcopter_x", "medium", "heavy"]
var weathers := ["calm", "wind4", "wind7", "rain"]
var polygons := ["field", "mountains", "city"]

var drone_labels := {
	"quadcopter_x": "Лёгкий (1.2 кг)",
	"medium": "Средний (30 кг)",
	"heavy": "Тяжёлый (150 кг)"
}

var weather_labels := {
	"calm": "Штиль",
	"wind4": "Ветер 4 м/с",
	"wind7": "Ветер 7 м/с",
	"rain": "Дождь"
}

var polygon_labels := {
	"field": "Поле",
	"mountains": "Горы",
	"city": "Город"
}

var selected_drone: String = "quadcopter_x"
var selected_weather: String = "wind4"
var selected_polygon: String = "field"

signal settings_changed

func set_drone(name: String) -> void:
	selected_drone = name
	settings_changed.emit()

func set_weather(name: String) -> void:
	selected_weather = name
	settings_changed.emit()

func set_polygon(name: String) -> void:
	selected_polygon = name
	settings_changed.emit()

func get_drone_label(name: String) -> String:
	return drone_labels.get(name, name)

func get_weather_label(name: String) -> String:
	return weather_labels.get(name, name)

func get_polygon_label(name: String) -> String:
	return polygon_labels.get(name, name)
