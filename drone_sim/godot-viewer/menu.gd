extends Control

@onready var drone_dropdown: OptionButton = $VBoxContainer/DroneDropdown
@onready var weather_dropdown: OptionButton = $VBoxContainer/WeatherDropdown
@onready var polygon_dropdown: OptionButton = $VBoxContainer/PolygonDropdown
@onready var start_button: Button = $VBoxContainer/StartButton
@onready var quit_button: Button = $VBoxContainer/QuitButton


func _ready() -> void:
	for d in GameState.drones:
		drone_dropdown.add_item(GameState.get_drone_label(d))
	for w in GameState.weathers:
		weather_dropdown.add_item(GameState.get_weather_label(w))
	for p in GameState.polygons:
		polygon_dropdown.add_item(GameState.get_polygon_label(p))

	drone_dropdown.select(GameState.drones.find(GameState.selected_drone))
	weather_dropdown.select(GameState.weathers.find(GameState.selected_weather))
	polygon_dropdown.select(GameState.polygons.find(GameState.selected_polygon))

	drone_dropdown.item_selected.connect(_on_drone_selected)
	weather_dropdown.item_selected.connect(_on_weather_selected)
	polygon_dropdown.item_selected.connect(_on_polygon_selected)
	start_button.pressed.connect(_on_start_pressed)
	quit_button.pressed.connect(_on_quit_pressed)


func _on_drone_selected(index: int) -> void:
	GameState.set_drone(GameState.drones[index])
	print("Выбран дрон: ", GameState.selected_drone)


func _on_weather_selected(index: int) -> void:
	GameState.set_weather(GameState.weathers[index])
	print("Выбрана погода: ", GameState.selected_weather)


func _on_polygon_selected(index: int) -> void:
	GameState.set_polygon(GameState.polygons[index])
	print("Выбран полигон: ", GameState.selected_polygon)


func _on_start_pressed() -> void:
	print("Запуск: дрон=", GameState.selected_drone,
		  ", погода=", GameState.selected_weather)
	get_tree().change_scene_to_file("res://scenes/world.tscn")


func _on_quit_pressed() -> void:
	get_tree().quit()
