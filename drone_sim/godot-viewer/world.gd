extends Node3D

const PLAYBACK_SPEED := 1.0
const LOOP := true
const PYTHON_EXE := "C:\\Users\\dslob\\AppData\\Local\\Programs\\Python\\Python312\\python.exe"
const MAIN_PY := "C:/Users/dslob/Downloads/drone_sim/main.py"
const CSV_PATH := "C:/Users/dslob/Downloads/drone_sim/output/logs/telemetry.csv"

var frames: Array = []
var frame_index: int = 0
var playback_time: float = 0.0
var trail_points: PackedVector3Array = []
var trail_mesh: ImmediateMesh
var polygon_loaded: bool = false

@onready var drone: Node3D = $Drone
@onready var camera_3rd: Camera3D = $Camera
@onready var camera_fpv: Camera3D = $Drone/CameraFPV
@onready var hud_label: Label = $HUD/ControlHUD/TelemetryLabel
@onready var trail: MeshInstance3D = $Trail


func _ready() -> void:
	if not polygon_loaded:
		_load_polygon()
		polygon_loaded = true

	_setup_trail()
	camera_3rd.current = true
	if camera_fpv:
		camera_fpv.current = false

	$HUD/Control/ButtonsPanel/RestartButton.pressed.connect(_on_restart_pressed)
	$HUD/Control/ButtonsPanel/MenuButton.pressed.connect(_on_menu_pressed)
	$HUD/Control/ButtonsPanel/QuitButton.pressed.connect(_on_quit_pressed)

	_run_python_simulation()
	_load_csv()


func _load_polygon() -> void:
	var poly_name: String = GameState.selected_polygon
	var poly_path: String = "res://scenes/polygon_%s.tscn" % poly_name
	print("Загрузка полигона: ", poly_path)

	if not ResourceLoader.exists(poly_path):
		push_error("Полигон не найден: " + poly_path)
		return

	var scene: PackedScene = load(poly_path) as PackedScene
	if scene == null:
		push_error("Не удалось загрузить полигон")
		return

	var instance: Node = scene.instantiate()
	if instance == null:
		push_error("Не удалось создать экземпляр полигона")
		return

	add_child(instance)
	print("Полигон загружен: ", poly_name)


func _run_python_simulation() -> void:
	var drone_name := GameState.selected_drone
	var weather_name := GameState.selected_weather
	print("Запуск Python: ", PYTHON_EXE, " ", MAIN_PY)
	OS.execute(PYTHON_EXE, [MAIN_PY, drone_name, weather_name], [], true)


func _setup_trail() -> void:
	trail_mesh = ImmediateMesh.new()
	trail.mesh = trail_mesh
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(1.0, 0.6, 0.0, 0.8)
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	trail.material_override = mat


func _load_csv() -> void:
	frames.clear()
	frame_index = 0
	playback_time = 0.0
	trail_points.clear()

	var file := FileAccess.open(CSV_PATH, FileAccess.READ)
	if not file:
		push_error("Не могу открыть CSV: " + CSV_PATH)
		return

	var headers := file.get_csv_line()
	var idx := {}
	for i in headers.size():
		idx[headers[i]] = i

	while not file.eof_reached():
		var row := file.get_csv_line()
		if row.size() < headers.size():
			continue
		frames.append({
			"time": float(row[idx["time"]]),
			"pos_x": float(row[idx["pos_x"]]),
			"pos_y": float(row[idx["pos_y"]]),
			"pos_z": float(row[idx["pos_z"]]),
			"roll": float(row[idx["roll_deg"]]),
			"pitch": float(row[idx["pitch_deg"]]),
			"yaw": float(row[idx["yaw_deg"]]),
			"vel_x": float(row[idx["vel_x"]]),
			"vel_y": float(row[idx["vel_y"]]),
			"vel_z": float(row[idx["vel_z"]]),
			"battery": float(row[idx["battery_pct"]]),
		})

	file.close()
	print("Загружено кадров: ", frames.size())


func _process(delta: float) -> void:
	if camera_3rd.current:
		var angle := Time.get_ticks_msec() / 3000.0
		var radius := 12.0
		var height := 6.0
		camera_3rd.position = Vector3(
			drone.position.x + cos(angle) * radius,
			drone.position.y + height,
			drone.position.z + sin(angle) * radius
		)
		camera_3rd.look_at(drone.position, Vector3.UP)

	if frames.is_empty():
		return

	playback_time += delta * PLAYBACK_SPEED

	while frame_index < frames.size() - 1 and frames[frame_index + 1]["time"] <= playback_time:
		frame_index += 1

	if frame_index >= frames.size() - 1:
		if LOOP:
			frame_index = 0
			playback_time = 0.0
			trail_points.clear()
			_redraw_trail()
		else:
			return

	_update_drone(frames[frame_index])
	_update_hud(frames[frame_index])


func _update_drone(f: Dictionary) -> void:
	drone.position = Vector3(f["pos_x"], f["pos_z"], f["pos_y"])
	drone.rotation_degrees = Vector3(f["roll"], f["yaw"], f["pitch"])

	if trail_points.size() == 0 or trail_points[-1].distance_to(drone.position) > 0.5:
		trail_points.append(drone.position)
		_redraw_trail()


func _update_hud(f: Dictionary) -> void:
	var speed := sqrt(f["vel_x"] * f["vel_x"] + f["vel_y"] * f["vel_y"] + f["vel_z"] * f["vel_z"])
	hud_label.text = (
		"=== ТЕЛЕМЕТРИЯ ===\n"
		+ "Дрон:      %s\n" % GameState.get_drone_label(GameState.selected_drone)
		+ "Погода:    %s\n" % GameState.get_weather_label(GameState.selected_weather)
		+ "Полигон:   %s\n" % GameState.get_polygon_label(GameState.selected_polygon)
		+ "Высота:    %6.2f м\n" % f["pos_z"]
		+ "Скорость:  %6.2f м/с\n" % speed
		+ "Крен:      %6.1f°\n" % f["roll"]
		+ "Тангаж:    %6.1f°\n" % f["pitch"]
		+ "Рыскание:  %6.1f°\n" % f["yaw"]
		+ "Батарея:   %6.1f%%\n" % f["battery"]
		+ "\n[C] — сменить камеру\n[R] — перезапуск"
	)


func _redraw_trail() -> void:
	trail_mesh.clear_surfaces()
	if trail_points.size() < 2:
		return
	trail_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
	for p in trail_points:
		trail_mesh.surface_add_vertex(p)
	trail_mesh.surface_end()


func _input(event: InputEvent) -> void:
	if not (event is InputEventKey and event.pressed):
		return
	if event.keycode == KEY_C:
		if camera_3rd.current:
			camera_3rd.current = false
			if camera_fpv:
				camera_fpv.current = true
		else:
			if camera_fpv:
				camera_fpv.current = false
			camera_3rd.current = true
	elif event.keycode == KEY_R:
		_restart_simulation()


func _on_restart_pressed() -> void:
	_restart_simulation()


func _on_menu_pressed() -> void:
	get_tree().change_scene_to_file("res://scenes/menu.tscn")


func _on_quit_pressed() -> void:
	get_tree().quit()


func _restart_simulation() -> void:
	_run_python_simulation()
	_load_csv()
