# One-off visual probe: build the real world, force the crash banner on,
# save a screenshot, quit. Not a ctest — run by hand when styling the banner:
#   PROPWASH_CORE=/nonexistent PROPWASH_SCREEN=off PROPWASH_SHOT_OUT=/tmp/x.png \
#     godot --path client-godot --script res://tests/banner_shot.gd
extends SceneTree

const Main = preload("res://scripts/main.gd")

var _main: Node3D
var _frames := 0


func _initialize() -> void:
	_main = Node3D.new()
	_main.set_script(Main)
	root.add_child(_main)


func _process(_d: float) -> bool:
	_frames += 1
	if _frames == 30:
		_main._update_crash_banner(1, 0, true)   # crashed while still armed
		_main._toast_msg("disarm first (E / ARM switch), then T to repair")
	if _frames == 45:
		var img := root.get_viewport().get_texture().get_image()
		var out := OS.get_environment("PROPWASH_SHOT_OUT")
		if out.is_empty():
			out = "/tmp/propwash_banner.png"
		img.save_png(out)
		print("[banner_shot] saved ", out)
		return true
	return false
