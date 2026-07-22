# Every shader in the project must actually compile.
#
# This exists because a broken shader is SILENT in the places we already test.
# The headless paths skip _build_goggle_layer entirely (no rasterisation, so no
# goggle pass), so goggle.gdshader was never even parsed by the 30-test suite —
# and a `return` added to its fragment() shipped green while, on a real
# renderer, it failed the whole shader, dropped the O3 feed treatment
# completely, and emitted `version_get_shader: Parameter "version" is null`
# once per frame forever.
#
# Godot's shader parser DOES run headless: assigning a Shader to a
# ShaderMaterial parses it and prints `SHADER ERROR` to stderr. The script
# cannot catch that itself, so the ctest carries
# FAIL_REGULAR_EXPRESSION "SHADER ERROR" and this script just makes sure every
# shader is loaded and assigned.
#
#   godot --headless --path client-godot --script res://tests/shader_compile_test.gd
extends SceneTree

const SHADER_DIR := "res://shaders"


func _initialize() -> void:
	var dir := DirAccess.open(SHADER_DIR)
	if dir == null:
		print("FAIL cannot open %s" % SHADER_DIR)
		quit(1)
		return

	var checked := 0
	var fails := 0
	for f in dir.get_files():
		if not f.ends_with(".gdshader"):
			continue                      # skip .uid sidecars
		var path := "%s/%s" % [SHADER_DIR, f]
		var sh: Shader = load(path) as Shader
		if sh == null:
			fails += 1
			print("FAIL %s did not load as a Shader" % path)
			continue
		if sh.code.strip_edges().is_empty():
			fails += 1
			print("FAIL %s is empty" % path)
			continue
		# the assignment is what forces the parse; any error lands on stderr
		# and the ctest's FAIL_REGULAR_EXPRESSION catches it
		var mat := ShaderMaterial.new()
		mat.shader = sh
		checked += 1
		print("[shader_test] parsed %s (%d bytes)" % [path, sh.code.length()])

	# A pass with zero shaders would mean the walk broke, not that everything
	# is fine — the client has at least the goggle and ground shaders.
	if checked < 2:
		fails += 1
		print("FAIL only %d shader(s) found under %s — did the walk break?"
				% [checked, SHADER_DIR])

	print("[shader_test] %s (%d checked, %d failure(s))"
			% ["PASS" if fails == 0 else "FAIL", checked, fails])
	quit(1 if fails > 0 else 0)
