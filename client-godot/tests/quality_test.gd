# Headless unit test for the quality-tier selection rule.
#
# Worth having as a real test because the interesting cases are the ones the
# developer's monitor cannot produce: you cannot verify the 240 Hz -> low path
# on a 60 Hz laptop, and plugging a specific panel in is not a test strategy.
# Pure function, no rendering, runs in well under a second.
#
#   godot --headless --path client-godot --script res://tests/quality_test.gd
extends SceneTree

const Q = preload("res://scripts/quality.gd")


func _initialize() -> void:
	var fails := 0

	var FHD := 1920.0 * 1080.0
	var RETINA := 2940.0 * 1846.0
	var QHD := 2560.0 * 1600.0
	var UHD := 3840.0 * 2160.0

	# [refresh, pixels, headless, PROPWASH_QUALITY, expected tier]
	var cases := [
		[60.0, FHD, false, "", "high"],       # 124 MP/s
		[60.0, RETINA, false, "", "medium"],  # 326 MP/s - measured 51fps on high
		[144.0, FHD, false, "", "medium"],    # 299 MP/s
		[60.0, UHD, false, "", "medium"],     # 498 MP/s
		[240.0, QHD, false, "", "low"],       # 983 MP/s
		[165.0, RETINA, false, "", "low"],
		[-1.0, FHD, false, "", "medium"],     # driver reports unknown
		[60.0, 0.0, false, "", "medium"],     # size unknown
		[240.0, QHD, true, "", "low"],        # headless floor beats budget
		[60.0, FHD, true, "high", "low"],     # headless floor beats explicit ask
		[240.0, QHD, false, "high", "high"],  # explicit ask beats budget
		[240.0, QHD, false, "junk", "low"],   # unknown value falls back to auto
	]
	for c in cases:
		var got: String = Q.resolve(c[3], c[0], c[1], c[2])
		if got != c[4]:
			fails += 1
			print("FAIL refresh=%s px=%s headless=%s want='%s' -> %s (expected %s)"
					% [c[0], c[1], c[2], c[3], got, c[4]])

	# Every tier must define every key, or _apply_quality throws at runtime on
	# whichever machine happens to select the incomplete tier.
	var keys: Array = Q.TIERS["high"].keys()
	for tier in Q.TIERS:
		for k in keys:
			if not Q.TIERS[tier].has(k):
				fails += 1
				print("FAIL tier '%s' missing key '%s'" % [tier, k])

	# Cost must be monotonic low -> medium -> high, or "low" is not actually the
	# cheap one and the whole auto-selection rule is backwards.
	var order := ["low", "medium", "high"]
	for i in range(order.size() - 1):
		var a: Dictionary = Q.TIERS[order[i]]
		var b: Dictionary = Q.TIERS[order[i + 1]]
		if a.scale_3d > b.scale_3d or a.trees > b.trees \
				or a.shadow_max_dist > b.shadow_max_dist or a.msaa > b.msaa:
			fails += 1
			print("FAIL tier '%s' is not cheaper than '%s'" % [order[i], order[i + 1]])

	print("[quality_test] %s (%d failure(s))" % ["PASS" if fails == 0 else "FAIL", fails])
	quit(1 if fails > 0 else 0)
