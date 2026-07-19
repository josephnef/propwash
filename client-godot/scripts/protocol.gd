# propwash wire protocol codec (MIT). Mirrors protocol/propwash_protocol.h:
# packed little-endian structs, PwHeader + payload per datagram.
class_name PwProtocol

const MAGIC := 0x48535750  # "PWSH"
const VERSION := 2

const PW_INIT := 1
const PW_INIT_ACK := 2
const PW_STATE_IN := 3
const PW_STATE_OUT := 4
const PW_OSD := 5
const PW_RC_OVERRIDE := 6
const PW_COMMAND := 7

const PW_CMD_RESET := 1
const PW_CMD_REPAIR := 6

const MAX_CONTACTS := 6

# PwSurfaceType
const SURF_GROUND := 0
const SURF_GATE := 1
const SURF_TREE := 2
const SURF_OBJECT := 3


static func _header(type: int, payload_len: int) -> StreamPeerBuffer:
	var b := StreamPeerBuffer.new()
	b.big_endian = false
	b.put_u32(MAGIC)
	b.put_u8(VERSION)
	b.put_u8(type)
	b.put_u16(payload_len)
	return b


# PwStateIn: frame_id u32, dt f32, rc[8] f32, pos 3f, quat wxyz 4f,
# angvel 3f, linvel 3f, gyro_noise f32, prop_damage 4f, ground_effect 4f,
# vbat f32, contact u8, contact_count u8, 6x contact (point_body 3f,
# normal_world 3f, depth f32, surface u8)  => 308 bytes payload
#
# contacts: Array of {point_body: Vector3, normal_world: Vector3,
# depth: float, surface: int}, already converted to the sim frame.
static func pack_state_in(frame_id: int, dt: float, rc: Array,
		pos: Vector3, rot: Quaternion, angvel: Vector3, linvel: Vector3,
		vbat: float, contact: bool, contacts: Array = [],
		ground_effect: Array = []) -> PackedByteArray:
	var b := _header(PW_STATE_IN, 308)
	b.put_u32(frame_id)
	b.put_float(dt)
	for i in range(8):
		b.put_float(rc[i])
	b.put_float(pos.x); b.put_float(pos.y); b.put_float(pos.z)
	b.put_float(rot.w); b.put_float(rot.x); b.put_float(rot.y); b.put_float(rot.z)
	b.put_float(angvel.x); b.put_float(angvel.y); b.put_float(angvel.z)
	b.put_float(linvel.x); b.put_float(linvel.y); b.put_float(linvel.z)
	b.put_float(0.0002)  # gyro noise amp
	for i in range(4):
		b.put_float(0.0)  # prop damage (external/scripted; unused here)
	for i in range(4):
		b.put_float(ground_effect[i] if i < ground_effect.size() else 0.0)
	b.put_float(vbat)
	b.put_u8(1 if contact else 0)
	var n: int = mini(contacts.size(), MAX_CONTACTS)
	b.put_u8(n)
	for i in range(MAX_CONTACTS):
		if i < n:
			var c: Dictionary = contacts[i]
			var p: Vector3 = c.point_body
			var nrm: Vector3 = c.normal_world
			b.put_float(p.x); b.put_float(p.y); b.put_float(p.z)
			b.put_float(nrm.x); b.put_float(nrm.y); b.put_float(nrm.z)
			b.put_float(c.depth)
			b.put_u8(c.surface)
		else:
			for j in range(7):
				b.put_float(0.0)
			b.put_u8(0)
	return b.data_array


static func pack_command(cmd: int) -> PackedByteArray:
	var b := _header(PW_COMMAND, 4)
	b.put_u32(cmd)
	return b.data_array


# PwRcOverride: rc[8] f32. Server priority: joystick > override > packet rc.
static func pack_rc_override(rc: Array) -> PackedByteArray:
	var b := _header(PW_RC_OVERRIDE, 32)
	for i in range(8):
		b.put_float(rc[i])
	return b.data_array


# PW_OSD: 16 rows x 30 cols of Betaflight OSD character codes. Returns the
# printable text as 16 lines (non-printable codes -> space).
static func unpack_osd(pkt: PackedByteArray) -> PackedStringArray:
	var b := StreamPeerBuffer.new()
	b.big_endian = false
	b.data_array = pkt
	if b.get_u32() != MAGIC or b.get_u8() != VERSION or b.get_u8() != PW_OSD:
		return PackedStringArray()
	b.get_u16()  # payload_len
	var lines := PackedStringArray()
	for y in range(16):
		var s := ""
		for x in range(30):
			var c := b.get_u8()
			s += char(c) if (c >= 32 and c < 127) else " "
		lines.append(s)
	return lines


# PwStateOut (131 bytes): frame_id u32, sim_time u64, quat wxyz, angvel,
# linvel, pos, accel, motor_rpm 4f, motor_status 4u8, armed u8,
# arming u32, mode u32, beeper u8, vbat f32, amperage f32,
# prop_damage 4f, crash_flags u8
static func unpack_state_out(pkt: PackedByteArray) -> Dictionary:
	var b := StreamPeerBuffer.new()
	b.big_endian = false
	b.data_array = pkt
	var magic := b.get_u32()
	var version := b.get_u8()
	var type := b.get_u8()
	var _len := b.get_u16()
	if magic != MAGIC or version != VERSION or type != PW_STATE_OUT:
		return {}
	var out := {}
	out.frame_id = b.get_u32()
	out.sim_time_us = b.get_u64()
	var w := b.get_float(); var x := b.get_float(); var y := b.get_float(); var z := b.get_float()
	out.rotation = Quaternion(x, y, z, w)
	out.angvel = Vector3(b.get_float(), b.get_float(), b.get_float())
	out.linvel = Vector3(b.get_float(), b.get_float(), b.get_float())
	out.position = Vector3(b.get_float(), b.get_float(), b.get_float())
	out.accel = Vector3(b.get_float(), b.get_float(), b.get_float())
	out.motor_rpm = [b.get_float(), b.get_float(), b.get_float(), b.get_float()]
	out.motor_status = [b.get_u8(), b.get_u8(), b.get_u8(), b.get_u8()]
	out.armed = b.get_u8() == 1
	out.arming_disable = b.get_u32()
	out.mode_flags = b.get_u32()
	out.beeper = b.get_u8() == 1
	out.vbat = b.get_float()
	out.amperage = b.get_float()
	out.prop_damage = [b.get_float(), b.get_float(), b.get_float(), b.get_float()]
	out.crash_flags = b.get_u8()
	return out
