# propwash wire protocol codec (MIT). Mirrors protocol/propwash_protocol.h:
# packed little-endian structs, PwHeader + payload per datagram.
class_name PwProtocol

const MAGIC := 0x48535750  # "PWSH"
const VERSION := 1

const PW_INIT := 1
const PW_INIT_ACK := 2
const PW_STATE_IN := 3
const PW_STATE_OUT := 4
const PW_OSD := 5
const PW_RC_OVERRIDE := 6
const PW_COMMAND := 7

const PW_CMD_RESET := 1


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
# vbat f32, contact u8  => 133 bytes payload
static func pack_state_in(frame_id: int, dt: float, rc: Array,
		pos: Vector3, rot: Quaternion, angvel: Vector3, linvel: Vector3,
		vbat: float, contact: bool) -> PackedByteArray:
	var b := _header(PW_STATE_IN, 133)
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
		b.put_float(0.0)  # prop damage
	for i in range(4):
		b.put_float(0.0)  # ground effect
	b.put_float(vbat)
	b.put_u8(1 if contact else 0)
	return b.data_array


static func pack_command(cmd: int) -> PackedByteArray:
	var b := _header(PW_COMMAND, 4)
	b.put_u32(cmd)
	return b.data_array


# PwStateOut (114 bytes): frame_id u32, sim_time u64, quat wxyz, angvel,
# linvel, pos, accel, motor_rpm 4f, motor_status 4u8, armed u8,
# arming u32, mode u32, beeper u8, vbat f32, amperage f32
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
	return out
