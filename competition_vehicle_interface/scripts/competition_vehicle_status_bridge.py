#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import ctypes
import math
import socket
import threading
import time
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float64, Header, Int8


class PackedStruct(ctypes.LittleEndianStructure):
    _pack_ = 1


class UdpEgoVehicleStatus24(PackedStruct):
    _fields_ = [
        ("header", ctypes.c_char * 11),
        ("data_lenght", ctypes.c_int32),
        ("aux_data", ctypes.c_int32 * 3),
        ("sec", ctypes.c_int32),
        ("nsec", ctypes.c_int32),
        ("ctrl_mode", ctypes.c_int8),
        ("gear", ctypes.c_int8),
        ("signed_vel", ctypes.c_float),
        ("map_data_id", ctypes.c_int32),
        ("accel", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("size_x", ctypes.c_float),
        ("size_y", ctypes.c_float),
        ("size_z", ctypes.c_float),
        ("overhang", ctypes.c_float),
        ("wheelbase", ctypes.c_float),
        ("rear_overhang", ctypes.c_float),
        ("pos_x", ctypes.c_float),
        ("pos_y", ctypes.c_float),
        ("pos_z", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("vel_x", ctypes.c_float),
        ("vel_y", ctypes.c_float),
        ("vel_z", ctypes.c_float),
        ("ang_vel_x", ctypes.c_float),
        ("ang_vel_y", ctypes.c_float),
        ("ang_vel_z", ctypes.c_float),
        ("accel_x", ctypes.c_float),
        ("accel_y", ctypes.c_float),
        ("accel_z", ctypes.c_float),
        ("steer", ctypes.c_float),
        ("link_id", ctypes.c_char * 38),
        ("tire_lateral_force_fl", ctypes.c_float),
        ("tire_lateral_force_fr", ctypes.c_float),
        ("tire_lateral_force_rl", ctypes.c_float),
        ("tire_lateral_force_rr", ctypes.c_float),
        ("side_slip_angle_fl", ctypes.c_float),
        ("side_slip_angle_fr", ctypes.c_float),
        ("side_slip_angle_rl", ctypes.c_float),
        ("side_slip_angle_rr", ctypes.c_float),
        ("tire_cornering_stiffness_fl", ctypes.c_float),
        ("tire_cornering_stiffness_fr", ctypes.c_float),
        ("tire_cornering_stiffness_rl", ctypes.c_float),
        ("tire_cornering_stiffness_rr", ctypes.c_float),
        ("tail", ctypes.c_char * 2),
    ]


class UdpEgoVehicleStatus26(PackedStruct):
    _fields_ = [
        ("header", ctypes.c_char * 11),
        ("data_lenght", ctypes.c_int32),
        ("aux_data", ctypes.c_int32 * 3),
        ("sec", ctypes.c_int32),
        ("nsec", ctypes.c_int32),
        ("ctrl_mode", ctypes.c_int8),
        ("gear", ctypes.c_int8),
        ("signed_vel", ctypes.c_float),
        ("map_data_id", ctypes.c_int32),
        ("accel", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("size_x", ctypes.c_float),
        ("size_y", ctypes.c_float),
        ("size_z", ctypes.c_float),
        ("overhang", ctypes.c_float),
        ("wheelbase", ctypes.c_float),
        ("rear_overhang", ctypes.c_float),
        ("pos_x", ctypes.c_float),
        ("pos_y", ctypes.c_float),
        ("pos_z", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("yaw", ctypes.c_float),
        ("vel_x", ctypes.c_float),
        ("vel_y", ctypes.c_float),
        ("vel_z", ctypes.c_float),
        ("ang_vel_x", ctypes.c_float),
        ("ang_vel_y", ctypes.c_float),
        ("ang_vel_z", ctypes.c_float),
        ("accel_x", ctypes.c_float),
        ("accel_y", ctypes.c_float),
        ("accel_z", ctypes.c_float),
        ("front_steer", ctypes.c_float),
        ("rear_steer", ctypes.c_float),
        ("link_id", ctypes.c_char * 38),
        ("tire_lateral_force_fl", ctypes.c_float),
        ("tire_lateral_force_fr", ctypes.c_float),
        ("tire_lateral_force_rl", ctypes.c_float),
        ("tire_lateral_force_rr", ctypes.c_float),
        ("side_slip_angle_fl", ctypes.c_float),
        ("side_slip_angle_fr", ctypes.c_float),
        ("side_slip_angle_rl", ctypes.c_float),
        ("side_slip_angle_rr", ctypes.c_float),
        ("tire_cornering_stiffness_fl", ctypes.c_float),
        ("tire_cornering_stiffness_fr", ctypes.c_float),
        ("tire_cornering_stiffness_rl", ctypes.c_float),
        ("tire_cornering_stiffness_rr", ctypes.c_float),
        ("distance_left_lane_boundary", ctypes.c_float),
        ("distance_right_lane_boundary", ctypes.c_float),
        ("cross_track_error", ctypes.c_float),
        ("tail", ctypes.c_char * 2),
    ]


@dataclass
class ParsedVehicleStatus:
    stamp_sec: int
    stamp_nsec: int
    control_mode: int
    gear: int
    signed_speed_kph: float
    position_x: float
    position_y: float
    position_z: float
    heading_deg: float
    velocity_x_kph: float
    velocity_y_kph: float
    velocity_z_kph: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float
    front_steer_deg: float
    rear_steer_deg: float
    accel: float
    brake: float


def _packet_stamp(parsed):
    if parsed.stamp_sec > 0 and 0 <= parsed.stamp_nsec < 1000000000:
        return rospy.Time(parsed.stamp_sec, parsed.stamp_nsec)
    return rospy.Time.now()


def _bool_param(param_name, default):
    value = rospy.get_param(param_name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _normalize_angle_rad(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _valid_packet(raw, expected_size, stream_name, strict_markers):
    if len(raw) != expected_size:
        rospy.logwarn_throttle(
            5.0,
            "%s packet size mismatch: got %d bytes, expected %d bytes",
            stream_name,
            len(raw),
            expected_size,
        )
        return False

    if not strict_markers:
        return True

    if raw[0] != 0x23 or raw[-2:] != b"\r\n" or b"$" not in raw[:32]:
        rospy.logwarn_throttle(5.0, "%s packet marker check failed", stream_name)
        return False
    return True


def _check_data_length(packet, raw_len, header_len, stream_name, strict_data_length):
    declared = int(packet.data_lenght)
    actual = raw_len - header_len - ctypes.sizeof(ctypes.c_int32) - ctypes.sizeof(ctypes.c_int32) * 3 - 2
    if declared == actual:
        return True

    rospy.logwarn_throttle(
        5.0,
        "%s data_lenght mismatch: packet says %d bytes, actual payload is %d bytes",
        stream_name,
        declared,
        actual,
    )
    return not strict_data_length


class UdpReceiver:
    def __init__(self, bind_ip, bind_port, callback):
        self.bind_ip = bind_ip
        self.bind_port = bind_port
        self.callback = callback
        self.closed = False

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)
        self.socket.settimeout(0.2)
        self.socket.bind((self.bind_ip, self.bind_port))

        self.thread = threading.Thread(target=self._run, name="competition_vehicle_status_udp", daemon=True)

    def start(self):
        self.thread.start()
        rospy.loginfo("[competition_vehicle_status_bridge] UDP receiver bound to %s:%d", self.bind_ip, self.bind_port)

    def close(self):
        self.closed = True
        try:
            self.socket.close()
        except OSError:
            pass

    def _run(self):
        while not rospy.is_shutdown() and not self.closed:
            try:
                raw, address = self.socket.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self.closed:
                    rospy.logwarn("[competition_vehicle_status_bridge] UDP receiver socket closed")
                return

            try:
                self.callback(raw, address)
            except Exception as exc:
                rospy.logerr_throttle(5.0, "[competition_vehicle_status_bridge] UDP packet handling failed: %s", exc)


class CompetitionVehicleStatusBridge:
    def __init__(self):
        rospy.init_node("competition_vehicle_status_bridge", anonymous=False)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        self.bind_port = int(rospy.get_param("~bind_port", 9012))
        self.expected_source_port = int(rospy.get_param("~expected_source_port", 9011))
        self.validate_source_port = _bool_param("~validate_source_port", False)
        self.strict_packet_markers = _bool_param("~strict_packet_markers", True)
        self.strict_data_length = _bool_param("~strict_data_length", False)
        self.status_timeout = float(rospy.get_param("~status_timeout", 0.5))
        self.queue_size = int(rospy.get_param("~queue_size", 1))

        self.last_rx_monotonic = None
        self.status_valid = None
        self.closed = False

        self._create_publishers()
        self.receiver = UdpReceiver(self.bind_ip, self.bind_port, self.handle_udp_packet)
        self.timeout_thread = threading.Thread(target=self._timeout_loop, name="competition_vehicle_status_timeout", daemon=True)

        rospy.on_shutdown(self.close)

    def _create_publishers(self):
        self.heading_pub = rospy.Publisher(rospy.get_param("~heading_topic", "/heading"), Float64, queue_size=self.queue_size)
        self.current_speed_pub = rospy.Publisher(
            rospy.get_param("~current_speed_topic", "/current_speed"),
            Float64,
            queue_size=self.queue_size,
        )
        self.signed_speed_pub = rospy.Publisher(
            rospy.get_param("~signed_speed_topic", "/vehicle/signed_speed"),
            Float64,
            queue_size=self.queue_size,
        )
        self.velocity_pub = rospy.Publisher(
            rospy.get_param("~velocity_topic", "/vehicle/velocity"),
            Vector3Stamped,
            queue_size=self.queue_size,
        )
        self.angular_velocity_pub = rospy.Publisher(
            rospy.get_param("~angular_velocity_topic", "/vehicle/angular_velocity"),
            Vector3Stamped,
            queue_size=self.queue_size,
        )
        self.acceleration_pub = rospy.Publisher(
            rospy.get_param("~acceleration_topic", "/vehicle/acceleration"),
            Vector3Stamped,
            queue_size=self.queue_size,
        )
        self.front_steer_pub = rospy.Publisher(
            rospy.get_param("~front_steer_topic", "/vehicle/front_steer_angle"),
            Float64,
            queue_size=self.queue_size,
        )
        self.rear_steer_pub = rospy.Publisher(
            rospy.get_param("~rear_steer_topic", "/vehicle/rear_steer_angle"),
            Float64,
            queue_size=self.queue_size,
        )
        self.accel_pub = rospy.Publisher(rospy.get_param("~accel_topic", "/vehicle/accel"), Float64, queue_size=self.queue_size)
        self.brake_pub = rospy.Publisher(rospy.get_param("~brake_topic", "/vehicle/brake"), Float64, queue_size=self.queue_size)
        self.gear_pub = rospy.Publisher(rospy.get_param("~gear_topic", "/vehicle/gear"), Int8, queue_size=self.queue_size)
        self.control_mode_pub = rospy.Publisher(
            rospy.get_param("~control_mode_topic", "/vehicle/control_mode"),
            Int8,
            queue_size=self.queue_size,
        )
        self.valid_pub = rospy.Publisher(
            rospy.get_param("~status_valid_topic", "/vehicle/status_valid"),
            Bool,
            queue_size=self.queue_size,
            latch=True,
        )
        self.publish_status_valid(False)

    def start(self):
        self.receiver.start()
        self.timeout_thread.start()
        rospy.loginfo(
            "[competition_vehicle_status_bridge] MORAI UDP %s:%d -> /heading, /current_speed, /vehicle/*",
            self.bind_ip,
            self.bind_port,
        )
        rospy.spin()

    def close(self):
        self.closed = True
        self.receiver.close()

    def handle_udp_packet(self, raw, address):
        if self.validate_source_port and int(address[1]) != self.expected_source_port:
            rospy.logwarn_throttle(
                5.0,
                "[competition_vehicle_status_bridge] ignored packet from %s:%d, expected source port %d",
                address[0],
                address[1],
                self.expected_source_port,
            )
            return

        try:
            parsed = self.parse_packet(raw)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "[competition_vehicle_status_bridge] %s", exc)
            return

        self.publish_control_topics(parsed, self.make_header(parsed))
        self.last_rx_monotonic = time.monotonic()
        self.publish_status_valid(True)

    def parse_packet(self, raw):
        size_24 = ctypes.sizeof(UdpEgoVehicleStatus24)
        size_26 = ctypes.sizeof(UdpEgoVehicleStatus26)

        if len(raw) == size_24:
            if not _valid_packet(raw, size_24, "EgoVehicleStatus24", self.strict_packet_markers):
                raise ValueError("invalid EgoVehicleStatus24 packet")
            packet = UdpEgoVehicleStatus24.from_buffer_copy(raw)
            if not _check_data_length(packet, len(raw), 11, "EgoVehicleStatus24", self.strict_data_length):
                raise ValueError("invalid EgoVehicleStatus24 data_lenght")
            front_steer = float(packet.steer)
            rear_steer = 0.0
        elif len(raw) == size_26:
            if not _valid_packet(raw, size_26, "EgoVehicleStatus26", self.strict_packet_markers):
                raise ValueError("invalid EgoVehicleStatus26 packet")
            packet = UdpEgoVehicleStatus26.from_buffer_copy(raw)
            if not _check_data_length(packet, len(raw), 11, "EgoVehicleStatus26", self.strict_data_length):
                raise ValueError("invalid EgoVehicleStatus26 data_lenght")
            front_steer = float(packet.front_steer)
            rear_steer = float(packet.rear_steer)
        else:
            raise ValueError("unexpected EgoVehicleStatus packet size: {}".format(len(raw)))

        return ParsedVehicleStatus(
            stamp_sec=int(packet.sec),
            stamp_nsec=int(packet.nsec),
            control_mode=int(packet.ctrl_mode),
            gear=int(packet.gear),
            signed_speed_kph=float(packet.signed_vel),
            position_x=float(packet.pos_x),
            position_y=float(packet.pos_y),
            position_z=float(packet.pos_z),
            heading_deg=float(packet.yaw),
            velocity_x_kph=float(packet.vel_x),
            velocity_y_kph=float(packet.vel_y),
            velocity_z_kph=float(packet.vel_z),
            angular_velocity_x=float(packet.ang_vel_x),
            angular_velocity_y=float(packet.ang_vel_y),
            angular_velocity_z=float(packet.ang_vel_z),
            acceleration_x=float(packet.accel_x),
            acceleration_y=float(packet.accel_y),
            acceleration_z=float(packet.accel_z),
            front_steer_deg=front_steer,
            rear_steer_deg=rear_steer,
            accel=float(packet.accel),
            brake=float(packet.brake),
        )

    def make_header(self, parsed):
        header = Header()
        header.stamp = _packet_stamp(parsed)
        header.frame_id = self.frame_id
        return header

    def publish_control_topics(self, parsed, header):
        heading_rad = _normalize_angle_rad(math.radians(parsed.heading_deg))
        signed_speed_mps = parsed.signed_speed_kph / 3.6

        self.heading_pub.publish(Float64(data=heading_rad))
        self.current_speed_pub.publish(Float64(data=abs(signed_speed_mps)))
        self.signed_speed_pub.publish(Float64(data=signed_speed_mps))

        velocity = Vector3Stamped()
        velocity.header = header
        velocity.vector.x = parsed.velocity_x_kph / 3.6
        velocity.vector.y = parsed.velocity_y_kph / 3.6
        velocity.vector.z = parsed.velocity_z_kph / 3.6
        self.velocity_pub.publish(velocity)

        angular_velocity = Vector3Stamped()
        angular_velocity.header = header
        angular_velocity.vector.x = parsed.angular_velocity_x
        angular_velocity.vector.y = parsed.angular_velocity_y
        angular_velocity.vector.z = parsed.angular_velocity_z
        self.angular_velocity_pub.publish(angular_velocity)

        acceleration = Vector3Stamped()
        acceleration.header = header
        acceleration.vector.x = parsed.acceleration_x
        acceleration.vector.y = parsed.acceleration_y
        acceleration.vector.z = parsed.acceleration_z
        self.acceleration_pub.publish(acceleration)

        self.front_steer_pub.publish(Float64(data=math.radians(parsed.front_steer_deg)))
        self.rear_steer_pub.publish(Float64(data=math.radians(parsed.rear_steer_deg)))
        self.accel_pub.publish(Float64(data=parsed.accel))
        self.brake_pub.publish(Float64(data=parsed.brake))
        self.gear_pub.publish(Int8(data=parsed.gear))
        self.control_mode_pub.publish(Int8(data=parsed.control_mode))

    def publish_status_valid(self, valid):
        if self.status_valid == valid:
            return
        self.status_valid = valid
        self.valid_pub.publish(Bool(data=valid))

    def _timeout_loop(self):
        while not rospy.is_shutdown() and not self.closed:
            if self.last_rx_monotonic is None:
                self.publish_status_valid(False)
            elif time.monotonic() - self.last_rx_monotonic > self.status_timeout:
                rospy.logwarn_throttle(1.0, "[competition_vehicle_status_bridge] vehicle status UDP timeout")
                self.publish_status_valid(False)
            time.sleep(0.1)


if __name__ == "__main__":
    try:
        CompetitionVehicleStatusBridge().start()
    except rospy.ROSInterruptException:
        pass
