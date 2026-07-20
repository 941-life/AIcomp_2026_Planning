#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Bool, Float64


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def speed_to_mps(speed, unit):
    if unit in ("kph", "kmh", "km/h"):
        return speed / 3.6
    if unit in ("mps", "m/s"):
        return speed
    raise ValueError("unsupported speed unit: {}".format(unit))


class ControlStatusAdapter:
    def __init__(self):
        rospy.init_node("control_status_adapter", anonymous=False)

        self.ego_status_topic = rospy.get_param("~ego_status_topic", "/morai/ego_vehicle_status")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.current_speed_topic = rospy.get_param("~current_speed_topic", "/current_speed")
        self.status_valid_topic = rospy.get_param("~status_valid_topic", "/vehicle_status_valid")

        self.heading_input_unit = rospy.get_param("~heading_input_unit", "deg").lower()
        self.speed_input_unit = rospy.get_param("~speed_input_unit", "kph").lower()
        self.status_timeout = float(rospy.get_param("~status_timeout", 0.5))

        self.last_status_received_at = rospy.Time(0)
        self.status_valid = False

        self.heading_pub = rospy.Publisher(self.heading_topic, Float64, queue_size=10)
        self.current_speed_pub = rospy.Publisher(self.current_speed_topic, Float64, queue_size=10)
        self.status_valid_pub = rospy.Publisher(self.status_valid_topic, Bool, queue_size=10, latch=True)
        self.status_valid_pub.publish(Bool(data=False))

        rospy.Subscriber(self.ego_status_topic, EgoVehicleStatus, self.ego_status_callback, queue_size=1)

        rospy.loginfo(
            "[control_status_adapter] %s -> %s, %s, %s",
            self.ego_status_topic,
            self.heading_topic,
            self.current_speed_topic,
            self.status_valid_topic,
        )

    def publish_status_valid(self, valid):
        if self.status_valid == valid:
            return
        self.status_valid = valid
        self.status_valid_pub.publish(Bool(data=valid))

    def ego_status_callback(self, msg):
        yaw = float(msg.heading)
        if self.heading_input_unit in ("deg", "degree", "degrees"):
            yaw = math.radians(yaw)
        elif self.heading_input_unit not in ("rad", "radian", "radians"):
            raise ValueError("unsupported heading unit: {}".format(self.heading_input_unit))
        self.heading_pub.publish(Float64(data=normalize_angle(yaw)))

        speed = math.hypot(float(msg.velocity.x), float(msg.velocity.y))
        self.current_speed_pub.publish(Float64(data=speed_to_mps(speed, self.speed_input_unit)))

        self.last_status_received_at = rospy.Time.now()
        self.publish_status_valid(True)

    def spin(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if (
                not self.last_status_received_at.is_zero()
                and (rospy.Time.now() - self.last_status_received_at).to_sec() > self.status_timeout
            ):
                rospy.logwarn_throttle(1.0, "[control_status_adapter] EgoVehicleStatus timeout")
                self.publish_status_valid(False)
            rate.sleep()


if __name__ == "__main__":
    try:
        ControlStatusAdapter().spin()
    except rospy.ROSInterruptException:
        pass
