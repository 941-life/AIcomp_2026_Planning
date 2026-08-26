#!/usr/bin/env python3

import math

import rospy
from gadis_perception_msgs.msg import Object, ObjectArray
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64, String


def clamp(value, low, high):
    return max(low, min(value, high))


class SyntheticHighwayScene:
    def __init__(self):
        rospy.init_node("synthetic_highway_scene")
        self.scenario = rospy.get_param("~scenario", "empty")
        if self.scenario not in ("empty", "rear_blocked", "front_follow"):
            raise ValueError("scenario must be empty, rear_blocked, or front_follow")

        self.x = 0.0
        self.y = -3.5
        self.speed_mps = 70.0 / 3.6
        self.target_speed_mps = self.speed_mps
        self.stage = "ENTRY"
        self.fsm = "KEEP_LANE"
        self.local_path = None
        self.last_time = rospy.Time.now()
        self.rear_vehicle_x = -20.0
        self.front_vehicle_x = 45.0

        self.odom_pub = rospy.Publisher(
            "/odometry/filtered", Odometry, queue_size=1
        )
        self.speed_pub = rospy.Publisher("/current_speed", Float64, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/vehicle/status_valid", Bool, queue_size=1
        )
        self.objects_pub = rospy.Publisher("/objects", ObjectArray, queue_size=1)
        self.region_pub = rospy.Publisher("/highway/region", String, queue_size=1)
        self.guard_pub = rospy.Publisher(
            "/highway/distance_to_guard_stop", Float64, queue_size=1
        )
        self.limit_pub = rospy.Publisher(
            "/highway/distance_to_next_limit", Float64, queue_size=1
        )
        self.lane_pubs = {
            1: rospy.Publisher(
                "/highway/lane1_centerline", Path, queue_size=1, latch=True
            ),
            2: rospy.Publisher(
                "/highway/lane2_centerline", Path, queue_size=1, latch=True
            ),
            3: rospy.Publisher(
                "/highway/lane3_centerline", Path, queue_size=1, latch=True
            ),
            4: rospy.Publisher(
                "/highway/lane4_centerline", Path, queue_size=1, latch=True
            ),
        }

        rospy.Subscriber(
            "/highway/selected_path_with_speed",
            Path,
            self.path_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/highway/target_speed_mps",
            Float64,
            self.target_speed_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/highway/lane_change_state", String, self.fsm_callback, queue_size=1
        )

        self.publish_lanes()
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)
        rospy.loginfo("[synthetic_highway_scene] scenario=%s", self.scenario)

    def publish_lanes(self):
        lane_y = {1: 7.0, 2: 3.5, 3: 0.0, 4: -3.5}
        stamp = rospy.Time.now()
        for lane_id, y_value in lane_y.items():
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = "map"
            for x_value in range(0, 3001, 2):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(x_value)
                pose.pose.position.y = y_value
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.lane_pubs[lane_id].publish(path)

    def path_callback(self, message):
        self.local_path = message

    def target_speed_callback(self, message):
        if math.isfinite(message.data) and message.data >= 0.0:
            self.target_speed_mps = message.data

    def fsm_callback(self, message):
        self.fsm = message.data

    def target_local_y(self):
        if self.local_path is None or not self.local_path.poses:
            return 0.0
        for pose in self.local_path.poses:
            if pose.pose.position.x >= 20.0:
                return pose.pose.position.y
        return self.local_path.poses[-1].pose.position.y

    def update_vehicle(self, dt):
        speed_error = self.target_speed_mps - self.speed_mps
        self.speed_mps += clamp(speed_error, -2.0 * dt, 1.0 * dt)
        lateral_velocity = clamp(0.6 * self.target_local_y(), -0.8, 0.8)
        self.y += lateral_velocity * dt
        self.x += self.speed_mps * dt

        if (
            self.stage == "ENTRY"
            and abs(self.y) < 0.20
            and self.fsm == "KEEP_LANE"
        ):
            self.stage = "MAIN"
            rospy.loginfo("[synthetic_highway_scene] entry 4->3 complete")
        elif (
            self.stage == "MAIN"
            and abs(self.y - 7.0) < 0.20
            and self.fsm == "KEEP_LANE"
        ):
            self.stage = "SINGLE"
            rospy.loginfo("[synthetic_highway_scene] main 3->2->1 complete")

        self.rear_vehicle_x += 30.0 * dt
        self.front_vehicle_x += 12.0 * dt

    def make_object(self, object_id, world_x, world_y):
        rel_x_base = world_x - self.x
        rel_y_base = world_y - self.y
        center_x_lidar = rel_x_base - 1.144
        center_y_lidar = rel_y_base
        half_length = 2.3
        half_width = 0.95

        message = Object()
        message.id = object_id
        message.class_name = "car"
        message.measurement_time = rospy.Time.now()
        message.is_predicted = False
        message.is_lidar_only = False
        points = []
        for z_value in (-1.0, 0.6):
            for x_value, y_value in (
                (center_x_lidar - half_length, center_y_lidar - half_width),
                (center_x_lidar - half_length, center_y_lidar + half_width),
                (center_x_lidar + half_length, center_y_lidar + half_width),
                (center_x_lidar + half_length, center_y_lidar - half_width),
            ):
                point = Point32()
                point.x = x_value
                point.y = y_value
                point.z = z_value
                points.append(point)
        message.bbox3d_points = points
        return message

    def publish_objects(self, stamp):
        message = ObjectArray()
        message.header.stamp = stamp
        message.header.frame_id = "velodyne"
        message.sensor_healthy = True
        message.sensor_stale_s = 0.0

        if self.scenario == "rear_blocked" and self.stage == "ENTRY":
            if abs(self.rear_vehicle_x - self.x) < 100.0:
                message.objects.append(
                    self.make_object(0, self.rear_vehicle_x, 0.0)
                )
        elif self.scenario == "front_follow" and self.stage == "ENTRY":
            if abs(self.front_vehicle_x - self.x) < 100.0:
                message.objects.append(
                    self.make_object(0, self.front_vehicle_x, -3.5)
                )
        self.objects_pub.publish(message)

    def publish_state(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.speed_mps
        self.odom_pub.publish(odom)
        self.speed_pub.publish(Float64(data=self.speed_mps))
        self.status_pub.publish(Bool(data=True))

        if self.stage == "ENTRY":
            region = "HW_ENTRY"
        elif self.stage == "MAIN":
            region = "HW_MAIN"
        else:
            region = "HW_TOLL"
        self.region_pub.publish(String(data=region))
        self.guard_pub.publish(Float64(data=150.0))
        self.limit_pub.publish(Float64(data=120.0))
        self.publish_objects(stamp)

    def timer_callback(self, _event):
        now = rospy.Time.now()
        dt = clamp((now - self.last_time).to_sec(), 0.0, 0.1)
        self.last_time = now
        self.update_vehicle(dt)
        self.publish_state(now)
        rospy.loginfo_throttle(
            1.0,
            "[synthetic] stage=%s fsm=%s x=%.1f y=%.2f speed=%.1f kph",
            self.stage,
            self.fsm,
            self.x,
            self.y,
            self.speed_mps * 3.6,
        )


if __name__ == "__main__":
    try:
        SyntheticHighwayScene()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
