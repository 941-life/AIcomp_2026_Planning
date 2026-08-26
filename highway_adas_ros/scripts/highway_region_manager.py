#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64, String


class HighwayRegionManager:
    REGIONS = (
        "HW_ENTRY",
        "HW_ENTRY_GUARD",
        "HW_MAIN",
        "HW_MAIN_GUARD_1",
        "HW_MAIN_GUARD_2",
        "HW_TOLL",
    )

    def __init__(self):
        self.path_topic = rospy.get_param(
            "~reference_path_topic", "/highway/lane1_centerline"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/filtered")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.search_window_segments = int(
            rospy.get_param("~search_window_segments", 250)
        )
        self.maximum_projection_distance_m = float(
            rospy.get_param("~maximum_projection_distance_m", 15.0)
        )

        self.entry_guard_start_s = float(
            rospy.get_param("~entry_guard_start_s")
        )
        self.main_start_s = float(rospy.get_param("~main_start_s"))
        self.main_guard_1_start_s = float(
            rospy.get_param("~main_guard_1_start_s")
        )
        self.main_guard_2_start_s = float(
            rospy.get_param("~main_guard_2_start_s")
        )
        self.toll_start_s = float(rospy.get_param("~toll_start_s"))
        self.highway_end_s = float(rospy.get_param("~highway_end_s"))
        self.entry_guard_stop_s = float(
            rospy.get_param("~entry_guard_stop_s")
        )
        self.main_guard_1_stop_s = float(
            rospy.get_param("~main_guard_1_stop_s")
        )
        self.main_guard_2_stop_s = float(
            rospy.get_param("~main_guard_2_stop_s")
        )
        self.next_limit_s = float(rospy.get_param("~next_limit_s"))
        self.validate_configuration()

        self.points = []
        self.cumulative_s = []
        self.last_segment = None
        self.last_progress_s = None

        self.region_pub = rospy.Publisher(
            "/highway/region", String, queue_size=1, latch=True
        )
        self.guard_pub = rospy.Publisher(
            "/highway/distance_to_guard_stop", Float64, queue_size=1
        )
        self.limit_pub = rospy.Publisher(
            "/highway/distance_to_next_limit", Float64, queue_size=1
        )
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

    def validate_configuration(self):
        boundaries = (
            self.entry_guard_start_s,
            self.main_start_s,
            self.main_guard_1_start_s,
            self.main_guard_2_start_s,
            self.toll_start_s,
            self.highway_end_s,
        )
        if any(a >= b for a, b in zip(boundaries, boundaries[1:])):
            raise ValueError("highway region boundaries must be strictly increasing")
        if not (
            self.entry_guard_start_s
            < self.entry_guard_stop_s
            < self.main_start_s
        ):
            raise ValueError("entry guard stop must be inside the entry guard")
        if not (
            self.main_guard_1_start_s
            < self.main_guard_1_stop_s
            < self.main_guard_2_start_s
        ):
            raise ValueError("main guard 1 stop must be inside main guard 1")
        if not (
            self.main_guard_2_start_s
            < self.main_guard_2_stop_s
            < self.toll_start_s
        ):
            raise ValueError("main guard 2 stop must be inside main guard 2")
        if not self.toll_start_s < self.next_limit_s <= self.highway_end_s:
            raise ValueError("next speed limit must be inside the toll region")

    def path_callback(self, message):
        if message.header.frame_id != self.map_frame or len(message.poses) < 2:
            rospy.logwarn_throttle(
                1.0, "[highway_region] reference path must be a map-frame path"
            )
            return
        self.points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        self.cumulative_s = [0.0]
        for first, second in zip(self.points, self.points[1:]):
            self.cumulative_s.append(
                self.cumulative_s[-1]
                + math.hypot(second[0] - first[0], second[1] - first[1])
            )
        self.last_segment = None
        self.last_progress_s = None

    @staticmethod
    def project_to_segment(point, first, second):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            return 0.0, math.hypot(point[0] - first[0], point[1] - first[1])
        ratio = ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy)
        ratio = max(0.0, min(1.0, ratio / length_squared))
        projected = (first[0] + ratio * dx, first[1] + ratio * dy)
        return ratio, math.hypot(point[0] - projected[0], point[1] - projected[1])

    def project_progress(self, position):
        segment_count = len(self.points) - 1
        if self.last_segment is None:
            start, end = 0, segment_count
        else:
            start = max(0, self.last_segment - self.search_window_segments)
            end = min(segment_count, self.last_segment + self.search_window_segments + 1)

        best = None
        for index in range(start, end):
            ratio, distance = self.project_to_segment(
                position, self.points[index], self.points[index + 1]
            )
            if best is None or distance < best[0]:
                segment_length = (
                    self.cumulative_s[index + 1] - self.cumulative_s[index]
                )
                progress = self.cumulative_s[index] + ratio * segment_length
                best = (distance, progress, index)

        if best is None or best[0] > self.maximum_projection_distance_m:
            return None
        self.last_segment = best[2]
        progress = best[1]
        if self.last_progress_s is not None:
            progress = max(progress, self.last_progress_s)
        self.last_progress_s = progress
        return progress

    def region_at(self, progress_s):
        if progress_s < self.entry_guard_start_s:
            return self.REGIONS[0]
        if progress_s < self.main_start_s:
            return self.REGIONS[1]
        if progress_s < self.main_guard_1_start_s:
            return self.REGIONS[2]
        if progress_s < self.main_guard_2_start_s:
            return self.REGIONS[3]
        if progress_s < self.toll_start_s:
            return self.REGIONS[4]
        if progress_s < self.highway_end_s:
            return self.REGIONS[5]
        return "NONE"

    def guard_stop_at(self, region):
        if region == "HW_ENTRY_GUARD":
            return self.entry_guard_stop_s
        if region == "HW_MAIN_GUARD_1":
            return self.main_guard_1_stop_s
        if region == "HW_MAIN_GUARD_2":
            return self.main_guard_2_stop_s
        return None

    def odom_callback(self, message):
        if message.header.frame_id != self.map_frame or len(self.points) < 2:
            return
        progress_s = self.project_progress(
            (message.pose.pose.position.x, message.pose.pose.position.y)
        )
        if progress_s is None:
            rospy.logwarn_throttle(
                1.0, "[highway_region] ego is outside the highway path corridor"
            )
            return

        region = self.region_at(progress_s)
        self.region_pub.publish(String(data=region))
        guard_stop_s = self.guard_stop_at(region)
        if guard_stop_s is not None:
            self.guard_pub.publish(
                Float64(data=max(0.0, guard_stop_s - progress_s))
            )
        if region == "HW_TOLL":
            self.limit_pub.publish(
                Float64(data=max(0.0, self.next_limit_s - progress_s))
            )


if __name__ == "__main__":
    try:
        rospy.init_node("highway_region_manager")
        HighwayRegionManager()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as error:
        rospy.logfatal("[highway_region] %s", error)
