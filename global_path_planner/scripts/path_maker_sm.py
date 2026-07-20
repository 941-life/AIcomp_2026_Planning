#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sqrt, pow
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
from scipy.interpolate import InterpolatedUnivariateSpline, splprep, splev


class make_path:

    def __init__(self):
        filename = 'path.txt'
        self.closed_loop = False
        self.closed_loop_radius = 1.0
        self.sample_distance = 0.1
        self.smoothing_factor = 0.1
        self.logging_enabled = True

        rospy.init_node('path_maker', anonymous=False)
        filename = rospy.get_param("~file_name", filename)
        self.closed_loop = rospy.get_param("~closed_loop", self.closed_loop)
        self.closed_loop_radius = float(rospy.get_param("~closed_loop_radius", self.closed_loop_radius))
        self.sample_distance = float(rospy.get_param("~sample_distance", self.sample_distance))
        self.smoothing_factor = float(rospy.get_param("~smoothing_factor", self.smoothing_factor))
        self.odom_topic = rospy.get_param("~odom_topic", "gps_utm_odom")
        self.path_topic = rospy.get_param("~path_topic", "/local_path")
        self.frame_id = rospy.get_param("~frame_id", "map")
        if not filename.endswith(".txt"):
            filename = filename + ".txt"
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        self.is_odom = False
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.prev_x = 0
        self.prev_y = 0
        self.positions = []
        self.line_width = 0.5
        self.marker_size = 2
        self.file_closed = False
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('global_path_planner')
        directory = pkg_path + "/path_data/"
        if not os.path.exists(directory):
            os.makedirs(directory)
        self.plot_directory = os.path.join(directory, "plot")
        if not os.path.exists(self.plot_directory):
            os.makedirs(self.plot_directory)
        if os.path.exists(os.path.join(directory, filename)):
            i = 1
            while os.path.exists(os.path.join(directory, f"{os.path.splitext(filename)[0]}_{i}.txt")):
                i += 1
            filename = f"{os.path.splitext(filename)[0]}_{i}.txt"
        self.full_path = os.path.join(directory, filename)
        self.f = open(self.full_path, 'w')
        rospy.loginfo("[path_maker] 저장 파일: %s | 로깅 상태: %s",
                      self.full_path, str(self.logging_enabled).lower())
        rospy.loginfo("[path_maker] closed_loop: %s", str(self.closed_loop).lower())
        rospy.on_shutdown(self.plot_waypoints)
        rospy.spin()
        self.close_file()

    def odom_callback(self, msg):
        if self.file_closed or not self.logging_enabled:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.positions.append((x, y))

        if self.is_odom:
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
            if distance > 0.1:
                waypint_pose = PoseStamped()
                waypint_pose.pose.position.x = x
                waypint_pose.pose.position.y = y
                waypint_pose.pose.orientation.w = 1
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data = '{0}\t{1}\n'.format(x, y)
                self.f.write(data)
                self.prev_x = x
                self.prev_y = y
                print(x, y)
        else:
            self.is_odom = True
            self.prev_x = x
            self.prev_y = y

    def close_file(self):
        if not self.file_closed:
            self.f.close()
            self.file_closed = True
            self.logging_enabled = False
            rospy.loginfo("[path_maker] 저장 파일: %s | 로깅 상태: %s",
                          self.full_path, str(self.logging_enabled).lower())

    def find_closed_loop_end_index(self, x_coords, y_coords):
        if len(x_coords) < 4:
            return None

        dx = np.asarray(x_coords) - x_coords[0]
        dy = np.asarray(y_coords) - y_coords[0]
        distances = np.sqrt(dx * dx + dy * dy)
        outside_indices = np.where(distances > self.closed_loop_radius)[0]
        if outside_indices.size == 0:
            return None

        search_start_index = int(outside_indices[0])
        close_indices = np.where(
            (np.arange(len(distances)) > search_start_index) &
            (distances <= self.closed_loop_radius)
        )[0]
        if close_indices.size == 0:
            return None

        return int(close_indices[-1])

    def remove_duplicate_points(self, x_coords, y_coords):
        if len(x_coords) < 2:
            return x_coords, y_coords

        filtered_x = [x_coords[0]]
        filtered_y = [y_coords[0]]
        for x, y in zip(x_coords[1:], y_coords[1:]):
            dx = x - filtered_x[-1]
            dy = y - filtered_y[-1]
            if dx * dx + dy * dy > 1e-10:
                filtered_x.append(x)
                filtered_y.append(y)
        return filtered_x, filtered_y

    def smooth_open_path(self, x_coords, y_coords):
        distances = np.sqrt(np.diff(x_coords) ** 2 + np.diff(y_coords) ** 2)
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
        total_distance = cumulative_distances[-1]
        if total_distance <= 0.0:
            return np.asarray(x_coords), np.asarray(y_coords)

        spl_x = InterpolatedUnivariateSpline(cumulative_distances, x_coords)
        spl_y = InterpolatedUnivariateSpline(cumulative_distances, y_coords)
        spl_x.set_smoothing_factor(self.smoothing_factor)
        spl_y.set_smoothing_factor(self.smoothing_factor)

        new_distances = np.arange(0, total_distance, self.sample_distance)
        return spl_x(new_distances), spl_y(new_distances)

    def smooth_closed_loop_path(self, x_coords, y_coords):
        loop_x = list(x_coords)
        loop_y = list(y_coords)
        closing_distance = sqrt(pow(loop_x[0] - loop_x[-1], 2) + pow(loop_y[0] - loop_y[-1], 2))
        if closing_distance > 1e-6:
            loop_x.append(loop_x[0])
            loop_y.append(loop_y[0])

        distances = np.sqrt(np.diff(loop_x) ** 2 + np.diff(loop_y) ** 2)
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
        total_distance = cumulative_distances[-1]
        if total_distance <= 0.0:
            return np.asarray(x_coords), np.asarray(y_coords)

        u = cumulative_distances / total_distance
        tck, _ = splprep([loop_x, loop_y], u=u, s=self.smoothing_factor, per=True)
        new_distances = np.arange(0, total_distance, self.sample_distance)
        smoothed_x, smoothed_y = splev(new_distances / total_distance, tck)
        return np.asarray(smoothed_x), np.asarray(smoothed_y)

    def plot_waypoints(self):
        self.close_file()

        x_coords = []
        y_coords = []

        with open(self.full_path, 'r') as file:
            for line in file:
                x, y = map(float, line.strip().split())
                x_coords.append(x)
                y_coords.append(y)

        if len(x_coords) < 2:
            rospy.logwarn("[path_maker] 저장된 점이 부족해서 smoothing과 plot을 건너뜁니다.")
            return

        smoothing_x = x_coords
        smoothing_y = y_coords
        closed_loop_end_index = None
        if self.closed_loop:
            closed_loop_end_index = self.find_closed_loop_end_index(x_coords, y_coords)
            if closed_loop_end_index is None:
                rospy.logwarn("[path_maker] closed_loop 끝점을 찾지 못해 열린 경로로 smoothing합니다.")
            else:
                smoothing_x = x_coords[:closed_loop_end_index + 1]
                smoothing_y = y_coords[:closed_loop_end_index + 1]
                rospy.loginfo("[path_maker] closed_loop 끝점 인덱스: %d | 시작점 반경: %.2fm",
                              closed_loop_end_index, self.closed_loop_radius)

        smoothing_x, smoothing_y = self.remove_duplicate_points(smoothing_x, smoothing_y)
        if len(smoothing_x) < 4:
            rospy.logwarn("[path_maker] smoothing에 필요한 점이 부족해서 원본 경로를 저장합니다.")
            smoothed_x = np.asarray(smoothing_x)
            smoothed_y = np.asarray(smoothing_y)
        elif self.closed_loop and closed_loop_end_index is not None:
            smoothed_x, smoothed_y = self.smooth_closed_loop_path(smoothing_x, smoothing_y)
        else:
            smoothed_x, smoothed_y = self.smooth_open_path(smoothing_x, smoothing_y)

        plt.figure()
        plt.plot(x_coords, y_coords, '-k', lw=self.line_width, label='raw')
        plt.plot(x_coords, y_coords, '.k', markersize=self.marker_size, label='raw')
        plt.plot(smoothed_x, smoothed_y, '-b', lw=self.line_width, label='smoothed')
        plt.plot(smoothed_x, smoothed_y, '.b', markersize=self.marker_size, label='smoothed')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Waypoints Path')
        plt.xlabel('X Coordinates')
        plt.ylabel('Y Coordinates')

        sampled_path = self.full_path.replace('.txt', '_sm.txt')
        with open(sampled_path, 'w') as f:
            for x, y in zip(smoothed_x, smoothed_y):
                f.write(f'{x}\t{y}\n')

        plot_name = os.path.splitext(os.path.basename(self.full_path))[0] + "_plot.png"
        plot_path = os.path.join(self.plot_directory, plot_name)
        plt.savefig(plot_path, dpi=200, bbox_inches='tight')
        plt.close()
        rospy.loginfo("[path_maker] plot 저장 파일: %s", plot_path)
       


if __name__ == '__main__':
    try:
        test_track = make_path()
    except rospy.ROSInterruptException:
        pass
