#!/usr/bin/env python3

import argparse
import sys
import time

import rospy
from std_msgs.msg import Float64, Int32, String


class ScenarioVerifier:
    def __init__(self, scenario):
        self.scenario = scenario
        self.start = time.monotonic()
        self.lanes = set()
        self.states = set()
        self.modes = set()
        self.minimum_speed_kph = float("inf")
        self.execute_time = None

        rospy.Subscriber("/highway/current_lane", Int32, self.lane_callback)
        rospy.Subscriber(
            "/highway/lane_change_state", String, self.state_callback
        )
        rospy.Subscriber(
            "/highway/longitudinal_mode", String, self.mode_callback
        )
        rospy.Subscriber(
            "/highway/target_speed_kph", Float64, self.speed_callback
        )

    def lane_callback(self, message):
        self.lanes.add(message.data)

    def state_callback(self, message):
        self.states.add(message.data)
        if message.data == "EXECUTE" and self.execute_time is None:
            self.execute_time = time.monotonic() - self.start

    def mode_callback(self, message):
        self.modes.add(message.data)

    def speed_callback(self, message):
        if message.data > 0.0:
            self.minimum_speed_kph = min(self.minimum_speed_kph, message.data)

    def complete(self):
        common = (
            {1, 2, 3, 4}.issubset(self.lanes)
            and {"CHECK_GAP", "EXECUTE", "SETTLE"}.issubset(self.states)
        )
        if not common:
            return False
        if self.scenario == "rear_blocked":
            return self.execute_time is not None and self.execute_time >= 1.5
        if self.scenario == "front_follow":
            return "FOLLOW" in self.modes and self.minimum_speed_kph < 69.0
        return True

    def summary(self):
        return (
            "scenario={} lanes={} states={} modes={} min_speed={:.1f} "
            "first_execute={}".format(
                self.scenario,
                sorted(self.lanes),
                sorted(self.states),
                sorted(self.modes),
                self.minimum_speed_kph,
                "none" if self.execute_time is None else "{:.2f}s".format(
                    self.execute_time
                ),
            )
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--timeout", type=float, default=40.0)
    arguments = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("highway_adas_synthetic_verifier", anonymous=True)
    verifier = ScenarioVerifier(arguments.scenario)
    deadline = time.monotonic() + arguments.timeout
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if verifier.complete():
            print("PASS: " + verifier.summary())
            return 0
        rate.sleep()
    print("FAIL: " + verifier.summary())
    return 1


if __name__ == "__main__":
    sys.exit(main())
