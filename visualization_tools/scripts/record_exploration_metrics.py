#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Optional CSV logger for exploration experiments (alongside metrics*.txt from
visualization_tools). Subscribes to per-robot topics published by visualizationTools:

  /<robot>/explored_volume      (std_msgs/Float32)
  /<robot>/traveling_distance
  /<robot>/time_duration
  /<robot>/battery_remaining_percent  (optional)

Example:
  rosrun visualization_tools record_exploration_metrics.py _output:=~/exp_run1.csv

  rosrun visualization_tools record_exploration_metrics.py \\
    _output:=/tmp/metrics.csv _rate:=2.0 _robots:="R1,R2,R3"

Stop with Ctrl+C; file is flushed on exit. Last row is a good snapshot for
"final explored volume" per robot.
"""

from __future__ import print_function

import csv
import os
import sys
import time

import rospy
from std_msgs.msg import Float32


class MetricsRecorder(object):
    def __init__(self):
        self.robots = [
            r.strip()
            for r in rospy.get_param("~robots", "R1,R2,R3").split(",")
            if r.strip()
        ]
        self.rate_hz = float(rospy.get_param("~rate", 1.0))
        out = rospy.get_param("~output", "")
        if not out:
            rospy.logerr("Param ~output must be set (e.g. _output:=/path/run.csv)")
            sys.exit(1)
        self.output_path = os.path.expanduser(out)
        self._latest = {}
        self._subs = []
        for r in self.robots:
            p = "/" + r
            self._latest[(r, "explored_volume")] = float("nan")
            self._latest[(r, "traveling_distance")] = float("nan")
            self._latest[(r, "time_duration")] = float("nan")
            self._latest[(r, "battery_remaining_percent")] = float("nan")
            self._subs.append(
                rospy.Subscriber(
                    p + "/explored_volume",
                    Float32,
                    self._cb,
                    callback_args=(r, "explored_volume"),
                    queue_size=5,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    p + "/traveling_distance",
                    Float32,
                    self._cb,
                    callback_args=(r, "traveling_distance"),
                    queue_size=5,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    p + "/time_duration",
                    Float32,
                    self._cb,
                    callback_args=(r, "time_duration"),
                    queue_size=5,
                )
            )
            self._subs.append(
                rospy.Subscriber(
                    p + "/battery_remaining_percent",
                    Float32,
                    self._cb,
                    callback_args=(r, "battery_remaining_percent"),
                    queue_size=5,
                )
            )

        d = os.path.dirname(self.output_path)
        if d and not os.path.isdir(d):
            os.makedirs(d)

        self._fp = open(self.output_path, "w", newline="")
        self._writer = csv.writer(self._fp)
        header = ["wall_time_ros", "wall_time_sec"]
        for r in self.robots:
            header += [
                r + "_explored_volume",
                r + "_traveling_distance",
                r + "_time_duration",
                r + "_battery_percent",
            ]
        self._writer.writerow(header)
        self._fp.flush()

        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 0.1)), self._on_timer)
        rospy.on_shutdown(self._close)
        rospy.loginfo(
            "Recording to %s at %.2f Hz for robots %s",
            self.output_path,
            self.rate_hz,
            ", ".join(self.robots),
        )

    def _cb(self, msg, args):
        robot, key = args
        self._latest[(robot, key)] = float(msg.data)

    def _on_timer(self, _evt):
        now = rospy.Time.now().to_sec()
        row = [repr(now), "%.3f" % now]
        for r in self.robots:
            row.append(self._latest[(r, "explored_volume")])
            row.append(self._latest[(r, "traveling_distance")])
            row.append(self._latest[(r, "time_duration")])
            row.append(self._latest[(r, "battery_remaining_percent")])
        self._writer.writerow(row)
        self._fp.flush()

    def _close(self):
        try:
            self._fp.close()
        except Exception:
            pass
        rospy.loginfo("Closed CSV: %s", self.output_path)


def main():
    rospy.init_node("record_exploration_metrics", anonymous=False)
    MetricsRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
