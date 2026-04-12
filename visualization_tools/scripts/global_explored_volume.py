#!/usr/bin/env python3
from __future__ import print_function

import os
import threading

import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


class GlobalExploredVolumeNode(object):
    def __init__(self):
        self.voxel_size = rospy.get_param("~voxel_size", 0.5)
        self.publish_topic = rospy.get_param("~publish_topic", "/overall_explored_volume")
        self.cloud_topics = rospy.get_param(
            "~cloud_topics",
            [
                "/R1/mggplanner_node/surface_pointcloud",
                "/R2/mggplanner_node/surface_pointcloud",
                "/R3/mggplanner_node/surface_pointcloud",
            ],
        )
        self.time_topic = rospy.get_param("~time_topic", "/R1/ground_truth/state")
        self.write_hz = rospy.get_param("~write_hz", 2.0)
        self.metric_file = rospy.get_param("~metric_file", "")

        self._lock = threading.Lock()
        self._voxels = set()
        self._start_time = None
        self._time_duration = 0.0
        self._metric_fp = None

        if self.metric_file:
            parent = os.path.dirname(self.metric_file)
            if parent and not os.path.exists(parent):
                os.makedirs(parent, exist_ok=True)
            self._metric_fp = open(self.metric_file, "w")
            self._metric_fp.write("# overall_explored_volume_m3 time_duration_s\n")
            self._metric_fp.flush()

        self.pub = rospy.Publisher(self.publish_topic, Float32, queue_size=5)
        self.subs = []
        for t in self.cloud_topics:
            self.subs.append(rospy.Subscriber(t, PointCloud2, self._cloud_cb, queue_size=2))
        self.time_sub = rospy.Subscriber(self.time_topic, Odometry, self._time_cb, queue_size=5)

        timer_period = 1.0 / max(self.write_hz, 0.2)
        self.timer = rospy.Timer(rospy.Duration(timer_period), self._timer_cb)

        rospy.loginfo(
            "[%s] union-volume voxel=%.3f topics=%s out=%s",
            rospy.get_name(),
            self.voxel_size,
            self.cloud_topics,
            self.publish_topic,
        )

    def _time_cb(self, msg):
        now = msg.header.stamp.to_sec()
        if self._start_time is None:
            self._start_time = now
            self._time_duration = 0.0
        else:
            self._time_duration = max(0.0, now - self._start_time)

    def _cloud_cb(self, msg):
        # Union in voxel space to avoid double-counting overlaps among robots.
        inv = 1.0 / max(self.voxel_size, 1e-6)
        new_keys = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            kx = int(round(p[0] * inv))
            ky = int(round(p[1] * inv))
            kz = int(round(p[2] * inv))
            new_keys.append((kx, ky, kz))
        if not new_keys:
            return
        with self._lock:
            self._voxels.update(new_keys)

    def _timer_cb(self, _evt):
        with self._lock:
            n = len(self._voxels)
        volume = (self.voxel_size ** 3) * float(n)

        msg = Float32()
        msg.data = volume
        self.pub.publish(msg)

        if self._metric_fp is not None:
            self._metric_fp.write("{:.6f} {:.6f}\n".format(volume, self._time_duration))
            self._metric_fp.flush()

    def close(self):
        if self._metric_fp is not None:
            self._metric_fp.close()
            self._metric_fp = None


def main():
    rospy.init_node("global_explored_volume", anonymous=False)
    node = GlobalExploredVolumeNode()
    try:
        rospy.spin()
    finally:
        node.close()


if __name__ == "__main__":
    main()
