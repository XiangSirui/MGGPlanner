#!/usr/bin/env python3
"""
Publish pairwise Euclidean distances between robots (in a common world frame) as
std_msgs/Float32MultiArray on /inter_robot_pairwise_distances.

Layout for N robots (frames): [d_0_1, d_0_2, ..., d_0_{N-1}, d_1_2, ...]
For 3 robots: [d_R1_R2, d_R1_R3, d_R2_R3].

Requires TF: world_frame -> each robot_base_frame (e.g. R1/base_link).
"""
from __future__ import print_function

import math
import rospy
import tf2_ros
from std_msgs.msg import Float32MultiArray


def pairwise_distances_3d(positions):
    """positions: list of (x,y,z), same order as robot_base_frames."""
    out = []
    n = len(positions)
    for i in range(n):
        for j in range(i + 1, n):
            dx = positions[i][0] - positions[j][0]
            dy = positions[i][1] - positions[j][1]
            dz = positions[i][2] - positions[j][2]
            out.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    return out


def pairwise_distances_xy(positions):
    out = []
    n = len(positions)
    for i in range(n):
        for j in range(i + 1, n):
            dx = positions[i][0] - positions[j][0]
            dy = positions[i][1] - positions[j][1]
            out.append(math.sqrt(dx * dx + dy * dy))
    return out


def main():
    rospy.init_node("inter_robot_distances", anonymous=False)

    world_frame = rospy.get_param("~world_frame", "world")
    robot_frames = rospy.get_param(
        "~robot_base_frames",
        ["R1/base_link", "R2/base_link", "R3/base_link"],
    )
    use_xy_only = rospy.get_param("~use_xy_only", False)
    rate_hz = rospy.get_param("~rate_hz", 10.0)
    publish_topic = rospy.get_param(
        "~publish_topic", "/inter_robot_pairwise_distances"
    )

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher(publish_topic, Float32MultiArray, queue_size=10)
    rospy.loginfo(
        "[%s] world=%s frames=%s -> %s",
        rospy.get_name(),
        world_frame,
        robot_frames,
        publish_topic,
    )

    r = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        positions = []
        ok = True
        for frame in robot_frames:
            try:
                t = tf_buffer.lookup_transform(
                    world_frame, frame, rospy.Time(0)
                )
                tr = t.transform.translation
                positions.append((tr.x, tr.y, tr.z))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as ex:
                rospy.logwarn_throttle(
                    5.0, "TF %s -> %s : %s", world_frame, frame, ex
                )
                ok = False
                break

        if ok and len(positions) == len(robot_frames):
            if use_xy_only:
                dists = pairwise_distances_xy(positions)
            else:
                dists = pairwise_distances_3d(positions)
            msg = Float32MultiArray()
            msg.data = dists
            pub.publish(msg)

        r.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
