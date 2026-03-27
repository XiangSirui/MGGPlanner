#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rospy
from std_msgs.msg import Float32, Float32MultiArray

mpl.rcParams['toolbar'] = 'None'
plt.ion()

time_duration = 0
start_time_duration = 0
first_iteration = 'True'

explored_volume = 0;
traveling_distance = 0;
run_time = 0;
battery_remaining_percent = 100.0
max_explored_volume = 0
max_traveling_diatance = 0
max_run_time = 0

time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])
time_list4 = np.array([])
run_time_list = np.array([])
explored_volume_list = np.array([])
traveling_distance_list = np.array([])
battery_remaining_percent_list = np.array([])
# Pairwise inter-robot distances [d12, d13, d23] for 3 robots
inter_dist_12_list = np.array([])
inter_dist_13_list = np.array([])
inter_dist_23_list = np.array([])
latest_inter_d12 = 0.0
latest_inter_d13 = 0.0
latest_inter_d23 = 0.0

def timeDurationCallback(msg):
    global time_duration, start_time_duration, first_iteration
    time_duration = msg.data
    if first_iteration == 'True':
        start_time_duration = time_duration
        first_iteration = 'False'

def runTimeCallback(msg):
    global run_time
    run_time = msg.data

def exploredVolumeCallback(msg):
    global explored_volume
    explored_volume = msg.data


def travelingDistanceCallback(msg):
    global traveling_distance
    traveling_distance = msg.data

def interRobotDistancesCallback(msg):
    """Float32MultiArray: [d_R1_R2, d_R1_R3, d_R2_R3] from inter_robot_distances node."""
    global latest_inter_d12, latest_inter_d13, latest_inter_d23
    d = msg.data
    if len(d) >= 3:
        latest_inter_d12 = d[0]
        latest_inter_d13 = d[1]
        latest_inter_d23 = d[2]
    elif len(d) == 2:
        latest_inter_d12 = d[0]
        latest_inter_d13 = d[1]
        latest_inter_d23 = 0.0
    elif len(d) == 1:
        latest_inter_d12 = d[0]
        latest_inter_d13 = 0.0
        latest_inter_d23 = 0.0

def batteryRemainingPercentCallback(msg):
    global battery_remaining_percent
    battery_remaining_percent = msg.data

def listener():
  global time_duration, start_time_duration, explored_volume, traveling_distance, run_time, battery_remaining_percent, max_explored_volume, max_traveling_diatance, max_run_time, time_list1, time_list2, time_list3, time_list4, run_time_list, explored_volume_list, traveling_distance_list, battery_remaining_percent_list
  global inter_dist_12_list, inter_dist_13_list, inter_dist_23_list, latest_inter_d12, latest_inter_d13, latest_inter_d23

  rospy.init_node('realTimePlot')
  rospy.Subscriber("time_duration", Float32, timeDurationCallback)
  rospy.Subscriber("runtime", Float32, runTimeCallback)
  rospy.Subscriber("explored_volume", Float32, exploredVolumeCallback)
  rospy.Subscriber("traveling_distance", Float32, travelingDistanceCallback)
  rospy.Subscriber("battery_remaining_percent", Float32, batteryRemainingPercentCallback)
  # Global topic from inter_robot_distances.py (run once at simulation root)
  rospy.Subscriber("/inter_robot_pairwise_distances", Float32MultiArray, interRobotDistancesCallback)

  fig=plt.figure(figsize=(8,13))
  fig1=fig.add_subplot(511)
  plt.title("Exploration Metrics\n", fontsize=14)
  plt.margins(x=0.001)
  fig1.set_ylabel("Explored\nVolume (m$^3$)", fontsize=12)
  l1, = fig1.plot(time_list2, explored_volume_list, color='r', label='Explored Volume')
  fig2=fig.add_subplot(512)
  fig2.set_ylabel("Traveling\nDistance (m)", fontsize=12)
  l2, = fig2.plot(time_list3, traveling_distance_list, color='r', label='Traveling Distance')
  fig3=fig.add_subplot(513)
  fig3.set_ylabel("Algorithm\nRuntime (s)", fontsize=12)
  l3, = fig3.plot(time_list1, run_time_list, color='r', label='Algorithm Runtime')
  fig4=fig.add_subplot(514)
  fig4.set_ylabel("Inter-robot\ndistance (m)", fontsize=12)
  fig4.set_xlabel("Time Duration (s)", fontsize=12)
  ld12, = fig4.plot(time_list4, inter_dist_12_list, color='b', label='R1-R2')
  ld13, = fig4.plot(time_list4, inter_dist_13_list, color='g', label='R1-R3')
  ld23, = fig4.plot(time_list4, inter_dist_23_list, color='orange', label='R2-R3')
  fig4.legend(loc='upper right', fontsize=8)
  fig5=fig.add_subplot(515)
  fig5.set_ylabel("Battery\nRemaining (%)", fontsize=12)
  fig5.set_xlabel("Time Duration (s)", fontsize=12)
  l5, = fig5.plot(time_list3, battery_remaining_percent_list, color='purple', label='Battery Remaining')

  count = 0
  r = rospy.Rate(100) # 100hz
  while not rospy.is_shutdown():
      r.sleep()
      count = count + 1

      if count % 25 == 0:
        max_explored_volume = explored_volume
        max_traveling_diatance = traveling_distance
        if run_time > max_run_time:
            max_run_time = run_time

        time_list2 = np.append(time_list2, time_duration)
        explored_volume_list = np.append(explored_volume_list, explored_volume)
        time_list3 = np.append(time_list3, time_duration)
        traveling_distance_list = np.append(traveling_distance_list, traveling_distance)
        time_list1 = np.append(time_list1, time_duration)
        run_time_list = np.append(run_time_list, run_time)
        time_list4 = np.append(time_list4, time_duration)
        inter_dist_12_list = np.append(inter_dist_12_list, latest_inter_d12)
        inter_dist_13_list = np.append(inter_dist_13_list, latest_inter_d13)
        inter_dist_23_list = np.append(inter_dist_23_list, latest_inter_d23)
        battery_remaining_percent_list = np.append(battery_remaining_percent_list, battery_remaining_percent)

      if count >= 100:
        count = 0
        l1.set_xdata(time_list2)
        l2.set_xdata(time_list3)
        l3.set_xdata(time_list1)
        ld12.set_xdata(time_list4)
        ld13.set_xdata(time_list4)
        ld23.set_xdata(time_list4)
        l1.set_ydata(explored_volume_list)
        l2.set_ydata(traveling_distance_list)
        l3.set_ydata(run_time_list)
        ld12.set_ydata(inter_dist_12_list)
        ld13.set_ydata(inter_dist_13_list)
        ld23.set_ydata(inter_dist_23_list)
        l5.set_xdata(time_list3)
        l5.set_ydata(battery_remaining_percent_list)

        max_d = max(
            float(np.max(inter_dist_12_list)) if inter_dist_12_list.size else 0.0,
            float(np.max(inter_dist_13_list)) if inter_dist_13_list.size else 0.0,
            float(np.max(inter_dist_23_list)) if inter_dist_23_list.size else 0.0,
            1.0,
        )

        fig1.set_ylim(0, max_explored_volume + 500)
        fig1.set_xlim(start_time_duration, time_duration + 10)
        fig2.set_ylim(0, max_traveling_diatance + 20)
        fig2.set_xlim(start_time_duration, time_duration + 10)
        fig3.set_ylim(0, max_run_time + 0.2)
        fig3.set_xlim(start_time_duration, time_duration + 10)
        fig4.set_ylim(0, max_d * 1.1 + 5.0)
        fig4.set_xlim(start_time_duration, time_duration + 10)
        fig5.set_ylim(0, 105)
        fig5.set_xlim(start_time_duration, time_duration + 10)

        fig.canvas.draw()

if __name__ == '__main__':
  listener()
  print("1")
