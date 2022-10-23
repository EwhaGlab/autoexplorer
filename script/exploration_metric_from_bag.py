#! /usr/bin/env python

'''
'''
import sys
import time
import os
import numpy as np
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
import tf
import pickle
import yaml
from PIL import Image
import math

exploration_rate_log = []
odom_log = []
path_length_log = []
time_log = []
end_flag = False
achieve_80 = False
achieve_85 = False
achieve_90 = False
achieve_95 = False
start_time = 0
last_msg_time = 0
gt_area = 0
begin_timing = False
num_robots = 1

single_map_list = [OccupancyGrid() for i in range(num_robots)]
single_robot_coverage_rate_list = [0 for i in range(num_robots)]

def get_gt(pgm_file, yaml_file):
    map_img = np.array(Image.open(pgm_file))
    map_info = yaml.load(open(yaml_file, mode='r'))
    gt_area = (np.sum((map_img != 205).astype(int)))*map_info['resolution']*map_info['resolution']
    return gt_area

def callback(data):
    
    # -1:unkown 0:free 100:obstacle
    global end_flag, start_time, achieve_80, achieve_85, achieve_90, achieve_95, last_msg_time, begin_timing
    
    #msg_secs = data.header.stamp.secs
    now = rospy.get_time()
    
    #print("{} {}".format(msg_secs, now) )
    
    if (now + 3 < last_msg_time):
        return
    else:
        last_msg_time = now

    
    if not end_flag:
        gridmap = np.array(data.data).reshape((data.info.height, data.info.width))
        explored_map = (gridmap != -1).astype(int)
        explored_area = explored_map.sum()*data.info.resolution*data.info.resolution
        exploration_rate = explored_area / gt_area
        exploration_rate_over_time = dict()
        exploration_rate_over_time['time'] = data.header.stamp
        exploration_rate_over_time['rate'] = exploration_rate
        curr_time = time.time()

        exploration_rate_log.append(exploration_rate_over_time)
        print("exploration time: {} rate: {}".format( curr_time - start_time, exploration_rate ) )
        
        if exploration_rate >= 0.8 and (not achieve_80): 
            print("achieve 0.8 coverage rate!")
            print("T_80: {}".format( curr_time - start_time) )
            achieve_80 = True
            
        if exploration_rate >= 0.85 and (not achieve_85):
            print("achieve 0.85 coverage rate!")
            print("T_85: {}".format( curr_time - start_time) )
            achieve_85 = True
            
        if exploration_rate >= 0.9 and (not achieve_90):
            print("achieve 0.9 coverage rate!")
            print("T_90: {}".format( curr_time - start_time) )
            achieve_90 = True
        
        if exploration_rate >= 0.95 and (not achieve_95):
            print("achieve 0.95 coverage rate!")
            print("T_95: ", curr_time - start_time)
            achieve_95 = True

        if exploration_rate >= 0.99:
            print("exploration ends!")
            print("T_total: ", curr_time - start_time)
            # # compute coverage std
            # coverage_std = np.std(np.array(single_robot_coverage_rate_list))
            # print("exploration coverage std: ", coverage_std)
            # # compute overlap rate
            # overlap_rate = np.sum(np.array(single_robot_coverage_rate_list)) - 1
            # print("exploration overlap rate: ", overlap_rate)

            end_flag = True        
        #time.sleep(1)
    #else:
        #time.sleep(1)

def odom_callback(data):
    global end_flag
    if not end_flag:
        current_pos = [data.pose.pose.position.x, data.pose.pose.position.y]
        odom_over_time = dict()
        odom_over_time['time'] = data.header.stamp
        odom_over_time['odom'] = current_pos
        if len(odom_log) == 0:
            odom_log.append(odom_over_time)
            path_length_over_time = dict()
            path_length_over_time['time'] = data.header.stamp
            path_length_over_time['path_length'] = 0
            path_length_log.append(path_length_over_time)
        else:
            path_length_over_time = dict()
            path_length_over_time['time'] = data.header.stamp
            path_length_over_time['path_length'] = path_length_log[-1]['path_length'] + math.hypot(odom_log[-1]['odom'][0]-current_pos[0], odom_log[-1]['odom'][1]-current_pos[1])
            path_length_log.append(path_length_over_time)
            odom_log.append(odom_over_time)
        time.sleep(1)
    else:
        time.sleep(1)

def single_map_callback(data):
    global single_map_list
    # print(int(data.header.frame_id[5]))
    single_map_list[int(data.header.frame_id[5])-1] = data

def single_robot_coverage_rate_callback(data):
    global single_robot_coverage_rate_list, gt_area
    gridmap = np.array(data.data).reshape((data.info.height, data.info.width))
    explored_map = (gridmap != -1).astype(int)
    explored_area = explored_map.sum()*data.info.resolution*data.info.resolution
    exploration_rate = explored_area / gt_area
    single_robot_coverage_rate_list[int(data.header.frame_id[5])-1] = exploration_rate

def StartCallback(data):
    global start_time, begin_timing
    start_time = time.time()
    last_msg_time = start_time
    print("Exploration start!")
    print("Start time: ", start_time)

def main(argv):
    global gt_area
    gt_area = get_gt(argv[1], argv[2]) 
    rospy.init_node('exploration_metric', anonymous=True)
    rospy.Subscriber("begin_exploration", Bool, StartCallback)
    rospy.Subscriber("robot1/map",  OccupancyGrid, callback, queue_size=1)
    #rospy.Subscriber("odom", Odometry, odom_callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

