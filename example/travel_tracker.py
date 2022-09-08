#!/usr/bin/env python
import rospy
from math import sqrt
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from nav_msgs.msg import Path

class OdometryRecorder:

  def __init__(self):
    self.sub = rospy.Subscriber("odom", Odometry, self.callback)
    self.pub_dist = rospy.Publisher('total_distance', Float32, queue_size=1)
    self.pub_traj = rospy.Publisher('total_traj', Path, queue_size=1)

    self.total_distance = 0.
    self.previous_x = 0
    self.previous_y = 0
    self.first_run = true
    self.traj = Path

  def callback(self, data):
    if(self.first_run):
      self.previous_x = data.pose.pose.position.x
      self.previous_y = data.pose.pose.position.y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    d_increment = sqrt((x - self.previous_x) * (x - self.previous_x) +
                   (y - self.previous_y)(y - self.previous_y))
    self.total_distance = self.total_distance + d_increment
    print("Total distance traveled is {.2f}m".format(self.total_distance))
    
    geometry_msgs.PoseStamped currpose
    currpose.pose.point.x = x
    currpose.pose.point.y = y
    currpose.pose.point.z = 0
    self.traj.push_back( currpose )
    
    self.pub_dist.publish(self.total_distance)
    self.pub_traj.publish(self.traj)
    
    self.previous_x = data.pose.pose.position.x
    self.previous_y = data.pose.pose.position.y
    
    self.first_run = false


if __name__ == '__main__':
  try:
    rospy.init_node('traj_recorder', anonymous=True)
    
    ros::Rate rate(0.5)
    odom = OdometryRecorder()
    
     while(ros::ok())
     {
        // Run your function and check condition
        
         rate.sleep(); // runs out whatever duration is remaining for the 1 second interval
         ros::spinOnce();
     }
    
