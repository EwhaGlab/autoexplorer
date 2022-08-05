/*********************************************************************
* Software License Agreement (XXX License)
*
*  Copyright (c) 2022, Ewha Graphics Lab
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************

 *  Created on: Apr, 2022
 *      Author: Kyungmin Han (hankm@ewha.ac.kr)
*/


#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>


class Twist2TwistStamped
{
public:

	Twist2TwistStamped(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_ )
	:m_nh(nh_),
	 m_nh_private(private_nh_)
	{
		twiststamped_pub = m_nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_twiststamped", 1000);
		ros::Subscriber sub = m_nh.subscribe("cmd_vel", 1000, &Twist2TwistStamped::stampCallback, this);
	}
	virtual ~Twist2TwistStamped(){};

	void stampCallback(const geometry_msgs::Twist::ConstPtr& msg)
	{
		geometry_msgs::TwistStamped cmd_vel ;
		cmd_vel.twist.linear  = (*msg).linear ;
		cmd_vel.twist.angular = (*msg).angular ;
		cmd_vel.header.stamp = ros::Time::now();
		cmd_vel.header.frame_id = std::string("vehicle");

		twiststamped_pub.publish(cmd_vel);
	}

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;
	ros::Publisher twiststamped_pub ;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist2twiststamped");
  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  Twist2TwistStamped oTwist2TwistStamped(private_nh, nh);

  ros::spin();
  return 0;
}



