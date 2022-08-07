/*
 * twist2twiststamped.hpp
 *
 *  Created on: 2022. 8. 5.
 *      Author: hankm
 */

#ifndef INCLUDE_TWIST2TWISTSTAMPED_HPP_
#define INCLUDE_TWIST2TWISTSTAMPED_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

namespace twist2twiststamped
{
	class Twist2TwistStamped
	{
	public:

		Twist2TwistStamped(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_ )
		:m_nh(nh_),
		 m_nh_private(private_nh_)
		{
			m_twiststamped_pub = m_nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_twiststamped", 1000);
			m_cmdvel_sub = m_nh.subscribe("cmd_vel", 1000, &Twist2TwistStamped::stampCallback, this);
		}
		~Twist2TwistStamped()
		{
			ROS_WARN("finishing twist2twistamped node \n");
		};

		void stampCallback(const geometry_msgs::Twist::ConstPtr& msg)
		{
			geometry_msgs::TwistStamped cmd_vel ;
			cmd_vel.twist.linear  = (*msg).linear ;
			cmd_vel.twist.angular = (*msg).angular ;
			cmd_vel.header.stamp = ros::Time::now();
			cmd_vel.header.frame_id = std::string("vehicle");

			m_twiststamped_pub.publish(cmd_vel);
		}

		ros::NodeHandle m_nh;
		ros::NodeHandle m_nh_private;
		ros::Publisher m_twiststamped_pub ;
		ros::Subscriber m_cmdvel_sub ;
	};

}

#endif /* INCLUDE_TWIST2TWISTSTAMPED_HPP_ */
