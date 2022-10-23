/*********************************************************************
*  Copyright (c) 2022, Ewha Graphics Lab
*
* This file is a part of Autoexplorer
*
* Autoexplorer is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Autoexplorer is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Autoexplorer.
* If not, see <http://www.gnu.org/licenses/>.

*      Author: Kyungmin Han (hankm@ewha.ac.kr)
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
			cmd_vel.header.seq = m_seq;

			m_seq++;
			m_twiststamped_pub.publish(cmd_vel);
		}

		uint32_t m_seq = 0;
		ros::NodeHandle m_nh;
		ros::NodeHandle m_nh_private;
		ros::Publisher m_twiststamped_pub ;
		ros::Subscriber m_cmdvel_sub ;
	};

}

#endif /* INCLUDE_TWIST2TWISTSTAMPED_HPP_ */
