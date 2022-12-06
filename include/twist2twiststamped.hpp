/*********************************************************************
Copyright 2022 The Ewha Womans University.
All Rights Reserved.
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
Permission to use, copy, modify OR distribute this software and its
documentation for educational, research and non-profit purposes, without
fee, and without a written agreement is hereby granted, provided that the
above copyright notice and the following three paragraphs appear in all
copies.

IN NO EVENT SHALL THE EWHA WOMANS UNIVERSITY BE
LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE EWHA WOMANS UNIVERSITY
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

THE EWHA WOMANS UNIVERSITY SPECIFICALLY DISCLAIM ANY
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE EWHA WOMANS UNIVERSITY
HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


The authors may be contacted via:


Mail:        Y. J. Kim, Kyung Min Han
             Computer Graphics Lab                       
             Department of Computer Science and Engineering
             Ewha Womans University
             11-1 Daehyun-Dong Seodaemun-gu, Seoul, Korea 120-750


Phone:       +82-2-3277-6798


EMail:       kimy@ewha.ac.kr
             hankm@ewha.ac.kr
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
