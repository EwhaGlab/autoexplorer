/*
 * gazebo_handler.hpp
 *
 *  Created on: Dec 27, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_GAZEBO_HANDLER_HPP_
#define INCLUDE_GAZEBO_HANDLER_HPP_

#include <ros/ros.h>
#include <ros/console.h>

namespace autoexplorer
{

class GazeboHandler
{
	GazeboHandler(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~GazeboHandler();

	void resetWorldSrv();

protected:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

};

}




#endif /* INCLUDE_GAZEBO_HANDLER_HPP_ */
