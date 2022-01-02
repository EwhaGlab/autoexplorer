/*
 * gazebo_handler.cpp
 *
 *  Created on: Dec 27, 2021
 *      Author: hankm
 */



#include "gazebo_handler.hpp"

namespace autoexplorer
{

GazeboHandler::GazeboHandler(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_)
{

}

GazeboHandler::~GazeboHandler(){};

//void GazeboHandler::reset_world()
//{
//
//}

}
