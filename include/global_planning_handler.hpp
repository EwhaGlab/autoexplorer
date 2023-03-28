/*********************************************************************
Copyright 2022 The Ewha Womans University.
All Rights Reserved.
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

#ifndef INCLUDE_GLOBAL_PLANNING_HANDLER_HPP_
#define INCLUDE_GLOBAL_PLANNING_HANDLER_HPP_

#include <ros/ros.h>
#include <nav_fn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>
//#include <nav_fn/potarr_point.h>

namespace autoexplorer
{

using namespace std;

class GlobalPlanningHandler
{

public:
	GlobalPlanningHandler(  );
	GlobalPlanningHandler( costmap_2d::Costmap2D &ocostmap,  const std::string& worldframe, const std::string& baseframe, bool b_allow_unknown );
	virtual ~GlobalPlanningHandler();

	void initialization() ;
	void reinitialization(  ) ;

	bool makePlan( const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal,
			  	  std::vector<geometry_msgs::PoseStamped>& plan );

	int makePlan(   const int& tid, const float& fbound, const bool& boneqgrid,
					const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
					std::vector<geometry_msgs::PoseStamped>& plan, float& fendpotential );

    /**
     * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
     * @param world_point The point to get the potential for
     * @return The navigation function's value at that point in the world
     */
    float getPointPotential(const geometry_msgs::Point& world_point);

    bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) ;

//    void setCostmap( uint8_t* cmap, unsigned int size_x, unsigned int size_y, float resolution,
//    		float origin_x, float origin_y );

    void setCostmap( vector<signed char> cmap, unsigned int size_x, unsigned int size_y, float resolution,
    		float origin_x, float origin_y );
private:

    inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
      double dx = p1.pose.position.x - p2.pose.position.x;
      double dy = p1.pose.position.y - p2.pose.position.y;
      return dx*dx +dy*dy;
    }

    signed char* mp_cost_translation_table;

    void mapToWorld(double mx, double my, double& wx, double& wy);
    void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);

	// global planner and costmap related variables
    //costmap_2d::Costmap2D* mp_costmap;
    costmap_2d::Costmap2D m_costmap;

	//boost::shared_ptr<costmap_2d::Costmap2D> mp_costmap;

//	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

//    tf2_ros::Buffer& tf_;

    boost::shared_ptr<navfn::NavFn> planner_;
    std::string robot_base_frame_, global_frame_;

    float mf_tolerance ;
    //boost::mutex m_mutex;
    bool mb_initialized ;
    bool mb_allow_unknown;
    bool mb_visualize_potential ;
    float mf_endpotential ; // pot @ start ( pot propagates from the goal )
};

}

#endif /* INCLUDE_GLOBAL_PLANNING_HANDLER_HPP_ */
