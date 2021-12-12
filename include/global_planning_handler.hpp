/*
 * global_planning_handler.hpp
 *
 *  Created on: Nov 12, 2021
 *  * Original Author: 	Eitan Marder-Eppstein
 *    Modified by: 		hankm
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

namespace autoexplorer
{

using namespace std;

class GlobalPlanningHandler
{

public:
	GlobalPlanningHandler( );
	virtual ~GlobalPlanningHandler();

	void initialization() ;
	void reinitialization( costmap_2d::Costmap2D* pocostmap2d ) ;

	bool makePlan( const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal,
			  	  std::vector<geometry_msgs::PoseStamped>& plan );

	bool makePlan(  const int& tid, const float& fbound, const bool& boneqgrid,
					const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
					std::vector<geometry_msgs::PoseStamped>& plan, float& fendpotential );

    /**
     * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
     * @param world_point The point to get the potential for
     * @return The navigation function's value at that point in the world
     */
    double getPointPotential(const geometry_msgs::Point& world_point);

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
    costmap_2d::Costmap2D* mp_costmap;
	//boost::shared_ptr<costmap_2d::Costmap2D> mp_costmap;

//	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

//    tf2_ros::Buffer& tf_;

    boost::shared_ptr<navfn::NavFn> planner_;
    std::string robot_base_frame_, global_frame_;

    float mf_tolerance ;
    boost::mutex m_mutex;
    bool mb_initialized ;
    bool mb_allow_unknown;
    bool mb_visualize_potential ;
    float mf_endpotential ; // pot @ start ( pot propagates from the goal )
};

}

#endif /* INCLUDE_GLOBAL_PLANNING_HANDLER_HPP_ */
