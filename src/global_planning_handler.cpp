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


#include <global_planning_handler.hpp>

namespace autoexplorer
{

GlobalPlanningHandler::GlobalPlanningHandler( ):
robot_base_frame_("base_footprint"), // was "base_link"
global_frame_("map"),
mb_initialized(false), mb_visualize_potential(false),
mf_tolerance(0.0),
mp_cost_translation_table(NULL)
{
	planner_ = boost::shared_ptr<navfn::NavFn>( new navfn::NavFn(0, 0));
	mb_initialized = true;
	//mb_allow_unknown = false ;
}

GlobalPlanningHandler::GlobalPlanningHandler( costmap_2d::Costmap2D &ocostmap, const std::string& worldframe, const std::string& baseframe, bool b_allow_unknown  ):
robot_base_frame_(baseframe), // was "base_link"
global_frame_(worldframe),
mb_initialized(false), mb_allow_unknown(b_allow_unknown), mb_visualize_potential(false),
mf_tolerance(0.0),
mp_cost_translation_table(NULL)
{
    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
	//ROS_INFO(" gph initilized with (world frame: %s) and (base frame: %s) \n", global_frame_.c_str(), robot_base_frame_.c_str() );

	m_costmap = costmap_2d::Costmap2D(ocostmap) ;
	planner_ = boost::shared_ptr<navfn::NavFn>( new navfn::NavFn(0, 0));
	mb_initialized = true;
}

GlobalPlanningHandler::~GlobalPlanningHandler()
{
    if(mp_cost_translation_table != NULL)
      delete [] mp_cost_translation_table;
}

//
//void GlobalPlanningHandler::initialization()
//{
//    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
//
//	if(mp_costmap == NULL)
//	{
//		mp_costmap = new costmap_2d::Costmap2D() ;
//		//mp_costmap = boost::shared_ptr<costmap_2d::Costmap2D>( new costmap_2d::Costmap2D() );
//	}
//
//	robot_base_frame_ = string("base_link");
//	global_frame_ = string("map");
//	mb_initialized = false;
//	mb_allow_unknown = true;
//	mb_visualize_potential = false;
//	mf_tolerance = 0.0;
//
//	if (mp_cost_translation_table == NULL)
//	{
//		mp_cost_translation_table = new signed char[101];
//
//		// special values:
//		mp_cost_translation_table[0] = 0;  // NO obstacle
//		mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
//		mp_cost_translation_table[100] = 254;  // LETHAL obstacle
////		mp_cost_translation_table[-1] = 255;  // UNKNOWN
//
//		// regular cost values scale the range 1 to 252 (inclusive) to fit
//		// into 1 to 98 (inclusive).
//		for (int i = 1; i < 99; i++)
//		{
//			mp_cost_translation_table[ i ] = char( ((i-1)*251 -1 )/97+1 );
//		}
//	}
//
//	if(planner_ == NULL)
//	{
//		planner_ = boost::shared_ptr<navfn::NavFn>( new navfn::NavFn(0, 0));
//	}
//
//	mb_initialized = true;
//}
//

void GlobalPlanningHandler::reinitialization(  )
{
    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map

//	robot_base_frame_ = string("base_link");
//	global_frame_ = string("map");
//	mb_initialized = false;
//	mb_allow_unknown = true;
	mb_visualize_potential = false;
	mf_tolerance = 0.0;

	planner_ = boost::shared_ptr<navfn::NavFn>( new navfn::NavFn(m_costmap.getSizeInCellsX(), m_costmap.getSizeInCellsY()) );
	mb_initialized = true;
}

//
//bool GlobalPlanningHandler::validPointPotential(const geometry_msgs::Point& world_point){
//  return validPointPotential(world_point, default_tolerance_);
//}
//
//bool GlobalPlanningHandler::validPointPotential(const geometry_msgs::Point& world_point, double tolerance){
//  if(!initialized_){
//    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
//    return false;
//  }
//
//  double resolution = costmap_->getResolution();
//  geometry_msgs::Point p;
//  p = world_point;
//
//  p.y = world_point.y - tolerance;
//
//  while(p.y <= world_point.y + tolerance){
//    p.x = world_point.x - tolerance;
//    while(p.x <= world_point.x + tolerance){
//      double potential = getPointPotential(p);
//      if(potential < POT_HIGH){
//        return true;
//      }
//      p.x += resolution;
//    }
//    p.y += resolution;
//  }
//
//  return false;
//}

float GlobalPlanningHandler::getPointPotential(const geometry_msgs::Point& world_point)
{
  if(!mb_initialized){
    //ROS_ERROR("GPH has not been initialized yet, but it is being used, please call initialize() before use");
    return -1.0;
  }

  unsigned int mx, my;
  if(!m_costmap.worldToMap(world_point.x, world_point.y, mx, my))
    return DBL_MAX;

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}
//
//bool GlobalPlanningHandler::computePotential(const geometry_msgs::Point& world_point){
//  if(!initialized_){
//    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
//    return false;
//  }
//
//  //make sure to resize the underlying array that Navfn uses
//  planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
//  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);
//
//  unsigned int mx, my;
//  if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
//    return false;
//
//  int map_start[2];
//  map_start[0] = 0;
//  map_start[1] = 0;
//
//  int map_goal[2];
//  map_goal[0] = mx;
//  map_goal[1] = my;
//
//  planner_->setStart(map_start);
//  planner_->setGoal(map_goal);
//
//  return planner_->calcNavFnDijkstra();
//}

void GlobalPlanningHandler::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my)
{
  if(!mb_initialized)
  {
    //ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  //set the associated costs in the cost map to be free
  m_costmap.setCost(mx, my, costmap_2d::FREE_SPACE);
  //ROS_WARN("\n\n\n\n allow unknown %d\n\n\n\n", allow_unknown_);
}


bool GlobalPlanningHandler::makePlan( const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal,
		  	  std::vector<geometry_msgs::PoseStamped>& plan )
{

//    if(!planner_->makePlan(start, goal, global_plan) || global_plan.empty())
//    {
//    	return false ;
//    }
//    else
//    {
//    	return true ;
//    }

//    boost::mutex::scoped_lock lock(m_mutex);
    if(!mb_initialized)
    {
      //ROS_ERROR("@GPH: This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
ROS_WARN("GlobalPlanningHandler::makePlan() is called to find a plan from (%f %f) to the goal (%f %f) \n",
		start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y );
    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_)
    {
      //ROS_ERROR("@GPH: The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
      //          global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if(start.header.frame_id != global_frame_)
    {
      //ROS_ERROR("@GPH: The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
      //          global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!m_costmap.worldToMap(wx, wy, mx, my))
    {
      //ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
//ROS_INFO("clearing robot cell \n");
    clearRobotCell(start, mx, my);
    //make sure to resize the underlying array that Navfn uses
//ROS_INFO("setting planner nav arr w/ cellsizes: %d %d\n",mp_costmap->getSizeInCellsX(), mp_costmap->getSizeInCellsY());
    planner_->setNavArr(m_costmap.getSizeInCellsX(), m_costmap.getSizeInCellsY());
//ROS_INFO("setting planner costmap \n");
    planner_->setCostmap(m_costmap.getCharMap(), true, mb_allow_unknown);
//costmap_->saveMap("/home/hankm/catkin_ws/src/frontier_detector/launch/cstmap.dat");

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!m_costmap.worldToMap(wx, wy, mx, my))
    {
      if(mf_tolerance <= 0.0)
      {
        //ROS_WARN_THROTTLE(1.0, "The goal sent to the global_planning_handler is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
//ROS_WARN("wx wy (%f %f) mx my (%u %u) mapstart(%d %d) mapgoal(%d %d):\n",
//		wx, wy, mx, my, map_start[0], map_start[1], map_goal[0], map_goal[1] );

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    bool success = planner_->calcNavFnAstar();
    //bool success = planner_->calcNavFnDijkstra(true);

    if(success)
    {
		//planner_->calcPath(mp_costmap->getSizeInCellsX() * 4);

		//extract the plan
		float *x = planner_->getPathX();
		float *y = planner_->getPathY();
		int len = planner_->getPathLen();
		ros::Time plan_time = ros::Time::now();

		for(int i = len - 1; i >= 0; --i)
		{
		  //convert the plan to world coordinates
		  double world_x, world_y;
		  mapToWorld(x[i], y[i], world_x, world_y);

		  geometry_msgs::PoseStamped pose;
		  pose.header.stamp = plan_time;
		  pose.header.frame_id = global_frame_;
		  pose.pose.position.x = world_x;
		  pose.pose.position.y = world_y;
		  pose.pose.position.z = 0.0;
		  pose.pose.orientation.x = 0.0;
		  pose.pose.orientation.y = 0.0;
		  pose.pose.orientation.z = 0.0;
		  pose.pose.orientation.w = 1.0;
		  plan.push_back(pose);
		}
		//ROS_INFO("GPH has found a legal plan with %d length \n", plan.size() );
		return true;
    }
    else
    {
    	//ROS_ERROR("@GPH: Failed to get a plan from the Astar search");
    	return false;
    }

}

int GlobalPlanningHandler::makePlan( const int& tid, const float& fbound, const bool& boneqgrid,
			  const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
		  	  std::vector<geometry_msgs::PoseStamped>& plan, float& fendpotential )
{
// does makePlan(), but breaks when f(n) > ubound occurs.
// we don't need such path since f(n') >= f(n) which is the consistency property of Euclidean heuristic.

    if(!mb_initialized)
    {
      ROS_ERROR("@GPH: This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return 0;
    }

//ROS_WARN("GlobalPlanningHandler::makePlan() is called to find a plan from (%f %f) to the goal (%f %f) \n",
//		start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y );
    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_)
    {
      ROS_ERROR("@GPH: The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return 0;
    }

    if(start.header.frame_id != global_frame_)
    {
      ROS_ERROR("@GPH: The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                global_frame_.c_str(), start.header.frame_id.c_str());
      return 0;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int mx, my;
    if(!m_costmap.worldToMap(wx, wy, mx, my))
    {
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return 0;
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
ROS_DEBUG("[tid %d] clearing robot cell \n", tid);
    clearRobotCell(start, mx, my);
    //make sure to resize the underlying array that Navfn uses
ROS_DEBUG("[tid %d] setting planner nav arr w/ cellsizes: %d %d\n",m_costmap.getSizeInCellsX(), m_costmap.getSizeInCellsY(), tid);
    planner_->setNavArr(m_costmap.getSizeInCellsX(), m_costmap.getSizeInCellsY());

	if(boneqgrid)
	{
    	planner_->setEqGridCostmap(m_costmap.getCharMap(), mb_allow_unknown);
	}
	else
	{
    	planner_->setCostmap(m_costmap.getCharMap(), true, mb_allow_unknown);
	}
//costmap_->saveMap("/home/hankm/catkin_ws/src/frontier_detector/launch/cstmap.dat");

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if(!m_costmap.worldToMap(wx, wy, mx, my))
    {
      if(mf_tolerance <= 0.0)
      {
       // ROS_WARN_THROTTLE(1.0, "The goal sent to the global_planning_handler is off the global costmap. Planning will always fail to this goal.");
        return 0;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
//ROS_WARN("wx wy (%f %f) mx my (%u %u) mapstart(%d %d) mapgoal(%d %d):\n",
//		wx, wy, mx, my, map_start[0], map_start[1], map_goal[0], map_goal[1] );

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    //bool success = planner_->calcNavFnAstar(   );
    //bool success = planner_->calcNavFnDijkstra(true);

    // -1 : currnode > upperbound
    // 1 : last node is open
    // -3: last node is open but invalid plan
    int success = planner_->calcNavFnBoundedAstar( tid, fbound, fendpotential ); // -3, -1, 1
    if(success < 0)
    	return success;

	double resolution = m_costmap.getResolution();
	geometry_msgs::PoseStamped p, best_pose;
	p = goal;

	bool found_legal = false;
	double best_sdist = DBL_MAX;

	p.pose.position.y = goal.pose.position.y - mf_tolerance;

	while(p.pose.position.y <= goal.pose.position.y + mf_tolerance)
	{
	  p.pose.position.x = goal.pose.position.x - mf_tolerance;
	  while(p.pose.position.x <= goal.pose.position.x + mf_tolerance)
	  {
		double potential = getPointPotential(p.pose.position);
		double sdist = sq_distance(p, goal);
		if(potential < POT_HIGH && sdist < best_sdist)
		{
		  best_sdist = sdist;
		  best_pose = p;
		  found_legal = true;
		}
		p.pose.position.x += resolution;
	  }
	  p.pose.position.y += resolution;
	}

	if(found_legal)
	{
	  //extract the plan
	  if(getPlanFromPotential(best_pose, plan))
	  {
		//make sure the goal we push on has the same timestamp as the rest of the plan
		geometry_msgs::PoseStamped goal_copy = best_pose;
		goal_copy.header.stamp = ros::Time::now();
		plan.push_back(goal_copy);

		planner_->writeAstarPlan(plan);

		//ROS_INFO("GPH has found a legal plan with %d length \n", plan.size() );
		if(fendpotential < fbound )
		{
			return 2; //!plan.empty();
		}
		else
		{
			return 1;
		}
	  }
	  else
	  {
		//ROS_ERROR("@GPH: Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
		return -6;
	  }
	}
	else
	{
		return -7;
	}

}


bool GlobalPlanningHandler::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(!mb_initialized)
  {
    //ROS_ERROR("GPH has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  //clear the plan, just in case
  plan.clear();

  //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
  if(goal.header.frame_id != global_frame_){
    //ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
     //         global_frame_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  double wx = goal.pose.position.x;
  double wy = goal.pose.position.y;

  //the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if(!m_costmap.worldToMap(wx, wy, mx, my))
  {
    //ROS_WARN_THROTTLE(1.0, "The goal sent to the GPH is off the global costmap. Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  planner_->calcPath(m_costmap.getSizeInCellsX() * 4);

  //extract the plan
  float *x = planner_->getPathX();
  float *y = planner_->getPathY();
  int len = planner_->getPathLen();
  ros::Time plan_time = ros::Time::now();

  for(int i = len - 1; i >= 0; --i){
    //convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  //publish the plan for visualization purposes
  //publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
  return !plan.empty();
}

void GlobalPlanningHandler::mapToWorld(double mx, double my, double& wx, double& wy)
{
  wx = m_costmap.getOriginX() + mx * m_costmap.getResolution();
  wy = m_costmap.getOriginY() + my * m_costmap.getResolution();
}


//void GlobalPlanningHandler::setCostmap( unsigned char* cmap, unsigned int size_x, unsigned int size_y, float resolution, float origin_x, float origin_y)
//{
//	mp_costmap->resizeMap(size_x, size_y, resolution, origin_x, origin_y);
//	unsigned char* pmap = mp_costmap->getCharMap() ;
//
//	memcpy( mp_costmap->getCharMap(), cmap, sizeof(uint8_t) * size_x * size_y );
//}

void GlobalPlanningHandler::setCostmap( vector<signed char> cmap, unsigned int size_x, unsigned int size_y, float resolution, float origin_x, float origin_y)
{
	m_costmap.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
	unsigned char* pmap = m_costmap.getCharMap() ;

//ROS_INFO(" resetting %d %d sized map\n ", size_x, size_y );
	for(uint32_t ridx = 0; ridx < size_y; ridx++)
	{
		for(uint32_t cidx=0; cidx < size_x; cidx++)
		{
			uint32_t idx = ridx * size_x + cidx ;
			int8_t val = (int8_t)cmap[idx];
			pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
		}
	}
}

}
