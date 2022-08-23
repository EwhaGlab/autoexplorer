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

#ifndef INCLUDE_FRONTIER_DETECTOR_DMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_DMS_HPP_

#include "frontier_detector.hpp"
#include "frontier_point.hpp"
#include "frontier_filter.hpp"
#include "global_planning_handler.hpp"
#include <omp.h>
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <algorithm>
#include <random>
#include <unordered_set>

//#define OCCUPANCY_THR (60)
//#define FD_DEBUG_MODE
#define ROI_OFFSET (10)
#define DIST_HIGH  (1.0e10)

namespace autoexplorer
{

using namespace std;

class FrontierDetectorDMS: public FrontierDetector
{
public:
	FrontierDetectorDMS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~FrontierDetectorDMS();

	void initmotion( );
	inline void SetInitMotionCompleted(){ m_isInitMotionCompleted = true;  }
	inline void SetNumThreads(int numthreads){ mn_numthreads = numthreads; }

	void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
	void globalCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
	void robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg);
	void doneCB( const actionlib::SimpleClientGoalState& state ) ;

	void mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //const octomap_server::mapframedata& msg ) ;
	void gobalPlanCallback(const visualization_msgs::Marker::ConstPtr& msg) ;
	//virtual void moveRobotCallback(const nav_msgs::Path::ConstPtr& msg ) ;
	void moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg );

	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );
	bool explorationisdone() const { return mb_explorationisdone; };
	void publishDoneExploration() ;
	void publishResetGazebo() ;

//	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
//	geometry_msgs::PoseStamped GetCurrPose ( ) ;

	cv::Point world2gridmap( cv::Point2f img_pt_roi );
	cv::Point2f gridmap2world( cv::Point grid_pt );

	int savegridmap( const nav_msgs::OccupancyGrid& gridmap, const string& filename ) ;
	int savecostmap( const nav_msgs::OccupancyGrid& costmap, const string& filename ) ;

	int frontier_summary( const vector<FrontierPoint>& voFrontierCurrFrame );

	geometry_msgs::PoseStamped GetCurrRobotPose ( )
	{
		tf::StampedTransform map2baselink;
		try{
		  m_listener.lookupTransform(m_worldFrameId, m_baseFrameId,
								   ros::Time(0), map2baselink);
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}

		geometry_msgs::PoseStamped outPose;
		outPose.pose.position.x = map2baselink.getOrigin().x();
		outPose.pose.position.y = map2baselink.getOrigin().y();
		outPose.pose.position.z = 0.f;
		outPose.header.frame_id = m_worldFrameId;

		return outPose;
	}

	inline bool frontier_sanity_check( int nx, int ny, int nwidth, const std::vector<signed char>& gmdata )
	{
		// 0	1	2
		// 3		5
		// 6	7	8

		int i0 = nwidth * (ny - 1) 	+	nx - 1 ;
		int i1 = nwidth * (ny - 1) 	+	nx 		;
		int i2 = nwidth * (ny - 1) 	+	nx + 1	;
		int i3 = nwidth * ny			+	nx - 1 ;
		int i5 = nwidth * ny			+	nx + 1 ;
		int i6 = nwidth * (ny + 1)	+	nx - 1 ;
		int i7 = nwidth * (ny + 1)	+	nx		;
		int i8 = nwidth * (ny + 1)	+	nx + 1 ;

		//ROS_INFO("width i0 val : %d %d %d\n", nwidth, i0, gmdata[i0] );

		if( gmdata[i0] < 0 || gmdata[i1] < 0 || gmdata[i2] < 0 || gmdata[i3] < 0 ||  gmdata[i5] < 0 || gmdata[i6] < 0 || gmdata[i7] < 0 || gmdata[i8] < 0 )
		{
			return true ;
		}
		else
		{
			return false ;
		}
	}



//	static bool lexico_compare(const visualization_msgs::Marker& pt1, const visualization_msgs::Marker& pt2)
//	{
//		if(pt1.pose.position.x < pt2.pose.position.x) {return true; }
//		if(pt1.pose.position.x > pt2.pose.position.x) {return false; }
//		return(  pt1.pose.position.y < pt2.pose.position.y  ) ;
//	}
//
//	static bool fpts_are_equal( const visualization_msgs::Marker& pt1, const visualization_msgs::Marker& pt2)
//	{
//		return ( (pt1.pose.position.x == pt2.pose.position.x) && (pt1.pose.position.y == pt2.pose.position.y) ) ;
//	}
//
//	void remove_duplicate_fpts( vector<visualization_msgs::Marker>& points  )
//	{
//		std::sort(points.begin(), points.end(), lexico_compare);
//		points.erase(std::unique(points.begin(), points.end(), fpts_are_equal), points.end());
//	}


protected:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber 	m_mapsub, m_poseSub, m_velSub, m_mapframedataSub, m_globalCostmapSub, m_globalCostmapUpdateSub, m_frontierCandSub,
						m_currGoalSub, m_globalplanSub, m_unreachablefrontierSub ;
	ros::Publisher 	m_targetspub, m_markercandpub, m_markerfrontierpub,
						m_makergoalpub, m_currentgoalpub, m_unreachpointpub, m_velpub, m_donepub, m_resetgazebopub ;

	int32_t mn_FrontierID, mn_UnreachableFptID ;

	int mn_numthreads;
	int m_nglobalcostmapidx ;
	string m_str_debugpath ;
	string m_str_inputparams ;
	bool m_isInitMotionCompleted ;

	cv::Mat m_uMapImg, m_uMapImgROI ;

	FrontierFilter m_oFrontierFilter;
	tf::TransformListener m_listener;

	//GlobalPlanningHandler* mpo_gph ;
	GlobalPlanningHandler mo_gph ;
	costmap_2d::Costmap2D* mpo_costmap;

	uint8_t* mp_cost_translation_table;

	ofstream m_ofs_time ;
	float mf_neighoringpt_decisionbound ;

//	// valid and unique frontier pts list @ the current input frame
//	vector<FrontierPoint> mvo_frontier_list;

private:
	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_curr_frontier_set ;
	std::mutex mutex_prev_frontier_set ;

	std::mutex mutex_gridmap;
	std::mutex mutex_costmap;
	std::mutex mutex_upperbound;
	std::mutex mutex_timing_profile;
	std::mutex mutex_currgoal ;

	omp_lock_t m_mplock;
};

}




#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
