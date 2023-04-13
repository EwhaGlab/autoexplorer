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

#ifndef INCLUDE_FRONTIER_DETECTOR_DMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_DMS_HPP_

#include "frontier_detector.hpp"
#include "frontier_point.hpp"
#include "frontier_filter.hpp"
#include "global_planning_handler.hpp"
#include <omp.h>
#include <algorithm>
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <algorithm>
#include <random>
#include <unordered_set>
#include <boost/format.hpp>

//#define OCCUPANCY_THR (60)
//#define DEBUG_MODE
#define ROI_OFFSET (10)
#define DIST_HIGH  (1.0e10)
#define FRONTIER_MARKER_SIZE (0.4)
#define TARGET_MARKER_SIZE (0.5)
#define UNREACHABLE_MARKER_SIZE (0.4)


namespace autoexplorer
{

enum PrevExpState {
  ABORTED = 0,
  SUCCEEDED
};
static const char *prevstate_str[] =
        { "ABORTED", "SUCCEEDED" };


using namespace std;

class FrontierDetectorDMS: public FrontierDetector
{
public:
	FrontierDetectorDMS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~FrontierDetectorDMS();

	void initmotion( const float& fvx, const float& fvy, const float& ftheta );
	inline void SetInitMotionCompleted(){ mb_isinitmotion_completed = true;  }
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

	void publishFrontierPoints() ;
	void publishFrontierPointMarkers( ) ;
	void publishFrontierRegionMarkers( const visualization_msgs::Marker& vizfrontier_regions  );
	void publishGoalPointMarker(  const geometry_msgs::PoseWithCovarianceStamped& targetgoal );
	void publishUnreachbleMarker( const geometry_msgs::PoseStamped& unreachablepose );
	void publishUnreachableMarkers( ); // const geometry_msgs::PoseStamped& unreachablepose );
	void appendUnreachablePoint( const geometry_msgs::PoseStamped& unreachablepose ) ;

	void updatePrevFrontierPointsList( );
//	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
//	geometry_msgs::PoseStamped GetCurrPose ( ) ;

	cv::Point world2gridmap( cv::Point2f img_pt_roi );
	cv::Point2f gridmap2world( cv::Point grid_pt );

	int saveMap( const nav_msgs::OccupancyGrid& map, const string& infofilename, const string& mapfilename ) ;
	int saveFrontierPoints( const nav_msgs::OccupancyGrid& map, const nav_msgs::Path& msg_frontiers, int bestidx, const string& frontierfile  ) ;
	int savefrontiercands( const nav_msgs::OccupancyGrid& map, const vector<FrontierPoint>& voFrontierPoints, const string& frontierfile ) ;

	int frontier_summary( const vector<FrontierPoint>& voFrontierCurrFrame );

	void updateUnreachablePointSet( const nav_msgs::OccupancyGrid& globalcostmap  ) ;

	int selectNextBestPoint( const geometry_msgs::PoseStamped& robotpose, const nav_msgs::Path& goalexclusivefpts, geometry_msgs::PoseStamped& nextbestpoint  ) ;
	int selectEscapingPoint( geometry_msgs::PoseStamped& escapepoint) ;
	int moveBackWard() ;

	int moveToHome() ;

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

	inline bool frontier_sanity_check( int nx, int ny, int nwidth, const std::vector<signed char>& cmdata )
	{
		// 0	1	2
		// 3	4	5
		// 6	7	8

		int i0 = nwidth * (ny - 1) 	+	nx - 1 ;
		int i1 = nwidth * (ny - 1) 	+	nx 		;
		int i2 = nwidth * (ny - 1) 	+	nx + 1	;
		int i3 = nwidth * ny			+	nx - 1 ;
		int i5 = nwidth * ny			+	nx + 1 ;
		int i4 = nwidth * ny			+ 	nx ;
		int i6 = nwidth * (ny + 1)	+	nx - 1 ;
		int i7 = nwidth * (ny + 1)	+	nx		;
		int i8 = nwidth * (ny + 1)	+	nx + 1 ;
		int nleathalcost = 70 ;

		//ROS_INFO("width i0 val : %d %d %d\n", nwidth, i0, gmdata[i0] );
		if( cmdata[i0] > nleathalcost || cmdata[i1] > nleathalcost || cmdata[i2] > nleathalcost || cmdata[i3] > nleathalcost ||
			cmdata[i5] > nleathalcost || cmdata[i6] > nleathalcost || cmdata[i7] > nleathalcost || cmdata[i8] > nleathalcost )
		{
			return false;
		}

		if( cmdata[i4] < 0 )
		{
			return true ;
		}
		else
		{
			return false ;
		}
	}

	inline bool is_explored( int nx, int ny, int nwidth, const std::vector<signed char>& gmdata )
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

		if( gmdata[i0] < 0 || gmdata[i1] < 0 || gmdata[i2] < 0 || gmdata[i3] < 0 ||  gmdata[i5] < 0 || gmdata[i6] < 0 || gmdata[i7] < 0 || gmdata[i8] < 0 )
		{
			return false ;
		}
		else
		{
			return true ;
		}
	}


    inline bool equals_to_prevgoal( const geometry_msgs::PoseStamped& in_goal )
    {
//    	  ROS_WARN("prev/curr goal (%f %f), (%f %f) \n",
//    			  previous_goal_.pose.position.x, previous_goal_.pose.position.y,
//				  planner_goal_.pose.position.x, planner_goal_.pose.position.y
//    	  );

  	  float fxdiff = (m_previous_goal.pose.pose.position.x - in_goal.pose.position.x) ;
  	  float fydiff = (m_previous_goal.pose.pose.position.y - in_goal.pose.position.y) ;
  	  return std::sqrt( fxdiff * fxdiff + fydiff * fydiff ) < 0.001 ? true : false;
    }

    inline void world_to_scaled_gridmap( float fwx, float fwy, float fox, float foy, float fres, int nscale, int& nmx, int& nmy )
    {
    	int ngmx = static_cast<int>( (fwx - fox) / fres ) ;
    	int ngmy = static_cast<int>( (fwy - foy) / fres ) ;

    	nmx = ngmx / nscale ;
    	nmy = ngmy / nscale ;
    }
    
    static float euc_dist(const cv::Point2f& lhs, const cv::Point2f& rhs)
    {
        return (float)sqrt(  (lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)  );
    }

protected:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber 	m_mapSub, m_poseSub, m_velSub, m_mapframedataSub, m_globalCostmapSub, m_globalCostmapUpdateSub, m_frontierCandSub,
						m_currGoalSub, m_globalplanSub, m_unreachablefrontierSub ;
	ros::Publisher 		m_targetsPub, m_markercandPub, m_markerfrontierPub, m_markerfrontierregionPub, m_makergoalPub,
						m_currentgoalPub, m_marker_unreachpointPub, m_unreachpointPub, m_velPub, m_donePub, m_resetgazeboPub, m_startmsgPub,
						m_otherfrontierptsPub ;

	int32_t mn_FrontierID, mn_UnreachableFptID ;

	int mn_numthreads;
	int mn_globalcostmapidx ;
	string mstr_debugpath ;
	string mstr_inputparams ;
	bool mb_isinitmotion_completed ;

	cv::Mat mcvu_mapimg, mcvu_mapimgroi ;

	FrontierFilter mo_frontierfilter;
	tf::TransformListener m_listener;

	//GlobalPlanningHandler* mpo_gph ;
	GlobalPlanningHandler mo_gph ;
	costmap_2d::Costmap2D* mpo_costmap;
	bool mb_allow_unknown ;

	uint8_t* mp_cost_translation_table;

	ofstream m_ofs_time ;
	float mf_neighoringpt_decisionbound ;
	bool mb_strict_unreachable_decision ;

	geometry_msgs::PoseWithCovarianceStamped m_previous_goal ;
	PrevExpState me_prev_exploration_state ;
	//int mn_prev_nbv_posidx ;
	bool mb_nbv_selected ;
	ros::Time m_last_oscillation_reset ;
	geometry_msgs::PoseStamped m_previous_robot_pose, m_home_pose ;

	bool mb_return_home ;

private:
	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_frontier_set;
	std::mutex mutex_curr_frontier_set ;
	std::mutex mutex_prev_frontier_set ;

	std::mutex mutex_gridmap;
	std::mutex mutex_costmap;
	std::mutex mutex_upperbound;
	std::mutex mutex_timing_profile;
	std::mutex mutex_currgoal ;

	omp_lock_t m_mplock;
    
    // for debug
	int mn_mapcallcnt ;
	double mf_totalcallbacktime_msec, mf_totalplanningtime_msec ;
};

}




#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
