/*
 * frontier_detector_dms.hpp
 *
 *  Created on: Sep 25, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_FRONTIER_DETECTOR_DMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_DMS_HPP_

#include "frontier_detector.hpp"
#include "frontier_point.hpp"
#include "frontier_filter.hpp"
#include "global_planning_handler.hpp"
#include <omp.h>

//#define OCCUPANCY_THR (60)
//#define FD_DEBUG_MODE
#define ROI_OFFSET (10)

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

	void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
	void globalCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
	void robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg);
	void doneCB( const actionlib::SimpleClientGoalState& state ) ;

	void mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //const octomap_server::mapframedata& msg ) ;
	//virtual void moveRobotCallback(const nav_msgs::Path::ConstPtr& msg ) ;
	void moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg );

	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );
	bool isDone() const { return isdone; };
	void publishDone() ;

//	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
//	geometry_msgs::PoseStamped GetCurrPose ( ) ;

	cv::Point world2gridmap( cv::Point2f img_pt_roi );
	cv::Point2f gridmap2world( cv::Point grid_pt );

protected:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber m_mapsub, m_poseSub, m_velSub, m_mapframedataSub, m_globalCostmapSub, m_globalCostmapUpdateSub,
					m_globalplanSub, m_unreachablefrontierSub ;
	ros::Publisher m_targetspub, m_markercandpub, m_markerfrontierpub,
					m_makergoalpub, m_currentgoalpub, m_unreachpointpub, m_velpub, m_donepub ;

	int m_nglobalcostmapidx ;
	string m_str_debugpath ;
	string m_str_inputparams ;
	bool m_isInitMotionCompleted ;

	cv::Mat m_uMapImg, m_uMapImgROI ;

	FrontierFilter m_oFrontierFilter;

	GlobalPlanningHandler* mpo_gph ;
	GlobalPlanningHandler mo_gph ;
	costmap_2d::Costmap2D* mpo_costmap;

	uint8_t* mp_cost_translation_table;

	ofstream m_ofs_time ;
private:
	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_gridmap;
	std::mutex mutex_costmap;
};

}




#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
