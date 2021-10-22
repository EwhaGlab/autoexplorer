/*
 * frontier_detector_dms.hpp
 *
 *  Created on: Sep 25, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_FRONTIER_DETECTOR_DMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_DMS_HPP_

#include "frontier_detector.hpp"

//#define OCCUPANCY_THR (60)
//#define FD_DEBUG_MODE

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
	void robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg);
	void doneCB( const actionlib::SimpleClientGoalState& state ) ;

	void mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //const octomap_server::mapframedata& msg ) ;
	//virtual void moveRobotCallback(const nav_msgs::Path::ConstPtr& msg ) ;
	void moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg );

	vector<cv::Point> eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize = 25 ) ;
	void accessFrontierPoint();

	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );
	bool isDone() const { return isdone; };
	void publishDone() ;

	cv::Point estimateFrontierPointPose( vector<cv::Point> frontier_contour, cv::Point frontier_point );

//	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
//	geometry_msgs::PoseStamped GetCurrPose ( ) ;

	cv::Point world2gridmap( cv::Point2f img_pt_roi );
	cv::Point2f gridmap2world( cv::Point grid_pt );

	inline int closestmultiple( const int num, const int sz )
	{
		int mp = static_cast<int>(pow(2,sz)) ;
		int rem = num % mp ;
		return rem == 0 ? num : num + (mp - rem) ;
	}

protected:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber m_mapsub, m_poseSub, m_velSub, m_mapframedataSub, m_globalCostmapSub, m_globalplanSub, m_unreachablefrontierSub ;
	ros::Publisher m_targetspub, m_markercandpub, m_markerfrontierpub,
					m_makergoalpub, m_currentgoalpub, m_unreachpointpub, m_velpub, m_donepub ;

	string m_str_debugpath ;
	string m_str_inputparams ;
	bool m_isInitMotionCompleted ;

	float m_frontier_cost_thr ;
	cv::Mat m_uMapImg, m_uMapImgROI ;

	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_gridmap_image;
};

}




#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
