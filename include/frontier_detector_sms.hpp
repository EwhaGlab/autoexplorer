/*
 * frontier_detector_sms.hpp
 *
 *  Created on: Sep 29, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_FRONTIER_DETECTOR_SMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_SMS_HPP_


#include "frontier_detector.hpp"
#include "frontier_point.hpp"
#include "frontier_filter.hpp"

// #define FD_DEBUG_MODE
#define TIMING_ANALYSIS

namespace autoexplorer
{

using namespace std;

//struct pointset
//{
//  float d[2];
//
//  bool operator()( const pointset & dia, const pointset & dib) const
//  {
//    for (size_t n=0; n<2; ++n)
//    {
//      if ( dia.d[n] < dib.d[n] ) return true;
//      if ( dia.d[n] > dib.d[n] ) return false;
//    }
//    return false;
//  }
//};
//
//typedef enum{	ROBOT_IS_NOT_MOVING 	= -1,
//				ROBOT_IS_READY_TO_MOVE	= 0,
//				FORCE_TO_STOP    		= 1, // is moving but needs to be stopped
//				ROBOT_IS_MOVING  		= 2
//			} ROBOT_STATE ;

class FrontierDetectorSMS: public FrontierDetector
{
public:
	FrontierDetectorSMS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_ );
	virtual ~FrontierDetectorSMS();

	void initmotion( );
	inline void SetInitMotionCompleted(){ m_isInitMotionCompleted = true;  }
	//void gridmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
	void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
	void robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg);
	void doneCB( const actionlib::SimpleClientGoalState& state ) ;

	void mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //const octomap_server::mapframedata& msg ) ;
	//virtual void moveRobotCallback(const nav_msgs::Path::ConstPtr& msg ) ;
	void moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	void unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg );

	//vector<cv::Point> eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize = 25 ) ;
	void saveGridmap( string filename, const nav_msgs::OccupancyGrid &mapData );
	void saveFrontierCandidates( string filename, vector<FrontierPoint> voFrontierCandidates );

	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );
	bool isDone() const { return isdone; };
	void publishDone() ;

//	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
//	geometry_msgs::PoseStamped GetCurrPose ( ) ;

	cv::Point2f gridmap2world( cv::Point img_pt_roi );
	cv::Point   world2gridmap( cv::Point2f grid_pt );

protected:

	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	ros::Subscriber m_mapsub, m_poseSub, m_velSub, m_mapframedataSub, m_globalCostmapSub, m_globalplanSub, m_unreachablefrontierSub ;
	ros::Publisher m_targetspub, m_markercandpub, m_markerfrontierpub,
					m_makergoalpub, m_currentgoalpub, m_unreachpointpub, m_velpub, m_donepub ;

	int m_nglobalcostmapidx ;
	string m_str_debugpath ;
	string m_str_inputparams ;
	bool m_isInitMotionCompleted ;

	cv::Mat m_uMapImg, m_uMapImgROI ;

	FrontierFilter m_oFrontierFilter;

	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_gridmap_image;
};

}





#endif /* INCLUDE_FRONTIER_DETECTOR_SMS_HPP_ */
