/*
 * frontier_detector_sms.hpp
 *
 *  Created on: Sep 29, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_FRONTIER_DETECTOR_SMS_HPP_
#define INCLUDE_FRONTIER_DETECTOR_SMS_HPP_


#include "frontier_detector.hpp"

//#define OCCUPANCY_THR (60)
// #define FD_DEBUG_MODE

namespace frontier_detector
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

	vector<cv::Point> eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize = 25 ) ;
	void accessFrontierPoint();

	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );
	bool isDone() const { return isdone; };
	void publishDone() ;

//	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
//	geometry_msgs::PoseStamped GetCurrPose ( ) ;

	cv::Point2f img2gridmap( cv::Point img_pt_roi );
	cv::Point gridmap2img( cv::Point2f grid_pt );

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


//	visualization_msgs::Marker m_points, m_cands, m_exploration_goal, m_unreachable_points ;
//	nav_msgs::OccupancyGrid m_gridmap;
//	nav_msgs::OccupancyGrid m_globalcostmap ;
//	nav_msgs::Path			m_pathplan ;
//
//	geometry_msgs::PoseWithCovarianceStamped m_robotpose ; // (w.r.t world)
//	geometry_msgs::Twist m_robotvel ;
//
//	std::string m_worldFrameId;
//	std::string m_mapFrameId;
//	std::string m_baseFrameId ;
//
//	int m_ncols, m_nrows, m_nroi_origx, m_nroi_origy ;
//	//int m_nCannotFindFrontierCount ;
//	bool isdone ;
//
//	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_move_client ;
//	ros::ServiceClient m_makeplan_client;
//	cv::Mat m_uMapImg, m_uMapImgROI ;
//
//	geometry_msgs::PoseWithCovarianceStamped m_bestgoal ;
//	set<pointset, pointset> m_unreachable_frontier_set ;
//	cv::FileStorage m_fs;


//	// thrs
//	float  m_frontier_cost_thr ;
//	int	m_noccupancy_thr ; // 0 ~ 100
//	int m_nlethal_cost_thr ;
//	double m_fRobotRadius ;

	//	vector<cv::Point> m_frontiers;
	//	int m_frontiers_region_thr ;
	//	int m_globalcostmap_rows ;
	//	int m_globalcostmap_cols ;
	//	ROBOT_STATE m_eRobotState ;

	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_gridmap_image;
};

}





#endif /* INCLUDE_FRONTIER_DETECTOR_SMS_HPP_ */
