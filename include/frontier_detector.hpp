/*
 * frontier_detector.hpp
 *
 *  Created on: Sep 25, 2021
 *      Author: hankm
 */

#ifndef INCLUDE_FRONTIER_DETECTOR_HPP_
#define INCLUDE_FRONTIER_DETECTOR_HPP_


#define OCCUPIED_BIN_THR 	(128)
#define WEAK_COMPONENT_THR	(10)
#define MIN_TARGET_DIST		(30) // to prevent relocating the current pose as the frontier point over and over again
#define LEATHAL_COST_THR	(80)

//#define FFD_DEBUG_MODE
#include <ros/console.h>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "octomap_server/mapframedata.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include <tf/transform_listener.h>

#include "ros/service_client.h"
#include "nav_msgs/GetPlan.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ffp.hpp"
#include <experimental/filesystem>
#include <set>

// global path planner
//#include "navigation/costmap_2d/include/costmap_2d/costmap_2d.h"
//#include "navigation/global_planner/include/global_planner/astar.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> SimpleMoveBaseClient ;
//typedef actionlib::ActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient ;


typedef struct _FrontierInfo {
	cv::Point location ;
	int revenue ;
}FrontierInfo ;

namespace frontier_detector
{

using namespace std;

struct pointset
{
  float d[2];

  bool operator()( const pointset & dia, const pointset & dib) const
  {
    for (size_t n=0; n<2; ++n)
    {
      if ( dia.d[n] < dib.d[n] ) return true;
      if ( dia.d[n] > dib.d[n] ) return false;
    }
    return false;
  }
};

typedef enum{	ROBOT_IS_NOT_MOVING 	= -1,
				ROBOT_IS_READY_TO_MOVE	= 0,
				FORCE_TO_STOP    		= 1, // is moving but needs to be stopped
				ROBOT_IS_MOVING  		= 2
			} ROBOT_STATE ;

class FrontierDetector
{
public:
	FrontierDetector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_);
	virtual ~FrontierDetector();

	void initmotion( );

	virtual void gridmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
	virtual void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
	virtual void robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	virtual void robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg);
	virtual void doneCB( const actionlib::SimpleClientGoalState& state ) ;
	virtual void mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); //const octomap_server::mapframedata& msg ) ;
	//virtual void moveRobotCallback(const nav_msgs::Path::ConstPtr& msg ) ;
	virtual void moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg ) ;
	virtual void unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg );

	float Norm(cv::Point2f x1, cv::Point2f x2);
	int8_t  gridValue(nav_msgs::OccupancyGrid &mapData, cv::Point2f Xp);
	int  costmapValue( nav_msgs::OccupancyGrid &mapData, cv::Point2f Xp );

	vector<cv::Point> eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize = 25 ) ;
	int displayMapAndFrontiers(const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize ) ;
	bool isValidPlan( vector<cv::Point>  );
	bool isDone(){ return isdone; };
	void publishDone() ;

	vector<FrontierInfo> assessFrontiers( vector<cv::Point> frontierCandidates ) ;
	int  computeRevenue( nav_msgs::OccupancyGrid &mapData, cv::Point2f Xp, int winsize = 10 );

	cv::Point estimateFrontierPointPose( vector<cv::Point> frontier_contour, cv::Point frontier_point );

	geometry_msgs::PoseStamped StampedPosefromSE2( float x, float y, float yaw ) ;
	geometry_msgs::PoseStamped GetCurrPose ( ) ;


	void downSampleMap( cv::Mat& uImage )
	{
		// Labeling is a bit weird but works OK with this way.
		// unknown 255, occupied 127, free 0

		cv::Mat uOccu = uImage.clone();
		cv::threshold( uOccu, uOccu, 187, 255, cv::THRESH_TOZERO_INV ); 	// 187 ~ 255 --> 0
		cv::threshold( uOccu, uOccu, 67,  255, cv::THRESH_TOZERO ); 		// 0 ~ 66 	--> 0
		cv::threshold( uOccu, uOccu, 0, 255, cv::THRESH_BINARY) ;// 67 ~ 187  --> 255 (occupied)

		cv::Mat uUnkn = uImage.clone();
		cv::threshold( uUnkn, uUnkn, 187, ffp::MapStatus::UNKNOWN, cv::THRESH_BINARY ); // 187 ~ 255 --> 255

		for(int iter=0; iter < m_nNumPyrDownSample; iter++ )
		{
			pyrDown(uOccu, uOccu, cv::Size(uOccu.rows/2, uOccu.cols/2));
			pyrDown(uUnkn, uUnkn, cv::Size(uUnkn.rows/2, uUnkn.cols/2));
		}

		cv::threshold(uOccu, uOccu, 0, ffp::MapStatus::OCCUPIED, CV_THRESH_BINARY) ;
		cv::threshold(uUnkn, uUnkn, 0, ffp::MapStatus::UNKNOWN, CV_THRESH_BINARY) ;
		uImage = uOccu + uUnkn ;
	}

	void clusterToThreeLabels( cv::Mat& uImage  )
	{
		cv::Mat uUnkn = uImage.clone();
		cv::threshold( uUnkn, uUnkn, 187, 255, cv::THRESH_TOZERO_INV ); 	// 187 ~ 255 --> 0
		cv::threshold( uUnkn, uUnkn, 67,  255, cv::THRESH_TOZERO ); 		// 0 ~ 66 	--> 0
		cv::threshold( uUnkn, uUnkn, 0, ffp::MapStatus::UNKNOWN, cv::THRESH_BINARY) ;// 67 ~ 187  --> 127 (unknown)

		cv::Mat uOcc = uImage.clone();
		cv::threshold( uOcc, uOcc, 128, ffp::MapStatus::OCCUPIED, cv::THRESH_BINARY ); // 187 ~ 255 --> 255
		uImage = uOcc + uUnkn ;
#ifdef SAVE_DEBUG_IMAGES
		cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/uImage.png",  uImage);
		cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/occ.png",  uOcc);
		cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/unkn.png", uUnkn);
#endif
	}

private:

	ros::Subscriber m_mapsub, m_poseSub, m_velSub, m_mapframedataSub, m_globalCostmapSub, m_globalplanSub, m_unreachablefrontierSub ;
	ros::Publisher m_targetspub, m_markercandpub, m_markerfrontierpub,
					m_makergoalpub, m_currentgoalpub, m_unreachpointpub, m_velpub, m_donepub ;
	//vector<geometry_msgs::PoseWithCovarianceStamped> m_exploration_goal ;
	//geometry_msgs::PoseWithCovarianceStamped  ;
	visualization_msgs::Marker m_points, m_cands, m_exploration_goal, m_unreachable_points ;
	nav_msgs::OccupancyGrid m_gridmap;
	nav_msgs::OccupancyGrid m_globalcostmap ;
	nav_msgs::Path			m_pathplan ;

	geometry_msgs::PoseWithCovarianceStamped m_robotpose ; // (w.r.t world)
	geometry_msgs::Twist m_robotvel ;

	std::string m_worldFrameId;
	std::string m_mapFrameId;
	std::string m_baseFrameId ;

	int m_nNumPyrDownSample;
	int m_nScale;
	int m_nROISize ;
	int m_nGridMapWidth, m_nGridMapHeight ;
	int m_nCannotFindFrontierCount ;
	bool isdone ;
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_private;

	//tf::TransformListener m_tfListener;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_move_client ;
	//actionlib::ActionClient<move_base_msgs::MoveBaseAction> m_move_client ;
	ros::ServiceClient m_makeplan_client;

	cv::Mat m_uMapImgOcc, m_uMapImgUnk, m_uMapImg_buf, m_uMapImg ;
	cv::Mat m_matCostMap ;
	//cv::Mat m_morph_kernel ;

	cv::Point2f img2gridmap( cv::Point img_pt );
	cv::Point gridmap2img( cv::Point2f grid_pt );

	vector<cv::Point> m_conquered_frontiers;
	vector<cv::Point> m_frontiers;

	int m_frontiers_region_thr ;
	int m_globalcostmap_rows ;
	int m_globalcostmap_cols ;
	ROBOT_STATE m_eRobotState ;

	geometry_msgs::PoseWithCovarianceStamped m_bestgoal ;
	set<pointset, pointset> m_unreachable_frontier_set ;
	cv::FileStorage m_fs;

	// thrs
	float m_frontier_cost_thr ;

	std::mutex mutex_robot_state;
	std::mutex mutex_unreachable_points;
	std::mutex mutex_gridmap_image;
};

}





#endif /* INCLUDE_FRONTIER_DETECTOR_HPP_ */
