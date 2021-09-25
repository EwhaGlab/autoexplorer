/*
 * FrontierDetector.cpp
 *
 *  Created on: Mar 18, 2021
 *      Author: hankm
 */


#include "frontier_detector.hpp"

#define ROI_WINSIZE (10)

namespace frontier_detector
{

FrontierDetector::FrontierDetector(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):

m_nh_private(private_nh_),
m_nh(nh_),
m_worldFrameId("map"), m_baseFrameId("base_link"),
m_globalcostmap_rows(0), m_globalcostmap_cols(0), m_eRobotState(ROBOT_STATE::ROBOT_IS_NOT_MOVING),
m_move_client("move_base", true),
m_nROISize(10), m_nCannotFindFrontierCount(0), isdone(false)
//m_move_client("move_base")
{
ROS_WARN("starting frontier detector instance \n");
	// gridmap generated from octomap is downsampled ( for the efficient RL training purpose )
	//m_nh.param("octomap_server/gridmap/num_downsamples", m_nNumPyrDownSample,0);
	m_nh.param("octomap_server/gridmap/width",  m_nGridMapWidth, 4000 );
	m_nh.param("octomap_server/gridmap/height", m_nGridMapHeight,4000 );

	m_nh.param("num_downsamples", m_nNumPyrDownSample,0);	// num of downsampling for gridmap img for the faster FMM
	m_nh.param("frame_id", m_worldFrameId, m_worldFrameId);

	//ros::param::param("/tesse0/octomap_server/gridmap/num_downsamples", m_nNumPyrDownSample,1);
	m_nScale = pow(2, m_nNumPyrDownSample);
	//m_morph_kernel = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(3,3) );
	m_targetspub = m_nh.advertise<geometry_msgs::PointStamped>("detected_points", 10);
	m_currentgoalpub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("curr_goalpose", 10);
	m_makergoalpub = m_nh.advertise<visualization_msgs::Marker>("curr_goal_shape",10);
	m_markercandpub = m_nh.advertise<visualization_msgs::Marker>("detected_shapes", 10);
	m_markerfrontierpub = m_nh.advertise<visualization_msgs::Marker>("filtered_shapes", 10);
	m_unreachpointpub = m_nh.advertise<visualization_msgs::Marker>("unreachable_shapes", 10);

	m_velpub		= m_nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	m_donepub		= m_nh.advertise<std_msgs::Bool>("move_base_simple/mapping_is_done",1);

	//---------------------------------------------------------------
	//m_mapsub = m_nh.subscribe("map", 1, &FrontierDetector::gridmapCallBack, this);  // "projected_map" if octomap is on
	m_mapframedataSub  	= m_nh.subscribe("map", 1, &FrontierDetector::mapdataCallback, this); // kmHan
	//m_globalplanSub 	= m_nh.subscribe("move_base_node/NavfnROS/plan",1 , &FrontierDetector::moveRobotCallback, this) ; // kmHan
	m_globalplanSub 	= m_nh.subscribe("curr_goalpose",1 , &FrontierDetector::moveRobotCallback, this) ; // kmHan
	m_globalCostmapSub 	= m_nh.subscribe("move_base/global_costmap/costmap", 1, &FrontierDetector::globalCostmapCallBack, this );
	m_poseSub		   	= m_nh.subscribe("pose", 10, &FrontierDetector::robotPoseCallBack, this);
	m_velSub			= m_nh.subscribe("cmd_vel", 10, &FrontierDetector::robotVelCallBack, this);
	m_unreachablefrontierSub = m_nh.subscribe("unreachable_frontier", 1, &FrontierDetector::unreachablefrontierCallback, this);
	m_makeplan_client = m_nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");
ROS_WARN("allocating map buff \n");
	m_uMapImg_buf = cv::Mat::zeros(m_nGridMapHeight, m_nGridMapWidth, CV_8U);
	m_uMapImgOcc  = cv::Mat::zeros(m_nGridMapHeight, m_nGridMapWidth, CV_8U);
	m_uMapImgUnk  = cv::Mat::zeros(m_nGridMapHeight, m_nGridMapWidth, CV_8U);
	m_uMapImg  	  = cv::Mat::zeros(m_nGridMapHeight, m_nGridMapWidth, CV_8U);

	char dotrosdir[256];
	getcwd(dotrosdir, 256);
	string yamlfile = string(dotrosdir) + "/../catkin_ws/src/frontier_detector/param/frontier_detection.yaml";
//ROS_INFO("pwd: %s", yamlfile.c_str() );
	m_fs = cv::FileStorage(yamlfile.c_str(), cv::FileStorage::READ);
	m_frontier_cost_thr = m_fs["FRONTIER_COST_THR"];
//	ROS_ERROR("fcost: %f  \n", m_frontier_cost_thr) ;

	while(!m_move_client.waitForServer(ros::Duration(5.0)))
	//while(!m_move_client.waitForActionServerToStart())
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
ROS_WARN("move_base action server is up");

	m_frontiers_region_thr = (WEAK_COMPONENT_THR) / m_nScale ;

	m_cands.header.frame_id= m_worldFrameId;
	m_cands.header.stamp=ros::Time(0);
	m_cands.ns= "markers";
	m_cands.id = 0;
	m_cands.type = m_cands.POINTS;

	m_cands.action = m_cands.ADD;
	m_cands.pose.orientation.w =1.0;
	m_cands.scale.x=0.2;
	m_cands.scale.y=0.2;

	m_cands.color.r = 255.0/255.0;
	m_cands.color.g = 0.0/255.0;
	m_cands.color.b = 0.0/255.0;
	m_cands.color.a=1.0;
	m_cands.lifetime = ros::Duration();

	m_points.header.frame_id= m_worldFrameId;
	m_points.header.stamp=ros::Time(0);
	m_points.ns= "markers";
	m_points.id = 0;
	m_points.type = m_points.POINTS;

	//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	m_points.action = m_points.ADD;
	m_points.pose.orientation.w =1.0;
	m_points.scale.x= 0.3;
	m_points.scale.y= 0.3;

	m_points.color.r = 0.0/255.0;
	m_points.color.g = 255.0/255.0;
	m_points.color.b = 0.0/255.0;
	m_points.color.a = 1.0;
	m_points.lifetime = ros::Duration();

	// set the goal marker
	m_exploration_goal.header.frame_id= m_worldFrameId;
	m_exploration_goal.header.stamp=ros::Time(0);
	m_exploration_goal.ns= "markers";
	m_exploration_goal.id = 0;
	m_exploration_goal.type = m_exploration_goal.POINTS;

	m_exploration_goal.action = m_exploration_goal.ADD;
	m_exploration_goal.pose.orientation.w =1.0;
	m_exploration_goal.scale.x=0.5;
	m_exploration_goal.scale.y=0.5;
	m_exploration_goal.color.r = 255.0/255.0;
	m_exploration_goal.color.g = 0.0/255.0;
	m_exploration_goal.color.b = 255.0/255.0;
	m_exploration_goal.color.a = 1.0;
	m_exploration_goal.lifetime = ros::Duration();

	// set invalid frontier marker
	m_unreachable_points.header.frame_id= m_worldFrameId;
	m_unreachable_points.header.stamp=ros::Time(0);
	m_unreachable_points.ns= "markers";
	m_unreachable_points.id = 0;
	m_unreachable_points.type = m_unreachable_points.POINTS;

	m_unreachable_points.action = m_unreachable_points.ADD;
	m_unreachable_points.pose.orientation.w =1.0;
	m_unreachable_points.scale.x=0.5;
	m_unreachable_points.scale.y=0.5;
	m_unreachable_points.color.r = 1.0;
	m_unreachable_points.color.g = 1.0;
	m_unreachable_points.color.b = 0.0; //255.0/255.0;
	m_unreachable_points.color.a = 1.0;
	m_unreachable_points.lifetime = ros::Duration();
}

FrontierDetector::~FrontierDetector()
{

}

void FrontierDetector::initmotion( )
{
ROS_INFO("+++++++++++++++++ Start the init motion ++++++++++++++\n");
	geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.5;

    uint32_t start_time = ros::Time::now().sec ;
    uint32_t curr_time = start_time ;
    while( curr_time < start_time + 12 )
    {
		m_velpub.publish(cmd_vel);
		curr_time = ros::Time::now().sec ;
    }

	cmd_vel.angular.z = 0.0;
	m_velpub.publish(cmd_vel);
ROS_INFO("+++++++++++++++++ end of the init motion ++++++++++++++\n");
}


//Norm function
float FrontierDetector::Norm(cv::Point2f x1, cv::Point2f x2)
{
	return pow(	(pow((x2.x-x1.x),2)+pow((x2.y-x1.y),2))	,0.5);
}


// default routine to convert img 2 gridmap
//cv::Point2f FrontierDetector::img2gridmap( cv::Point img_pt  )
//{
//	CV_Assert( m_nGridMapWidth == m_gridmap.info.width );
//
//	float fResolution=m_gridmap.info.resolution;
//	float fXstart =m_gridmap.info.origin.position.x  ;
//	float fYstart =m_gridmap.info.origin.position.y  ;
//
//	float fpx = static_cast<float>(img_pt.x) ;//- m_nGridMapWidth/2; //466/2 - 45 ;
//	float fpy = static_cast<float>(img_pt.y) ;//- m_nGridMapHeight/2; //528/2 ;
//
////	ROS_INFO("%f %f %f %d %d\n", fpx, fResolution, fXstart, m_nScale, m_nNumPyrDownSample );
////	ROS_INFO("%f %f %f %d \n", fpy, fResolution, fYstart, m_nScale );
//	float fgx = ( fpx * fResolution  + fXstart ) ;
//	float fgy = ( fpy * fResolution  + fYstart ) ;
//
//	return cv::Point2f( fgx, fgy );
//}
//
//cv::Point FrontierDetector::gridmap2img( cv::Point2f grid_pt)
//{
//	float fResolution=m_gridmap.info.resolution;
//	float fXstart =m_gridmap.info.origin.position.x  ;
//	float fYstart =m_gridmap.info.origin.position.y  ;
//
//	float fx = (grid_pt.x - fXstart) / fResolution ;//+ m_nGridMapWidth/2; //+ 466/2 + 45 ;
//	float fy = (grid_pt.y - fYstart) / fResolution ;//+ m_nGridMapHeight/2; //+ 528/2 ;
//
//	int nx = static_cast<int>( fx ) ; //+ fXstart  ;
//	int ny = static_cast<int>( fy ) ; //- fYstart  ;
//
//	return cv::Point2f( nx, ny );
//}



cv::Point2f FrontierDetector::img2gridmap( cv::Point img_pt  )
{
	CV_Assert( m_nGridMapWidth == m_gridmap.info.width );

	float fResolution=m_gridmap.info.resolution;
	float fXstart =m_gridmap.info.origin.position.x  ;
	float fYstart =m_gridmap.info.origin.position.y  ;

	float fpx = static_cast<float>(img_pt.x) - m_nGridMapWidth/2; //466/2 - 45 ;
	float fpy = static_cast<float>(img_pt.y) - m_nGridMapHeight/2; //528/2 ;

//	ROS_INFO("%f %f %f %d %d\n", fpx, fResolution, fXstart, m_nScale, m_nNumPyrDownSample );
//	ROS_INFO("%f %f %f %d \n", fpy, fResolution, fYstart, m_nScale );
	float fgx = ( fpx * fResolution ); // + fXstart ) ;
	float fgy = ( fpy * fResolution ); // - fYstart ) ;

	return cv::Point2f( fgx, fgy );
}

cv::Point FrontierDetector::gridmap2img( cv::Point2f grid_pt)
{
	float fResolution=m_gridmap.info.resolution;
	float fXstart =m_gridmap.info.origin.position.x  ;
	float fYstart =m_gridmap.info.origin.position.y  ;

	float fx = grid_pt.x / fResolution + m_nGridMapWidth/2; //+ 466/2 + 45 ;
	float fy = grid_pt.y / fResolution + m_nGridMapHeight/2; //+ 528/2 ;

	int nx = static_cast<int>( fx ); // + fXstart ) ;
	int ny = static_cast<int>( fy ); // - fYstart ) ;

	return cv::Point2f( nx, ny );
}

//gridValue function
int8_t FrontierDetector::gridValue(nav_msgs::OccupancyGrid &mapData, cv::Point2f Xp)
{
// Please make sure that Xp is not a downsampled point !!!!

	float resolution=mapData.info.resolution;
	float Xstartx=mapData.info.origin.position.x;
	float Xstarty=mapData.info.origin.position.y;
	float width=mapData.info.width;

	ROS_INFO("costmap info: %f %f %f %f \n", resolution, Xstartx, Xstarty, width);

	std::vector<signed char> Data=mapData.data;

	//returns grid value at "Xp" location
	//map data:  100 occupied      -1 unknown       0 free
	float indx=(  floor((Xp.y-Xstarty)/resolution)*width)+( floor((Xp.x-Xstartx)/resolution) );
	int nIdx = static_cast<int>(indx) ;
ROS_INFO("-- (res,height,width): %f %d %d \n", resolution, mapData.info.width, mapData.info.height);
ROS_INFO("-- grid idx map size: %d %d \n", nIdx, Data.size() );

	int8_t out;
	out=Data[nIdx];
ROS_INFO("out: %d", out);
	return out;
}

vector<cv::Point> FrontierDetector::eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize)
{
//ROS_INFO("eliminating suprious frontiers \n");
	//ROS_INFO("in func width: %d %d %f\n", m_globalcostmap.info.width, m_globalcostmap_cols, m_globalcostmap.info.resolution);

	vector<cv::Point> outFrontiers ;

	float fXstartx=costmapData.info.origin.position.x; // world coordinate in the costmap
	float fXstarty=costmapData.info.origin.position.y; // world coordinate in the costmap
	float resolution = costmapData.info.resolution ;

	int width= static_cast<int>(costmapData.info.width) ;
	std::vector<signed char> Data=costmapData.data;
ROS_INFO("front cand size: %d \n", frontierCandidates.size());

	for( size_t idx =0; idx < frontierCandidates.size(); idx++) // frontiers in image coord
	{
//ROS_INFO("frontier cand: %d %d \n", frontierCandidates[idx].x, frontierCandidates[idx].y );
		cv::Point2f frontier_in_Gridmap = img2gridmap( frontierCandidates[idx] * m_nScale );
		//returns grid value at "Xp" location
		//map data:  100 occupied      -1 unknown       0 free
//ROS_INFO("frontier in gridmap: %f %f origin: %f %f\n", frontier_in_Gridmap.x, frontier_in_Gridmap.y, fXstarty, fXstartx );
		int py_c= floor( (frontier_in_Gridmap.y - fXstarty ) / resolution  ) ; // px in costmap
		int px_c= floor( (frontier_in_Gridmap.x - fXstartx ) / resolution  ) ;
		int8_t cost ;
		int32_t ncost = 0;

//ROS_INFO(" idx px py %u %d %d\n", idx, px_c, py_c);
		cv::Mat roi = cv::Mat::zeros(winsize*2, winsize*2, CV_8U);

		for( int ridx =-winsize; ridx < winsize; ridx++)
		{
			for( int cidx=-winsize; cidx < winsize; cidx++)
			{
				int dataidx = px_c + cidx + (py_c + ridx) * width ;
//ROS_INFO("ind rix cidx %d %d %d ", idx, ridx, cidx);
				cost = Data[dataidx] ; // orig 0 ~ 254 --> mapped to 0 ~ 100
				if(cost > LEATHAL_COST_THR ) // unknown (-1)
				{
					ncost++; // = ncost + static_cast<float>(cost) ;
					//roi.at<uint8_t>(ridx+winsize, cidx+winsize) = static_cast<uint8_t>(cost);
				}
			}
		}
//		const int roi_idx = px_c + py_c * width ;
//		const string name = "/home/hankm/catkin_ws/src/frontier_detector/images/roi/frontier_patch_"
//							+ std::to_string(px_c) + "_" + std::to_string(py_c) + ".png" ;
//		cv::imwrite(name,  roi) ;
//ROS_INFO("here 3 \n" ) ;
		float fcost = static_cast<float>(ncost) / static_cast<float>( (winsize*2) * (winsize*2) ) ;
		if( fcost < m_frontier_cost_thr  ) // 0 ~ 100
		{
			// move back to image coord
			//ROS_INFO("pts in cost map:  (%d  %d) in gridmap (%f %f) frontierCandidates: (%d  %d) res: %f  sx sy (%f %f)\n",
//					px_c, py_c, frontier_in_Gridmap.x, frontier_in_Gridmap.y, frontierCandidates[idx].x, frontierCandidates[idx].y,
//					resolution, fXstartx, fXstarty ) ;

			cv::Point frontier_in_img = gridmap2img( frontier_in_Gridmap ) ;
//ROS_INFO("in gridmap: %d %d in image: %f %f\n", frontier_in_Gridmap.x, frontier_in_Gridmap.y, frontier_in_img.x/m_nScale, frontier_in_img.y/m_nScale );
			outFrontiers.push_back( cv::Point(frontier_in_img.x / m_nScale, frontier_in_img.y / m_nScale)  );
		}
	}

//ROS_INFO("eliminating unreachable pts \n" ) ;
	// Eliminate unreachable frontiers

	set<pointset, pointset> unreachable_frontiers;
	{
		const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
		unreachable_frontiers = m_unreachable_frontier_set ;
	}
	if( !unreachable_frontiers.empty() )
	{
		vector<cv::Point> validFrontiers ;
		cv::Point pt_img;
		for (size_t i=0; i < outFrontiers.size(); i++ )
		{
			bool isValidPt = true ;
			pt_img = outFrontiers[i];
			cv::Point2f pt = img2gridmap( pt_img * m_nScale );
			float fx = pt.x ;
			float fy = pt.y ;
			for (const auto & di : unreachable_frontiers)
			{
				float fdist = std::sqrt( (fx - di.d[0]) * (fx - di.d[0]) + (fy - di.d[1]) * (fy - di.d[1]) ) ;
				if(fdist < 0.05)
				{
					ROS_WARN("(%f %f) and (%f %f) are the same? unreachable point? \n",
								fx, fy, di.d[0], di.d[1]);
					isValidPt = false;
				}
			}
			if(isValidPt)
			{
				validFrontiers.push_back(pt_img);
			}
			else
			{
				cv::Point2f frontier_in_Gridmap = img2gridmap( pt_img * m_nScale );
				geometry_msgs::Point p;
				p.x = frontier_in_Gridmap.x ;
				p.y = frontier_in_Gridmap.y ;
				p.z = 0.0 ;
				m_unreachable_points.points.push_back(p) ;
				//ROS_ERROR("(%f %f) is in the unreachable goal list \n", pt.x, pt.y);
			}
		}

		return validFrontiers ;  // in image coord frame
	}
	else
	{
		return outFrontiers;
	}
}


int FrontierDetector::displayMapAndFrontiers( const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize)
{
	//ROS_INFO("weird maprows: %d %d\n", mapimg.rows, mapimg.cols);
	if(		mapimg.empty() ||
			mapimg.rows == 0 || mapimg.cols == 0 || m_globalcostmap.info.width == 0 || m_globalcostmap.info.height == 0)
		return 0;

	float fXstartx=m_globalcostmap.info.origin.position.x; // world coordinate in the costmap
	float fXstarty=m_globalcostmap.info.origin.position.y; // world coordinate in the costmap
	float resolution = m_globalcostmap.info.resolution ;
	int cmwidth= static_cast<int>(m_globalcostmap.info.width) ;
	//auto Data=m_globalcostmap.data ;

	int x = winsize ;
	int y = winsize ;
	int width = mapimg.cols  ;
	int height= mapimg.rows  ;
	cv::Mat img = cv::Mat::zeros(height + winsize*2, width + winsize*2, CV_8UC1);

//ROS_INFO("costmap size: %d %d \n",  m_globalcostmap.info.width, m_globalcostmap.info.height);

	cv::Mat tmp = img( cv::Rect( x, y, width, height ) ) ;

ROS_INFO("mapsize: %d %d %d %d %d %d\n", tmp.cols, tmp.rows, mapimg.cols, mapimg.rows, img.cols, img.rows );

	mapimg.copyTo(tmp);
	cv::Mat dst;
	cvtColor(tmp, dst, cv::COLOR_GRAY2BGR);

	for( size_t idx = 0; idx < frontiers.size(); idx++ )
	{
		int x = frontiers[idx].x - winsize ;
		int y = frontiers[idx].y - winsize ;
		int width = winsize * 2 ;
		int height= winsize * 2 ;
		cv::rectangle( dst, cv::Rect(x,y,width,height), cv::Scalar(0,255,0) );
	}

	cv::Mat dstroi = dst( cv::Rect( mapimg.cols/4, mapimg.rows/4, mapimg.cols/2, mapimg.rows/2 ) );
	cv::pyrDown(dstroi, dstroi, cv::Size(dstroi.cols/2, dstroi.rows/2) );

#ifdef FFD_DEBUG_MODE
	cv::namedWindow("mapimg", 1);
	cv::imshow("mapimg",dstroi);
	cv::waitKey(30);
#endif
}


bool FrontierDetector::isValidPlan( vector<cv::Point>  )
{

}

void FrontierDetector::publishDone( )
{
	std_msgs::Bool done_task;
	done_task.data = true;
	m_donepub.publish( done_task );
}



geometry_msgs::PoseStamped FrontierDetector::StampedPosefromSE2( float x, float y, float yaw_radian )
{
	geometry_msgs::PoseStamped outPose ;
	outPose.pose.position.x = x ;
	outPose.pose.position.y = y ;

	float c[3] = {0,};
	float s[3] = {0,};
	c[0] = cos(yaw_radian/2) ;
	c[1] = cos(0) ;
	c[2] = cos(0) ;
	s[0] = sin(yaw_radian/2) ;
	s[1] = sin(0) ;
	s[2] = sin(0) ;

	float qout[4] = {0,};
	qout[0] = c[0]*c[1]*c[2] + s[0]*s[1]*s[2];
	qout[1] = c[0]*c[1]*s[2] - s[0]*s[1]*c[2];
	qout[2] = c[0]*s[1]*c[2] + s[0]*c[1]*s[2];
	qout[3] = s[0]*c[1]*c[2] - c[0]*s[1]*s[2];

	outPose.pose.orientation.w = qout[0] ;
	outPose.pose.orientation.x = qout[1] ;
	outPose.pose.orientation.y = qout[2] ;
	outPose.pose.orientation.z = qout[3] ;

	outPose.header.frame_id = m_worldFrameId ;
	outPose.header.stamp = ros::Time::now() ;

	return outPose;
}

geometry_msgs::PoseStamped FrontierDetector::GetCurrPose ( )
{
	geometry_msgs::PoseStamped outPose ;
	outPose.header = m_robotpose.header ;
	outPose.pose.position.x = m_robotpose.pose.pose.position.x ;
	outPose.pose.position.y = m_robotpose.pose.pose.position.y ;
	outPose.pose.position.z = 0.f ;

	outPose.pose.orientation = m_robotpose.pose.pose.orientation ;

	return outPose;
}


void FrontierDetector::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	m_globalcostmap = *msg ;
	m_globalcostmap_rows = m_globalcostmap.info.height ;
	m_globalcostmap_cols = m_globalcostmap.info.width ;
}

void FrontierDetector::gridmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
//	ROS_ERROR("in gridmapcallback \n");
//	m_gridmap = *msg ;
//	{
//		const std::unique_lock<mutex> lock(mutex_gridmap_image) ;
//		memcpy( &m_cvMapImg.data[0], &m_gridmap.data[0], sizeof(uchar) * m_nGridMapWidth * m_nGridMapHeight  );
//	}
//	// m_gridmap  := -1, 0, 100
//	// m_cvMapImg := 128, 0, 100
//
//	cv::Mat dst = m_cvMapImg.clone();
//	cv::pyrDown(m_cvMapImg, dst, cv::Size(dst.cols/2, dst.rows/2));
//	cv::pyrDown(dst, dst, cv::Size(dst.cols/2, dst.rows/2));
//	cv::pyrDown(dst, dst, cv::Size(dst.cols/2, dst.rows/2));
//
//	cv::namedWindow("gridmap");
//	cv::imshow("gridmap", dst);
//	cv::waitKey(10);
}

void FrontierDetector::robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	m_robotpose = *msg ;
	//ROS_INFO("callback pose:  %f %f \n", m_robotpose.pose.pose.position.x, m_robotpose.pose.pose.position.y);
}

void FrontierDetector::robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg )
{
	m_robotvel = *msg ;
}



void FrontierDetector::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) //const octomap_server::mapframedata& msg )
{

	if(m_robotvel.linear.x == 0 && m_robotvel.angular.z == 0 ) // robot is physically stopped
		m_eRobotState = ROBOT_STATE::ROBOT_IS_NOT_MOVING;

	ROS_INFO("Robot state in mapdataCallback: %d \n ",  m_eRobotState);

	if(m_eRobotState >= ROBOT_STATE::FORCE_TO_STOP )
	{
		ROS_WARN("Force to stop flag is up cannot proceed mapdataCallback() \n");
		return;
	}
ros::WallTime startTime, endTime;
startTime = ros::WallTime::now();

	m_gridmap = *msg ;

	ROS_INFO("in mapdataCallback (map info: %f %f %d %d)\n",
			m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y,
			m_gridmap.info.height, m_gridmap.info.width);

	if(m_gridmap.info.height == 0)
		return;

	int nrows = m_gridmap.info.height ;
	int ncols = m_gridmap.info.width ;

//	{
//		const std::unique_lock<mutex> lock(mutex_gridmap_image) ;
//		//memcpy( &m_sMapImg.data[0], &m_gridmap.data[0], sizeof(schar) * m_nGridMapWidth * m_nGridMapHeight  ); // 0, -1, 255
//		memcpy( &img_crop.data[0], &m_gridmap.data[0], sizeof(uchar) * m_nGridMapWidth * m_nGridMapHeight  ); // 0(free), 255(unknown), 100(occu)
//		cv::threshold(m_uMapImg_buf, m_uMapImgUnk, 200, ffp::MapStatus::UNKNOWN, cv::THRESH_BINARY);	// 0, 127, 0
//		cv::threshold(m_uMapImg_buf, m_uMapImgOcc, 200, 1, cv::THRESH_TOZERO_INV);	// 0, 0, 100
//		cv::threshold(m_uMapImgOcc, m_uMapImgOcc, 0, ffp::MapStatus::OCCUPIED, cv::THRESH_BINARY);	// 0, 0, 255
//		m_uMapImg = m_uMapImgUnk + m_uMapImgOcc;
//	}

	for( int ii =0 ; ii < nrows; ii++)
	{
		for( int jj = 0; jj < ncols; jj++)
		{
			int8_t occupancy = m_gridmap.data[ ii * ncols + jj ];

			if ( occupancy < 0 )
			{
				m_uMapImg.at<uchar>(ii,jj) = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
			}
			else if( occupancy == 0 )
			{
				m_uMapImg.at<uchar>(ii,jj) = static_cast<uchar>(ffp::MapStatus::FREE) ;
			}
			else
			{
				m_uMapImg.at<uchar>(ii,jj) = static_cast<uchar>(ffp::MapStatus::OCCUPIED) ;
			}
		}
	}

	// 2's complement representation of a signed 8 bit int
	// -1 - -> 1 1 1 1 1 1 1 1
	// -127 -> 1 0 0 0 0 0 0 1
	// -128 -> 1 0 0 0 0 0 0 0

	// m_gridmap  := -1,  0, 100
	// m_cvMapImg := 255, 0, 100

	m_markerfrontierpub.publish(m_points); // Publish frontiers to renew Rviz
	m_makergoalpub.publish(m_exploration_goal);


// The robot is not moving (or ready to move)... we can go ahead plan the next action...
// i.e.) We locate frontier points again, followed by publishing the new goal

	ROS_INFO("******* Begin mapdataCallback procedure ******** \n");

//	cv_bridge::CvImagePtr cv_ptr;
//	cv_ptr = cv_bridge::toCvCopy( msg.image_map_2d, sensor_msgs::image_encodings::MONO8 );

	cv::Mat img ;
	img = m_uMapImg.clone();

	if( m_nNumPyrDownSample > 0)
	{
		// be careful here... using pyrDown() interpolates occ and free, making the boarder area (0 and 127) to be 127/2 !!
		// 127 reprents an occupied cell !!!
		//downSampleMap(img);
		for(int iter=0; iter < m_nNumPyrDownSample; iter++ )
			pyrDown(img, img, cv::Size(img.rows/2, img.cols/2));
		//cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/launch/pyr_img.png", img);
		clusterToThreeLabels( img );
	}
	ffp::FrontPropagation oFP(img); // image uchar
	oFP.update(img, cv::Point(0,0));
	oFP.extractFrontierRegion( img ) ;

	cv::Mat img_frontiers = oFP.GetFrontierContour() ;

	cv::Mat dst;
	cvtColor(img_frontiers, dst, cv::COLOR_GRAY2BGR);

// locate the most closest labeled points w.r.t the centroid pts

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	cv::findContours( img_frontiers, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );

#ifdef FFD_DEBUG_MODE
	cv::imwrite("/home/glab/catkin_ws/src/frontier_detector/images/input_img.png", m_uMapImg);
	cv::imwrite("/home/glab/catkin_ws/src/frontier_detector/images/labeled_img.png", img);
	cv::imwrite("/home/glab/catkin_ws/src/frontier_detector/images/img_frontiers.png",img_frontiers);
#endif

	if( contours.size() == 0 )
		return;

	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
//ROS_INFO("hierarchy: %d \n", idx);
		cv::Scalar color( rand()&255, rand()&255, rand()&255 );
		drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
	}

	vector<cv::Point2f> fcents;
	for(int i=0; i < contours.size(); i++)
	{
		float fx =0, fy =0 ;
		float fcnt = 0 ;
		vector<cv::Point> contour = contours[i];
		for( int j=0; j < contour.size(); j++)
		{
			fx += static_cast<float>( contour[j].x ) ;
			fy += static_cast<float>( contour[j].y ) ;
			fcnt += 1.0;
		}
		fx = fx/fcnt ;
		fy = fy/fcnt ;

		cv::Point2f fcent( fx,  fy ) ;
		fcents.push_back(fcent);
	}

	// get closest frontier pt to each cent
	// i.e.) the final estimated frontier points
	vector<cv::Point> frontiers_cand;
//ROS_INFO_ONCE("thr %d  \n", m_frontiers_region_thr);

	for( int i = 0; i < contours.size(); i++ )
	{
		vector<cv::Point> contour = contours[i] ;

		if(contour.size() < m_frontiers_region_thr ) // don't care about small frontier regions
			continue ;

		float fcentx = fcents[i].x ;
		float fcenty = fcents[i].y ;

		float fmindist = 1000 ;
		int nmindistidx = -1;

		for (int j=0; j < contour.size(); j++)
		{
			float fx = static_cast<float>(contour[j].x) ;
			float fy = static_cast<float>(contour[j].y) ;
			float fdist = std::sqrt( (fx - fcentx) * (fx - fcentx) + (fy - fcenty) * (fy - fcenty) );
			if(fdist < fmindist)
			{
				fmindist = fdist ;
				nmindistidx = j ;
			}
		}

		assert(nmindistidx >= 0);
		cv::Point frontier = contour[nmindistidx];
		frontiers_cand.push_back(frontier) ;
	}

ROS_INFO("costmap msg width: %d \n", m_globalcostmap.info.width );

	geometry_msgs::Point p;
	m_cands.points.clear();
	m_exploration_goal.points.clear();
	m_points.points.clear();
//	m_unreachable_points.points.clear();

	vector<cv::Point> frontier_cand_in_gridmap ;
	for(size_t idx=0; idx < frontiers_cand.size(); idx++)
	{
		cv::Point2f frontier_in_Gridmap = img2gridmap( frontiers_cand[idx] * m_nScale );
		frontier_cand_in_gridmap.push_back( frontier_in_Gridmap );
		p.x = frontier_in_Gridmap.x ;
		p.y = frontier_in_Gridmap.y ;
		p.z = 0.0 ;
		m_cands.points.push_back(p);
		//ROS_INFO("frontier cands: %f %f \n", p.x, p.y);
	}

	// eliminate frontier points at obtacles
	vector<cv::Point> frontiers;
	if( m_globalcostmap.info.width > 0 )
	{
ROS_INFO( "eliminating supurious frontiers \n" );
		frontiers = eliminateSupriousFrontiers( m_globalcostmap, frontiers_cand, m_nROISize) ;
	}
	else
	{
		ROS_INFO("costmap hasn't updated \n");
		frontiers = frontiers_cand ; // points in img coord
	}

//	for(int idx=0; idx < frontiers.size(); idx++)
//		ROS_INFO("Valid frontier points: %f %f \n", frontiers[idx].x, frontiers[idx].y);

	if( frontiers.size() == 0 )
	{
		ROS_WARN("no valid frontiers \n");
		//isdone = true;
		return;
	}

	if(img_frontiers.rows > 0 && img_frontiers.cols > 0 )
	{
		displayMapAndFrontiers( img_frontiers, frontiers, m_nROISize );
	}

//	if(m_globalcostmap.info.width > 0 &&  m_globalcostmap.info.height > 0 )
//		assessFrontiers( frontiers );

	// set exploration goals
	for(int idx=0; idx < frontiers.size(); idx++)
	{
		cv::Point frontier = frontiers[idx] ;
//ROS_INFO("frontier pts found: %d %d \n",frontier.x,frontier.y);
#ifdef FFD_DEBUG_MODE
		cv::circle(dst, frontier, 3, CV_RGB(255,0,0), 2);
#endif

		// scale then conv 2 gridmap coord
		cv::Point2f frontier_in_Gridmap = img2gridmap( frontier * m_nScale );
		geometry_msgs::PoseWithCovarianceStamped mygoal ; // float64
		mygoal.header.stamp=ros::Time(0);
		mygoal.header.frame_id = m_worldFrameId;
		mygoal.pose.pose.position.x= frontier_in_Gridmap.x ;
		mygoal.pose.pose.position.y= frontier_in_Gridmap.y ;
		mygoal.pose.pose.position.z=0.0;
		//m_exploration_goal.push_back(mygoal) ;

		m_targetspub.publish(mygoal);
		p.x = mygoal.pose.pose.position.x ;
		p.y = mygoal.pose.pose.position.y ;
		p.z = 0.0 ;
		m_points.points.push_back(p);

//		ROS_INFO("frontier in img: (%d %d) in gridmap: (%f %f) scale: %d\n",
//				frontier.x, frontier.y, p.x, p.y, m_nScale );

	}

#ifdef FFD_DEBUG_MODE
		imwrite("/home/glab/catkin_ws/src/frontier_detector/images/frontier_cents.png", dst);
#endif

	// publish the "to be exploration goals" (detected points)
	// std::random_shuffle(m_points.points.begin(), m_points.points.end());

	float resolution=m_globalcostmap.info.resolution;
	float Xstartx=m_globalcostmap.info.origin.position.x;
	float Xstarty=m_globalcostmap.info.origin.position.y;
	float width=m_globalcostmap.info.width;

	//ROS_INFO("costmap info: %f %f %f %f \n", resolution, Xstartx, Xstarty, width);
	//ROS_INFO("frontier: %f %f \n", m_points.points[0].x, m_points.points[0].y );

// generate a path trajectory
// call make plan service

	ROS_INFO("Looking for a new plan \n");

	nav_msgs::GetPlan::Response best_res ;
	size_t best_len = 100000000 ;
	size_t best_idx = 0;

	nav_msgs::GetPlan::Request req;

	req.start = GetCurrPose( );

	for (size_t idx=0; idx < m_points.points.size(); idx++)
	{
		p = m_points.points[idx];  // just for now... we need to fix it later

		req.goal  = StampedPosefromSE2( p.x, p.y, 0.f );
		req.goal.header.frame_id = m_worldFrameId ;

		//ROS_INFO("path to (%f %f)\n", req.goal.pose.position.x, req.goal.pose.position.y );

// makeplan_client.call() calls navfn_ros::makePlan(), which successfully finds the optimal plan.
		nav_msgs::GetPlan::Response res ;
		bool bsuc = m_makeplan_client.call( req, res );
///////////////////////////////////////////////////////////////////////////
// disable this part if Fetch robot is not used
///////////////////////////////////////////////////////////////////////////
		m_makeplan_client.waitForExistence();
		//sleep(.5);
///////////////////////////////////////////////////////////////////////////
		size_t curr_len = res.plan.poses.size() ;
//ROS_INFO("succ, curr/best len: (%d %u %u) \n",bsuc, curr_len, best_len);
		if(curr_len < best_len && curr_len > MIN_TARGET_DIST)
		{
			best_len = curr_len ;
			best_idx = idx ;
			best_res = res ;
			m_bestgoal.header.frame_id = m_worldFrameId ;
			m_bestgoal.pose.pose = req.goal.pose ;
		}
	}

	p.x = m_bestgoal.pose.pose.position.x ;
	p.y = m_bestgoal.pose.pose.position.y ;
	p.z = 0.0 ;
	m_exploration_goal.points.push_back(p);

	m_markercandpub.publish(m_cands);

	m_markerfrontierpub.publish(m_points);
//////////////////////////////////////////////////////
// lets disable publishing goal if Fetch robot is not used
	m_currentgoalpub.publish(m_bestgoal);
/////////////////////////////////////////////////////

	m_makergoalpub.publish(m_exploration_goal);

	// publish the best goal of the path plan
	ROS_INFO("@mapDataCallback start(%f %f) found the best goal(%f %f) best_len (%u)\n",
			req.start.pose.position.x, req.start.pose.position.y,
			m_bestgoal.pose.pose.position.x, m_bestgoal.pose.pose.position.y, best_len);

	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		m_eRobotState == ROBOT_STATE::ROBOT_IS_READY_TO_MOVE;
	}

endTime = ros::WallTime::now();

// print results
double execution_time = (endTime - startTime).toNSec() * 1e-6;
ROS_INFO("\n "
		 " ****************************************************** \n "
		 "	 mapDataCallback exec time (ms): %f \n "
		 " ****************************************************** \n "
				, execution_time);

}

void FrontierDetector::doneCB( const actionlib::SimpleClientGoalState& state )
{
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    if (m_move_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
         // do something as goal was reached
    	ROS_INFO("goal is reached \n");
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			m_eRobotState = ROBOT_STATE::ROBOT_IS_NOT_MOVING ;
		}
    }
    else if (m_move_client.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        // do something as goal was canceled
    	ROS_ERROR("Aboarted ... \n");
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			m_eRobotState = ROBOT_STATE::ROBOT_IS_NOT_MOVING ;
		}
    }
    else
    {
    	ROS_ERROR("unknown state \n");
    	exit(-1);
    }
}


void FrontierDetector::moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
// call actionlib
// robot is ready to move
	ROS_INFO("Robot state in moveRobotCallback: %d \n ",  m_eRobotState);

	if( m_eRobotState >= ROBOT_STATE::FORCE_TO_STOP ) //|| isdone )
		return;

	geometry_msgs::PoseWithCovarianceStamped goalpose = *msg ;

	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		m_eRobotState = ROBOT_STATE::ROBOT_IS_MOVING ;
	}

	ROS_INFO("@moveRobotCallback received a plan\n");

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = m_worldFrameId; //m_baseFrameId ;
	goal.target_pose.header.stamp = ros::Time::now() ;

//	geometry_msgs::PoseWithCovarianceStamped goalpose = // m_pathplan.poses.back() ;

	goal.target_pose.pose.position.x = goalpose.pose.pose.position.x ;
	goal.target_pose.pose.position.y = goalpose.pose.pose.position.y ;
	goal.target_pose.pose.orientation.w = goalpose.pose.pose.orientation.w ;

// inspect the path
//	ROS_INFO("printing path for start(%f %f) to goal(%f %f): \n",
//			req.start.pose.position.x, req.start.pose.position.y, req.goal.pose.position.x, req.goal.pose.position.y );

//	ROS_INFO("@moveRobotCallback goal pose: %f %f %f\n", goalpose.pose.pose.position.x, goalpose.pose.pose.position.y, goalpose.pose.pose.orientation.w);
//////////////////////////////////////////////////////////////////////////////////////////////
// when creating path to frontier points, navfn_ros::makePlan() is called
// move_client calls moveBase::makePlan() ...
//////////////////////////////////////////////////////////////////////////////////////////////
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, sending a goal +++++++++++++++++++++++++++++++++++++\n");
	m_move_client.sendGoal(goal, boost::bind(&FrontierDetector::doneCB, this, _1), SimpleMoveBaseClient::SimpleActiveCallback() ) ;
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, a goal is sent +++++++++++++++++++++++++++++++++++++\n");
	//m_move_client.sendGoalAndWait(goal, ros::Duration(0, 0), ros::Duration(0, 0)) ;
	m_move_client.waitForResult();
}

void FrontierDetector::unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg )
{
	ROS_INFO("Robot state in unreachablefrontierCallback: %d \n ",  m_eRobotState);

	geometry_msgs::PoseStamped unreachablepose = *msg ;
	pointset pi ;
	pi.d[0] = unreachablepose.pose.position.x ;
	pi.d[1] = unreachablepose.pose.position.y ;

	{
		const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
		m_unreachable_frontier_set.insert( pi ) ;

		geometry_msgs::Point p;
		p.x = pi.d[0];
		p.y = pi.d[1];
		m_unreachable_points.points.push_back(p);
	}

	m_unreachpointpub.publish( m_unreachable_points );

	for (const auto & di : m_unreachable_frontier_set)
	    ROS_WARN("unreachable pts: %f %f\n", di.d[0], di.d[1]);

	// stop the robot and restart frontier detection procedure

	ROS_WARN("+++++++++++Cancling the unreachable goal +++++++++++++\n");

	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		m_eRobotState = ROBOT_STATE::FORCE_TO_STOP ;
	}

	m_move_client.stopTrackingGoal();
	m_move_client.waitForResult();
	m_move_client.cancelGoal();
	m_move_client.waitForResult();

	ROS_INFO("+++++++++++ Robot is ready for motion +++++++++++++\n");
	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		m_eRobotState = ROBOT_STATE::ROBOT_IS_READY_TO_MOVE ;
	}

}



//void FrontierDetector::mapdataCallback(const octomap_server::mapframedata& msg )
//{
//	cv_bridge::CvImagePtr cv_ptr;
//	cv_ptr = cv_bridge::toCvCopy( msg.image_map_2d, sensor_msgs::image_encodings::MONO8 );
//
//	cv::Mat img, img_edge, img_occ_only, img_occ_dilate, img_non_occ_dialate, img_frontiers ;
//	img = cv_ptr->image.clone() ;
//
////	for(int rowidx =0; rowidx < img.rows; rowidx++)
////	{
////		for(int colidx=0; colidx < img.cols; colidx++)
////		{
////			//if( img.at<uchar>(rowidx,colidx) >= 200 )
////			ROS_INFO(" %u ", img.at<uchar>(rowidx,colidx) );
////		}
////		ROS_INFO("\n");
////	}
//
////	cv::imshow("img",img);
////	cv::waitKey(0);
//
////	ROS_INFO("img type: %d \n", img.type());  // 0 == CV_8U == uchar
//
//	// edge
//	cv::Mat imgsrc = img.clone();
//	img_edge = img.clone();
//	cv::Canny(img, img_edge, 0, 255, 3);
//	cv::threshold(img_edge, img_edge, OCCUPIED_BIN_THR, 255, cv::THRESH_BINARY);
//
//	// obstacles , walls
//	cv::threshold(img, img_occ_only, OCCUPIED_BIN_THR, 255, cv::THRESH_BINARY);
//
//	// dialate
//	//cv::morphologyEx(img_occ_only, , op, kernel, anchor, iterations, borderType, borderValue)
//	cv::dilate(img_occ_only, img_occ_dilate, m_morph_kernel, cv::Point(-1,-1),1);
//
//	// inverse image
//	cv::threshold( img_occ_dilate, img_non_occ_dialate, 1, 255, CV_THRESH_BINARY_INV);
//	img_frontiers = img_edge.mul( img_non_occ_dialate );
////	cv::erode(img_frontiers, img_frontiers, m_morph_kernel);
////	cv::dilate(img_frontiers, img_frontiers, m_morph_kernel);
//
/////////////////////////////////////////////////////////////////////////////////////////////////
////  connected componentWithStat
////	stat 	--> 5 col matrix where [0],[1],[2],[3] are x, y, width, and height. [4] is the size (number of pixels)
////	centroid--> 2 col matrix where [0],[1] are the centroids (x and y coordinates)
/////////////////////////////////////////////////////////////////////////////////////////////////
//
//	cv::Mat labelImage(img.size(), CV_32S);
//	cv::Mat stats, centroids;
//    int nLabels = cv::connectedComponentsWithStats(img_frontiers, labelImage, stats, centroids, 8);
//    std::vector<cv::Vec3b> colors(nLabels);
////    colors[0] = cv::Vec3b(0, 0, 0);//background
////    for(int label = 1; label < nLabels; ++label){
////        colors[label] = cv::Vec3b( (rand()&255), (rand()&255), (rand()&255) );
////    }
////    cv::Mat dst(img.size(), CV_8UC3);
////    for(int r = 0; r < dst.rows; ++r){
////        for(int c = 0; c < dst.cols; ++c){
////            int label = labelImage.at<int>(r, c);
////            cv::Vec3b &pixel = dst.at<cv::Vec3b>(r, c);
////            pixel = colors[label];
////         }
////     }
//
//    cv::Mat dst;
//    cvtColor(img_frontiers, dst, cv::COLOR_GRAY2BGR);
////    for(int i = 1; i < nLabels; ++i)
////    {
////        int* label = stats.ptr<int>(i);
////        if(label[4] < WEAK_COMPONENT_THR) continue;
////
////        cv::rectangle(dst, cv::Rect(label[0], label[1], label[2], label[3]), cv::Scalar(0,255,255));
////    }
//
//// locate the most closest labeled points w.r.t the centroid pts
//
//    vector<vector<cv::Point> > contours;
//    vector<cv::Vec4i> hierarchy;
//    cv::findContours( img_frontiers, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
//
//    // iterate through all the top-level contours,
//    // draw each connected component with its own random color
//    int idx = 0;
//    for( ; idx >= 0; idx = hierarchy[idx][0] )
//    {
//        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
//        drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
//    }
//
//    /// Get the moments
////    vector<cv::Moments> mu(contours.size() );
////    for( int i = 0; i < contours.size(); i++ )
////    	mu[i] = moments( contours[i], false );
////
////    // get cent
////    vector<cv::Point2f> mc( contours.size() );
////    for( int i = 0; i < contours.size(); i++ )
////    	mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
//
//    vector<cv::Point> cents;
//    for(int i=0; i < contours.size(); i++)
//    {
//    	float fx =0, fy =0 ;
//    	float fcnt = 0 ;
//    	vector<cv::Point> contour = contours[i];
//    	for( int j=0; j < contour.size(); j++)
//    	{
//    		fx += static_cast<float>( contour[j].x ) ;
//    		fy += static_cast<float>( contour[j].y ) ;
//    		fcnt += 1.0;
//    	}
//    	fx = fx/fcnt ;
//    	fy = fy/fcnt ;
//
//    	cv::Point cent( static_cast<int>(fx),  static_cast<int>(fy) ) ;
//    	cents.push_back(cent);
//    }
//
//    // get closest frontier pt to each cent
//    // i.e.) the final estimated frontier points
//    vector<cv::Point> frontiers;
//    //ROS_ERROR("thr %d  \n", m_frontiers_region_thr);
//	for( int i = 0; i < contours.size(); i++ )
//	{
//		vector<cv::Point> contour = contours[i] ;
//
//		if(contour.size() < m_frontiers_region_thr ) // don't care about small frontier regions
//			continue ;
//
//		float fcentx = static_cast<float>(cents[i].x) ;
//		float fcenty = static_cast<float>(cents[i].y) ;
//
//		float fmindist = 1000 ;
//		int nmindistidx = -1;
//
//		for (int j=0; j < contour.size(); j++)
//		{
//			float fx = static_cast<float>(contour[j].x) ;
//			float fy = static_cast<float>(contour[j].y) ;
//			float fdist = std::sqrt( (fx - fcentx) * (fx - fcentx) + (fy - fcenty) * (fy - fcenty) );
//			if(fdist < fmindist)
//			{
//				fmindist = fdist ;
//				nmindistidx = j ;
//			}
//		}
//
//		assert(nmindistidx >= 0);
//		cv::Point frontier = contour[nmindistidx];
//		frontiers.push_back(frontier) ;
//	}
//
//	geometry_msgs::Point p;
//	// publish detected points
//	for(int idx=0; idx < frontiers.size(); idx++)
//	{
//		cv::Point frontier = frontiers[idx] ;
////		ROS_ERROR("cent: %d %d \n",frontier.x,frontier.y);
//		cv::circle(dst, frontier, 3, CV_RGB(255,0,0), 2);
//
//		// scale then conv 2 gridmap coord
//		cv::Point2f frontier_in_Gridmap = img2grimap( frontier * m_nScale );
//
//		m_exploration_goal.header.stamp=ros::Time(0);
//		m_exploration_goal.header.frame_id = m_worldFrameId;
//		m_exploration_goal.point.x= frontier_in_Gridmap.x ;
//		m_exploration_goal.point.y= frontier_in_Gridmap.y ;
//		m_exploration_goal.point.z=0.0;
//
//		ROS_INFO("in image: %d %d in gridmap: %f %f ",
//				frontier.x*m_nScale, frontier.y*m_nScale, frontier_in_Gridmap.x, frontier_in_Gridmap.y );
//
//		m_targetspub.publish(m_exploration_goal); // detected points
//
//		p.x = m_exploration_goal.point.x ;
//		p.y = m_exploration_goal.point.y ;
//		p.z = 0.0 ;
//
//		m_points.points.push_back(p);
//	}
//
//	m_markerpub.publish(m_points);
//	m_points.points.clear();
//
////	cv::namedWindow("frontier_detector", 1);
////	cv::namedWindow("edge",1);
////	cv::namedWindow("threshold", 1);
////	cv::namedWindow("dilate", 1);
////	cv::namedWindow("frontiers", 1);
//
////	cv::imshow("img", imgsrc);
////	cv::imshow("frontier_detector", img);
////	cv::imshow("edge",img_edge);
////	cv::imshow("threshold", img_occ_only);
////	cv::imshow("dilate", img_occ_dilate);
////	cv::imshow("frontiers", img_frontiers);
//
//    imshow( "contours frontiers", dst );
//
//	cv::waitKey(10);
//
////	cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/img.png", img);
////	cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/edge.png", img_edge);
////	cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/threshold.png", img_occ_only);
////	cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/dilate.png", img_occ_dilate);
////	cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/frontier_contours.png", img_frontiers);
////	cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/frontier_points.png", dst);
//
//	return ;
//}


}

