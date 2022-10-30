/*********************************************************************
*  Copyright (c) 2022, Ewha Graphics Lab
*
* This file is a part of Autoexplorer
*
* Autoexplorer is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Autoexplorer is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Autoexplorer.
* If not, see <http://www.gnu.org/licenses/>.

*      Author: Kyungmin Han (hankm@ewha.ac.kr)
*/

#include "frontier_detector_dms.hpp"


namespace autoexplorer
{

FrontierDetectorDMS::FrontierDetectorDMS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
mn_globalcostmapidx(0), mn_numthreads(16),
mb_isinitmotion_completed(false),
mp_cost_translation_table(NULL),
mb_strict_unreachable_decision(true)
{
	float fcostmap_conf_thr, fgridmap_conf_thr; // mf_unreachable_decision_bound ;
	int nweakcomp_threshold ;

	m_nh.getParam("/autoexplorer/debug_data_save_path", m_str_debugpath);
	m_nh.param("/autoexplorer/costmap_conf_thr", fcostmap_conf_thr, 0.1f);
	m_nh.param("/autoexplorer/gridmap_conf_thr", fgridmap_conf_thr, 0.8f);
	m_nh.param("/autoexplorer/occupancy_thr", mn_occupancy_thr, 50);
	m_nh.param("/autoexplorer/lethal_cost_thr", mn_lethal_cost_thr, 80);
	m_nh.param("/autoexplorer/global_width", mn_globalmap_width, 4000) ;
	m_nh.param("/autoexplorer/global_height", mn_globalmap_height, 4000) ;
	m_nh.param("/autoexplorer/unreachable_decision_bound", mf_neighoringpt_decisionbound, 0.2f);
	m_nh.param("/autoexplorer/weak_comp_thr", nweakcomp_threshold, 10);
	m_nh.param("/autoexplorer/num_downsamples", mn_numpyrdownsample, 0);
	m_nh.param("/autoexplorer/frame_id", m_worldFrameId, std::string("map"));
	m_nh.param("/autoexplorer/strict_unreachable_decision", mb_strict_unreachable_decision, true);

	m_nh.param("/move_base/global_costmap/resolution", mf_resolution, 0.05f) ;
	m_nh.param("move_base/global_costmap/robot_radius", mf_robot_radius, 0.12); // 0.3 for fetch

	mn_scale = pow(2, mn_numpyrdownsample);
	m_frontiers_region_thr = nweakcomp_threshold / mn_scale ;
	mn_roi_size = static_cast<int>( round( mf_robot_radius / mf_resolution ) ) * 2 ; // we never downsample costmap !!! dont scale it with roisize !!

	mn_globalmap_centx = mn_globalmap_width  / 2 ;
	mn_globalmap_centy = mn_globalmap_height / 2 ;

	//m_targetspub = m_nh.advertise<geometry_msgs::PointStamped>("detected_points", 10);
	m_currentgoalPub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("curr_goalpose", 10);
	m_makergoalPub = m_nh.advertise<visualization_msgs::Marker>("curr_goal_shape",10);
	m_markercandPub = m_nh.advertise<visualization_msgs::MarkerArray>("detected_shapes", 10);
	m_markerfrontierPub = m_nh.advertise<visualization_msgs::MarkerArray>("filtered_shapes", 10);
	m_markerfrontierregionPub = m_nh.advertise<visualization_msgs::Marker>("FR_shapes", 1);
	m_unreachpointPub = m_nh.advertise<visualization_msgs::MarkerArray>("unreachable_shapes", 10);
	m_velPub		= m_nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	m_donePub		= m_nh.advertise<std_msgs::Bool>("exploration_is_done",1);
	m_startmsgPub	= m_nh.advertise<std_msgs::Bool>("begin_exploration",1);
	m_otherfrontierptsPub = m_nh.advertise<nav_msgs::Path>("goal_exclusive_frontierpoints_list",1);

	m_mapframedataSub  	= m_nh.subscribe("map", 1, &FrontierDetectorDMS::mapdataCallback, this); // kmHan
	//m_frontierCandSub		= m_nh.subscribe("filtered_shapes", 1, &FrontierDetectorDMS::frontierCandCallback, this);
	m_currGoalSub 		= m_nh.subscribe("curr_goalpose",1 , &FrontierDetectorDMS::moveRobotCallback, this) ; // kmHan
	m_globalCostmapSub 	= m_nh.subscribe("move_base/global_costmap/costmap", 1, &FrontierDetectorDMS::globalCostmapCallBack, this );

	m_poseSub		   	= m_nh.subscribe("pose", 10, &FrontierDetectorDMS::robotPoseCallBack, this);
	m_velSub			= m_nh.subscribe("cmd_vel", 10, &FrontierDetectorDMS::robotVelCallBack, this);
	m_unreachablefrontierSub = m_nh.subscribe("unreachable_posestamped", 1, &FrontierDetectorDMS::unreachablefrontierCallback, this);
	m_makeplan_client = m_nh.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");

	mcvu_mapimg  	  = cv::Mat(mn_globalmap_height, mn_globalmap_width, CV_8U, cv::Scalar(127));

	int ncostmap_roi_size = mn_roi_size ; // / 2 ;
	int ngridmap_roi_size = mn_roi_size ;
	mn_correctionwindow_width = mn_scale * 2 + 1 ; // the size of the correction search window

	mo_frontierfilter = FrontierFilter(
			ncostmap_roi_size, ngridmap_roi_size, m_str_debugpath, mn_numpyrdownsample,
			fgridmap_conf_thr, fcostmap_conf_thr, mn_occupancy_thr, mn_lethal_cost_thr,
			mn_globalmap_width, mn_globalmap_height,
			mf_resolution, mf_neighoringpt_decisionbound);

	//m_oFrontierFilter.SetUnreachableDistThr( funreachable_decision_bound ) ;

	while(!m_move_client.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("move_base action server is up");

	mpo_costmap = new costmap_2d::Costmap2D();
	if (mp_cost_translation_table == NULL)
	{
		mp_cost_translation_table = new uint8_t[101];

		// special values:
		mp_cost_translation_table[0] = 0;  // NO obstacle
		mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
		mp_cost_translation_table[100] = 254;  // LETHAL obstacle
//		mp_cost_translation_table[-1] = 255;  // UNKNOWN

		// regular cost values scale the range 1 to 252 (inclusive) to fit
		// into 1 to 98 (inclusive).
		for (int i = 1; i < 99; i++)
		{
			mp_cost_translation_table[ i ] = uint8_t( ((i-1)*251 -1 )/97+1 );
		}
	}

	// Set markers

	m_prev_frontier_set = set<pointset, pointset>();
	m_curr_frontier_set = set<pointset, pointset>();
	//m_exploration_goal = SetVizMarker( m_worldFrameId, 1.f, 0.f, 1.f, 0.5  );
	mn_FrontierID = 1;
	mn_UnreachableFptID = 0;
	ROS_INFO("autoexplorer has initialized \n");

	std_msgs::Bool begin_task;
	begin_task.data = true;
	m_startmsgPub.publish( begin_task );
}

FrontierDetectorDMS::~FrontierDetectorDMS()
{
	delete [] mp_cost_translation_table;
}

void FrontierDetectorDMS::initmotion( )
{
ROS_INFO("+++++++++++++++++ Start the init motion ++++++++++++++\n");
	geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;

    uint32_t start_time = ros::Time::now().sec ;
    uint32_t curr_time = start_time ;
    while( curr_time < start_time + 6 )
    {
		m_velPub.publish(cmd_vel);
		curr_time = ros::Time::now().sec ;
    }

	cmd_vel.angular.z = 0.0;
	m_velPub.publish(cmd_vel);
ROS_INFO("+++++++++++++++++ end of the init motion ++++++++++++++\n");
}


cv::Point2f FrontierDetectorDMS::gridmap2world( cv::Point img_pt_roi  )
{
	float fgx =  static_cast<float>(img_pt_roi.x) * m_gridmap.info.resolution + m_gridmap.info.origin.position.x  ;
	float fgy =  static_cast<float>(img_pt_roi.y) * m_gridmap.info.resolution + m_gridmap.info.origin.position.y  ;

	return cv::Point2f( fgx, fgy );
}

cv::Point FrontierDetectorDMS::world2gridmap( cv::Point2f grid_pt)
{
	float fx = (grid_pt.x - m_gridmap.info.origin.position.x) / m_gridmap.info.resolution ;
	float fy = (grid_pt.y - m_gridmap.info.origin.position.y) / m_gridmap.info.resolution ;

	return cv::Point( (int)fx, (int)fy );
}


int FrontierDetectorDMS::displayMapAndFrontiers( const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize)
{
	if(		mapimg.empty() ||
			mapimg.rows == 0 || mapimg.cols == 0 || m_globalcostmap.info.width == 0 || m_globalcostmap.info.height == 0)
		return 0;

	float fXstartx=m_globalcostmap.info.origin.position.x; // world coordinate in the costmap
	float fXstarty=m_globalcostmap.info.origin.position.y; // world coordinate in the costmap
	float resolution = m_globalcostmap.info.resolution ;
	int cmwidth= static_cast<int>(m_globalcostmap.info.width) ;

	int x = winsize ;
	int y = winsize ;
	int width = mapimg.cols  ;
	int height= mapimg.rows  ;
	cv::Mat img = cv::Mat::zeros(height + winsize*2, width + winsize*2, CV_8UC1);

	cv::Mat tmp = img( cv::Rect( x, y, width, height ) ) ;

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

#ifdef FD_DEBUG_MODE
//	ROS_INFO("displaying dstroi \n");
//	cv::namedWindow("mapimg", 1);
//	cv::imshow("mapimg",dst);
//	cv::waitKey(30);
#endif
}


bool FrontierDetectorDMS::isValidPlan( vector<cv::Point>  )
{

}

void FrontierDetectorDMS::publishDoneExploration( )
{
	ROS_INFO("The exploration task is done... publishing -done- msg" );
	std_msgs::Bool done_task;
	done_task.data = true;

	m_frontier_points = visualization_msgs::MarkerArray() ;
	m_markerfrontierPub.publish(m_frontier_points);
	m_exploration_goal = visualization_msgs::Marker() ;
	m_makergoalPub.publish(m_exploration_goal); // for viz

	m_donePub.publish( done_task );

	ros::spinOnce();
}

void FrontierDetectorDMS::publishFrontierRegionMarkers( const vector<vector<cv::Point>>& contour )
{

}


void FrontierDetectorDMS::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("@globalCostmapCallBack \n");
	const std::unique_lock<mutex> lock(mutex_costmap);
//ROS_INFO("cm callback is called \n");
	m_globalcostmap = *msg ;
	mu_cmheight = m_globalcostmap.info.height ;
	mu_cmwidth = m_globalcostmap.info.width ;
}

void FrontierDetectorDMS::globalCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg )
{
	const std::unique_lock<mutex> lock(mutex_costmap);
	map_msgs::OccupancyGridUpdate cm_updates = *msg;
	std::vector<signed char> Data=cm_updates.data;
	int x = cm_updates.x ;
	int y = cm_updates.y ;
	int height = cm_updates.height ;
	int width  = cm_updates.width ;
	int cmwidth = m_globalcostmap.info.width ;
	int dataidx = 0 ;
	{
		for(int ii=0; ii < height; ii++)
		{
			for(int jj=0; jj < width; jj++)
			{
				int idx = x + jj + ii * cmwidth ;
				m_globalcostmap.data[dataidx] = Data[dataidx] ;
				dataidx++;
			}
		}
	}
}


void FrontierDetectorDMS::robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	m_robotpose = *msg ;
}

void FrontierDetectorDMS::robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg )
{
	m_robotvel = *msg ;
}

//int FrontierDetectorDMS::savegridmap( const nav_msgs::OccupancyGrid& gridmap, const string& filename )
//{
//}

int FrontierDetectorDMS::saveMap( const nav_msgs::OccupancyGrid& map, const string& infofilename, const string& mapfilename )
{
	int nwidth  = map.info.width ;
	int nheight = map.info.height ;
	float fox = map.info.origin.position.x ;
	float foy = map.info.origin.position.y ;
	float fres = map.info.resolution ;
	std::vector<signed char> cmdata;
	cmdata = map.data ;

	if( nwidth == 0 || nheight == 0 )
	{
		ROS_ERROR("Cannot save incomplete costmap \n");
		return -1;
	}

	geometry_msgs::PoseStamped robotpose = GetCurrRobotPose();
	float frx_w = (robotpose.pose.position.x - m_gridmap.info.origin.position.x) / m_gridmap.info.resolution ;
	float fry_w = (robotpose.pose.position.y - m_gridmap.info.origin.position.y) / m_gridmap.info.resolution ;
	int nrx_g = static_cast<int>(frx_w) ;
	int nry_g = static_cast<int>(fry_w) ;
	//int8_t cost = cmdata[ nwidth * nry_g + nrx_g ] ;

	std::ofstream ofs_info( infofilename );
	std::ofstream ofs_map(mapfilename);

	std::vector<signed char> mapdata =map.data;
	ofs_info << nwidth << " " << nheight << " " << fox << " " << foy << " " << fres << " " <<
			robotpose.pose.position.x << " " << robotpose.pose.position.y << endl;

	for( int ii=0; ii < nheight; ii++ )
	{
		for( int jj=0; jj < nwidth; jj++ )
		{
			int dataidx = ii * nwidth + jj ;
			int val = static_cast<int>( mapdata[ dataidx ] ) ;
			ofs_map << val << " ";
		}
		ofs_map << "\n";
	}
	ofs_map.close();
	return 1;
}

// save prev
int FrontierDetectorDMS::saveFrontierPoints( const nav_msgs::OccupancyGrid& map, const nav_msgs::Path& msg_frontiers, int bestidx, const string& frontierfile  )
{

	int nwidth  = map.info.width ;
	int nheight = map.info.height ;
	float fox = map.info.origin.position.x ;
	float foy = map.info.origin.position.y ;
	float fres = map.info.resolution ;
	std::vector<signed char> cmdata;
	cmdata = map.data ;


	std::ofstream ofs_fpts( frontierfile );
	ofs_fpts << msg_frontiers.poses.size() << " " << bestidx << endl;
	{
		// append valid previous frontier points by sanity check
		for (const auto & pi : msg_frontiers.poses)
		{
			int ngmx = static_cast<int>( (pi.pose.position.x - fox) / fres ) ;
			int ngmy = static_cast<int>( (pi.pose.position.y - fox) / fres ) ;

			//int8_t cost = cmdata[ nwidth * ngmy + ngmx ] ;

			ofs_fpts << pi.pose.position.x << " " << pi.pose.position.y << " " << ngmx << " " << ngmy << "\n"; // << cost << "\n" ;
		}
	}

	ofs_fpts.close();

	return 1;
}

int FrontierDetectorDMS::savefrontiercands( const nav_msgs::OccupancyGrid& map, const vector<FrontierPoint>& voFrontierPoints, const string& frontierfile )
{

	int nwidth  = map.info.width ;
	int nheight = map.info.height ;
	float fox = map.info.origin.position.x ;
	float foy = map.info.origin.position.y ;
	float fres = map.info.resolution ;

	std::ofstream ofs_currfpts( frontierfile );

	for(const auto & fpt : voFrontierPoints)
	{
		ofs_currfpts <<
		fpt.GetCorrectedWorldPosition().x << " " << fpt.GetCorrectedWorldPosition().y << " " <<
		fpt.GetCorrectedGridmapPosition().x << " " << fpt.GetCorrectedGridmapPosition().y << " " <<
		fpt.GetCMConfidence() << " " << fpt.GetGMConfidence() << "\n" ;
	}
	ofs_currfpts.close();

	return 1;
}




int FrontierDetectorDMS::frontier_summary( const vector<FrontierPoint>& voFrontierCurrFrame )
{
	ROS_INFO("\n========================================================================== \n"
			  "============================Frontier Summary  =============================\n\n");
	ROS_INFO("\n\n frontier found in curr frame: \n\n");
	for(size_t idx=0; idx < voFrontierCurrFrame.size(); idx++)
	{
		FrontierPoint fpt= voFrontierCurrFrame[idx];
		if(fpt.isConfidentFrontierPoint())
		{
			ROS_INFO(" (%f %f) is (Reachablity: %s) (Frontierness: %s) \n",
				fpt.GetCorrectedWorldPosition().x, fpt.GetCorrectedWorldPosition().y, fpt.isReachable()?"true":"false", fpt.isConfidentFrontierPoint()?"true":"false" );
		}
		else
		{
			ROS_ERROR(" (%f %f) is (Reachablity: %s) (Frontierness: %s) \n",
				fpt.GetCorrectedWorldPosition().x, fpt.GetCorrectedWorldPosition().y, fpt.isReachable()?"true":"false", fpt.isConfidentFrontierPoint()?"true":"false" );
		}
	}

	ROS_INFO("\n\n Tot cumulated frontier points to process: \n\n");
	for (const auto & di : m_curr_frontier_set )
	{
			ROS_INFO("cumulated pts: (%f %f) \n", di.p[0], di.p[1]);
	}

	ROS_INFO("========================================================================== \n\n");
}


void FrontierDetectorDMS::updateUnreachablePointSet( const nav_msgs::OccupancyGrid& globalcostmap )
{
	std::vector<signed char> cmdata = globalcostmap.data ;
	{
		const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
		auto it = m_unreachable_frontier_set.begin() ;
		while (it != m_unreachable_frontier_set.end() )
		{
			auto it_element = it++;
			int ncmx = static_cast<int>( ((*it_element).p[0] - globalcostmap.info.origin.position.x) / globalcostmap.info.resolution ) ;
			int ncmy = static_cast<int>( ((*it_element).p[1] - globalcostmap.info.origin.position.y) / globalcostmap.info.resolution ) ;
			bool bis_explored = is_explored(ncmx, ncmy, globalcostmap.info.width, cmdata) ;

			if( bis_explored )
			{
				ROS_DEBUG("removing ufpt (%f %f) from the unreachable fpt list \n", (*it_element).p[0], (*it_element).p[1]) ;
				m_unreachable_frontier_set.erase(it_element);
			}
		}
	}

}


// mapcallback for dynamic mapsize (i.e for the cartographer)
void FrontierDetectorDMS::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) //const octomap_server::mapframedata& msg )
{

//ROS_INFO("start mapdata callback routine\n");

ros::WallTime	mapCallStartTime = ros::WallTime::now();
	//ROS_INFO("@ mapdataCallback() ");

	if(!mb_isinitmotion_completed)
	{
		ROS_WARN("FD has not fully instantiated yet !");
		return;
	}

	if(m_robotvel.linear.x == 0 && m_robotvel.angular.z == 0 ) // robot is physically stopped
	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::ROBOT_IS_NOT_MOVING;
	}

	nav_msgs::OccupancyGrid globalcostmap;
	float cmresolution, gmresolution, cmstartx, cmstarty, gmstartx, gmstarty;
	uint32_t cmwidth, cmheight, gmheight, gmwidth;
	std::vector<signed char> gmdata;
	std::vector<signed char> cmdata;

	{
		const std::unique_lock<mutex> lock(mutex_gridmap);
		m_gridmap = *msg ;
		gmdata = m_gridmap.data ;
		gmresolution = m_gridmap.info.resolution ;
		gmheight = m_gridmap.info.height ;
		gmwidth = m_gridmap.info.width ;
		gmstartx = m_gridmap.info.origin.position.x ;
		gmstarty = m_gridmap.info.origin.position.y ;
	}

	{
		const std::unique_lock<mutex> lock(mutex_costmap);
		globalcostmap = m_globalcostmap;
		cmresolution=globalcostmap.info.resolution;
		cmstartx=globalcostmap.info.origin.position.x;
		cmstarty=globalcostmap.info.origin.position.y;
		cmwidth =globalcostmap.info.width;
		cmheight=globalcostmap.info.height;
		cmdata  =globalcostmap.data;
	}

	if( gmheight == 0 || gmwidth == 0
		|| gmwidth  != cmwidth
		|| gmheight != cmheight)
	{
		ROS_WARN("unreliable grid map input h/w (%d, %d) gcostmap h/w (%d, %d). May be the (G)costmap has not been updated yet. \n",
				gmheight, gmwidth,
				cmheight, cmwidth);
		return;
	}

	mn_roi_origx = mn_globalmap_centx ; // - (int)round( m_gridmap.info.origin.position.x / m_fResolution ) ;
	mn_roi_origy = mn_globalmap_centy ; //- (int)round( m_gridmap.info.origin.position.y / m_fResolution ) ;
	cv::Rect roi( mn_roi_origx, mn_roi_origy, gmwidth, gmheight );

	mcvu_mapimgroi = mcvu_mapimg(roi);

	for( int ii =0 ; ii < gmheight; ii++)
	{
		for( int jj = 0; jj < gmwidth; jj++)
		{
			int8_t occupancy = m_gridmap.data[ ii * gmwidth + jj ]; // dynamic gridmap size
			int8_t obs_cost  = globalcostmap.data[ ii * cmwidth + jj] ;
			int y_ = (mn_roi_origy + ii) ;
			int x_ = (mn_roi_origx + jj) ;

			if ( occupancy < 0 && obs_cost < 0)
			{
				mcvu_mapimg.data[ y_ * mn_globalmap_width + x_ ] = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
			}
			else if( occupancy >= 0 && occupancy < mn_occupancy_thr && obs_cost < 98) // mp_cost_translation_table[51:98] : 130~252 : possibly circumscribed ~ inscribed
			{
				mcvu_mapimg.data[ y_ * mn_globalmap_width + x_ ] = static_cast<uchar>(ffp::MapStatus::FREE) ;
			}
			else
			{
				mcvu_mapimg.data[ y_ * mn_globalmap_width + x_ ] = static_cast<uchar>(ffp::MapStatus::OCCUPIED) ;
			}
		}
	}

//	m_markerfrontierpub.publish(m_points); 		// Publish frontiers to renew Rviz
//	m_makergoalpub.publish(m_exploration_goal);	// Publish frontiers to renew Rviz

// The robot is not moving (or ready to move)... we can go ahead plan the next action...
// i.e.) We locate frontier points again, followed by publishing the new goal

	cv::Mat img_ ;
	img_ = mcvu_mapimg( roi ); //m_uMapImgROI.clone();

	if( mn_numpyrdownsample > 0)
	{
		// be careful here... using pyrDown() interpolates occ and free, making the boarder area (0 and 127) to be 127/2 !!
		// 127 reprents an occupied cell !!!
		for(int iter=0; iter < mn_numpyrdownsample; iter++ )
		{
			int nrows = img_.rows; //% 2 == 0 ? img.rows : img.rows + 1 ;
			int ncols = img_.cols; // % 2 == 0 ? img.cols : img.cols + 1 ;
			//ROS_INFO("sizes orig: %d %d ds: %d %d \n", img_.rows, img_.cols, nrows/2, ncols/2 );
			pyrDown(img_, img_, cv::Size( ncols/2, nrows/2 ) );
		}
	}
	clusterToThreeLabels( img_ );

// We need to zero-pad around img b/c m_gridmap dynamically increases
// u = unk padding (offset), x = orig img contents

// u u u u u u u u
// u x x x x x x u
// u x x x x x x u
// u x x x x x x u
// u u u u u u u u

	uint8_t ukn = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
	cv::Mat img_plus_offset = cv::Mat( img_.rows + ROI_OFFSET*2, img_.cols + ROI_OFFSET*2, CV_8U, cv::Scalar(ukn) ) ;
	cv::Rect myroi( ROI_OFFSET, ROI_OFFSET, img_.cols, img_.rows );
	cv::Mat img_roi = img_plus_offset(myroi) ;
	img_.copyTo(img_roi) ;

	geometry_msgs::PoseStamped start = GetCurrRobotPose( );
	int ngmx = static_cast<int>( (start.pose.position.x - gmstartx) / gmresolution ) ;
	int ngmy = static_cast<int>( (start.pose.position.y - gmstarty) / gmresolution ) ;
	cv::Point start_gm (ngmx, ngmy);

	ffp::FrontPropagation oFP(img_plus_offset); // image uchar
	oFP.update(img_plus_offset, cv::Point(ngmx,ngmy), cv::Point(0,0) );
	oFP.extractFrontierRegion( img_plus_offset ) ;

	cv::Mat img_frontiers_offset = oFP.GetFrontierContour() ;

	cv::Mat dst_offset;
	cvtColor(img_frontiers_offset, dst_offset, cv::COLOR_GRAY2BGR);

// locate the most closest labeled points w.r.t the centroid pts

	vector<vector<cv::Point> > contours_plus_offset;
	vector<cv::Vec4i> hierarchy;
	cv::findContours( img_frontiers_offset, contours_plus_offset, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

#ifdef FD_DEBUG_MODE
	string outfilename =  m_str_debugpath + "/global_mapimg.png" ;
//	cv::imwrite( outfilename.c_str(), m_uMapImg);
//	cv::imwrite(m_str_debugpath + "/labeled_img.png", img);
//	cv::imwrite(m_str_debugpath + "/img_frontiers.png",img_frontiers);
#endif

	// update unreachable frontiers;
	updateUnreachablePointSet(  globalcostmap );

	// init fr viz markers
	visualization_msgs::Marker vizfrontier_regions;
	vizfrontier_regions = SetVizMarker( 0, visualization_msgs::Marker::ADD, 0.f, 0.f, 0.1, m_worldFrameId,	1.f, 0.f, 0.f, 0.1);
	vizfrontier_regions.type = visualization_msgs::Marker::POINTS;

	// init curr frontier point sets
	{
		const std::unique_lock<mutex> lock_prev(mutex_prev_frontier_set);
		const std::unique_lock<mutex> lock_curr(mutex_curr_frontier_set);

		// append valid previous frontier points after the sanity check
		for (const auto & pi : m_prev_frontier_set)
		{
			int ngmx = static_cast<int>( (pi.p[0] - m_gridmap.info.origin.position.x) / gmresolution ) ;
			int ngmy = static_cast<int>( (pi.p[1] - m_gridmap.info.origin.position.y) / gmresolution ) ;
			if( frontier_sanity_check(ngmx, ngmy, cmwidth, cmdata) )
			{
				pointset pnew( pi.p[0], pi.p[1] );
				m_curr_frontier_set.insert( pnew );
			}
		}
	}


	if( contours_plus_offset.size() == 0 )
	{
		ROS_WARN("There is no frontier region in this map \n");

		// we update/refresh the frontier points viz markers and publish them...
		// we don't need to proceed to planning
	}
	else // We found fr, update append new points to m_curr_frontier_set accordingly
	{
		// To publish the FR found to Rviz
		FrontierPoint oFRpoint( cv::Point(contours_plus_offset[0][0].x, contours_plus_offset[0][0].y), gmheight, gmwidth, m_gridmap.info.origin.position.y, m_gridmap.info.origin.position.x,
							   gmresolution, mn_numpyrdownsample );

		geometry_msgs::Point point_w;
		vector<cv::Point> vecents_offset;  // shifted by the offet param
		for(int i=0; i < contours_plus_offset.size(); i++)
		{
			int nx =0, ny =0 ;
			int ncnt = 0 ;
			vector<cv::Point> contour_plus_offset = contours_plus_offset[i];
			for( int j=0; j < contour_plus_offset.size(); j++)
			{

				FrontierPoint oFRpoint( cv::Point(contour_plus_offset[j].x - ROI_OFFSET, contour_plus_offset[j].y - ROI_OFFSET), gmheight, gmwidth, m_gridmap.info.origin.position.y, m_gridmap.info.origin.position.x,
									   gmresolution, mn_numpyrdownsample );

				point_w.x = oFRpoint.GetInitWorldPosition().x ;
				point_w.y = oFRpoint.GetInitWorldPosition().y ;
				vizfrontier_regions.points.push_back( point_w ) ;

				nx += contour_plus_offset[j].x ;
				ny += contour_plus_offset[j].y ;

	//			ROS_INFO("%d %d / %d %d \n", contour_plus_offset[j].x, contour_plus_offset[j].y, img_plus_offset.rows, img_plus_offset.cols );

				ncnt += 1;
			}
			int ncx = nx / ncnt ;
			int ncy = ny / ncnt ;


			cv::Point ncent( ncx,  ncy ) ;
			vecents_offset.push_back( ncent );
		}

		CV_Assert( contours_plus_offset.size() == vecents_offset.size() );
		// get closest frontier pt to each cent
		// i.e.) the final estimated frontier points
		vector<FrontierPoint> voFrontierCands;

		for( int i = 0; i < contours_plus_offset.size(); i++ )
		{
			vector<cv::Point> contour_plus_offset = contours_plus_offset[i] ;
			if(contour_plus_offset.size() < m_frontiers_region_thr ) // don't care about small frontier regions
				continue ;

			int ncentx_offset = vecents_offset[i].x ;
			int ncenty_offset = vecents_offset[i].y ;

			int nmindist = 10000000 ;
			int nmindistidx = -1;

			for (int j=0; j < contour_plus_offset.size(); j++)
			{
				int nx = contour_plus_offset[j].x ;
				int ny = contour_plus_offset[j].y ;
				int ndist = std::sqrt( (nx - ncentx_offset) * (nx - ncentx_offset) + (ny - ncenty_offset) * (ny - ncenty_offset) );
				if(ndist < nmindist)
				{
					nmindist = ndist ;
					nmindistidx = j ;
				}
			}

			CV_Assert(nmindistidx >= 0);
			cv::Point frontier_offset = contour_plus_offset[nmindistidx];

			//ROS_WARN(" %d %d \n", frontier_offset.x, frontier_offset.y);

			cv::Point frontier ;
			frontier.x = frontier_offset.x - ROI_OFFSET ;
			frontier.y = frontier_offset.y - ROI_OFFSET ;

			FrontierPoint oPoint( frontier, gmheight, gmwidth,
									m_gridmap.info.origin.position.y, m_gridmap.info.origin.position.x,
						   gmresolution, mn_numpyrdownsample );

	//oPoint.SetFrontierRegion(contour_plus_offset);
	//char outfile[10000];
	//sprintf(outfile,"/media/hankm/mydata/results/explore_bench/outpoint%04d.txt", i);
	//oPoint.saveFrontierInfo(std::string( outfile ) );

	/////////////////////////////////////////////////////////////////////
	// 				We need to run position correction here
	/////////////////////////////////////////////////////////////////////
			cv::Point init_pt 		= oPoint.GetInitGridmapPosition() ; 	// position @ ds0 (original sized map)
			cv::Point corrected_pt	= oPoint.GetCorrectedGridmapPosition() ;
			correctFrontierPosition( m_gridmap, init_pt, mn_correctionwindow_width, corrected_pt  );

			oPoint.SetCorrectedCoordinate(corrected_pt);
			voFrontierCands.push_back(oPoint);
		}

		// run filter
		const float fcm_conf = mo_frontierfilter.GetCostmapConf() ;
		const float fgm_conf = mo_frontierfilter.GetGridmapConf() ;

		// eliminate frontier points at obtacles

		if( globalcostmap.info.width > 0 )
		{
			mo_frontierfilter.measureCostmapConfidence(globalcostmap, voFrontierCands);
			mo_frontierfilter.measureGridmapConfidence(m_gridmap, voFrontierCands);

			for(size_t idx=0; idx < voFrontierCands.size(); idx++)
			{
				cv::Point frontier_in_gm = voFrontierCands[idx].GetCorrectedGridmapPosition();
				//bool bisexplored = false; //is_explored(frontier_in_gm.x, frontier_in_gm.y, gmwidth, gmdata) ;
				int gmidx = gmwidth * frontier_in_gm.y	+	frontier_in_gm.x ;
				bool bisexplored = gmdata[gmidx] >=0 ? true : false ;
				voFrontierCands[idx].SetFrontierFlag( fcm_conf, fgm_conf, bisexplored );
			}
			set<pointset, pointset> unreachable_frontiers;
			{
				const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
				unreachable_frontiers = m_unreachable_frontier_set ;
				mo_frontierfilter.computeReachability( unreachable_frontiers, voFrontierCands );
			}
		}
		else
		{
			//ROS_INFO("costmap hasn't updated \n");
			//frontiers = frontiers_cand ; // points in img coord
		}


		{
			//vector<size_t> valid_frontier_indexs;
			const std::unique_lock<mutex> lock(mutex_curr_frontier_set);
			for (size_t idx=0; idx < voFrontierCands.size(); idx++)
			{
				if( voFrontierCands[idx].isConfidentFrontierPoint() )
				{
					//valid_frontier_indexs.push_back( idx );
					cv::Point2f frontier_in_world = voFrontierCands[idx].GetCorrectedWorldPosition();
					pointset pt( frontier_in_world.x, frontier_in_world.y );
					m_curr_frontier_set.insert( pt );
				}
			}
		}
	}


#ifdef FD_DEBUG_MODE
	string strcandfile = m_str_debugpath + "/front_cand.txt" ;
	ofstream ofs_cand(strcandfile);
	for( int idx=0; idx < frontiers_cand.size(); idx++ )
	{
		ofs_cand << frontiers_cand[idx].x << " " << frontiers_cand[idx].y << endl;
	}
	ofs_cand.close();
#endif



// print frontier list
// save frontier info ;
	ROS_INFO(" The num of tot frontier points left :  %d\n", m_curr_frontier_set.size() );
	//frontier_summary( voFrontierCands );



//	static int fileidx = 0;
//	std::stringstream ssfptfile, ssmapfile ;
//	ssfptfile << "fpt" << std::setw(4) << std::setfill('0') << fileidx << ".txt" ;
//	ssmapfile << "map" << std::setw(4) << std::setfill('0') << fileidx << ".txt";
//	std::string resdir("/home/hankm/results/explore_bench/");
//	std::string strprevfrontierfile = resdir + "prev_" + ssfptfile.str() ;
//	std::string strcurrfrontierfile = resdir + "curr_" + ssfptfile.str() ;
//	std::string strmapfile			= resdir + ssmapfile.str() ;
//	std::string strmapinfofile		= resdir + "mapinfo_" + ssmapfile.str();
//
//	savemap( globalcostmap, strmapinfofile, strmapfile );
//	saveprevfrontierpoint( m_gridmap, strprevfrontierfile ) ;
//	savefrontiercands( m_gridmap, voFrontierCands, strcurrfrontierfile ) ;
//	fileidx++;
///////////////////////////////////////////////////////////////////////////////////////////////////


	if( m_curr_frontier_set.empty() )
	{
		ROS_WARN("no more valid frontiers \n");

		// refresh fpts in Rviz
		visualization_msgs::MarkerArray ftmarkers_old = m_frontier_points ;
		for(size_t idx=0; idx < ftmarkers_old.markers.size(); idx++)
			ftmarkers_old.markers[idx].action = visualization_msgs::Marker::DELETE; //SetVizMarker( idx, visualization_msgs::Marker::DELETE, 0.f, 0.f, 0.5, "map", 0.f, 1.f, 0.f );
		m_markerfrontierPub.publish(ftmarkers_old);
		m_frontier_points.markers.resize(0);

		// publish fpts to Rviz
		for (const auto & pi : m_curr_frontier_set)
		{
			visualization_msgs::Marker vizmarker = SetVizMarker( mn_FrontierID, visualization_msgs::Marker::ADD, pi.p[0], pi.p[1], (float)FRONTIER_MARKER_SIZE, m_worldFrameId, 0.f, 1.f, 0.f );
			m_frontier_points.markers.push_back(vizmarker);
			mn_FrontierID++ ;
		}
		m_markerfrontierPub.publish(m_frontier_points);

		// publish goal to Rviz
		m_exploration_goal.points.clear();
		m_exploration_goal = SetVizMarker( -1, visualization_msgs::Marker::ADD, m_targetgoal.pose.pose.position.x, m_targetgoal.pose.pose.position.y, (float)TARGET_MARKER_SIZE,
				m_worldFrameId,	1.f, 0.f, 1.f, 1.f);
		m_makergoalPub.publish(m_exploration_goal); // for viz

		mb_explorationisdone = true;
		return;
	}

#ifdef FFD_DEBUG_MODE
		imwrite(m_str_debugpath+"/frontier_cents.png", dst);
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		generate a path trajectory
// 		call make plan service
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	//ROS_INFO("resizing mpo_costmap \n");
	mpo_costmap->resizeMap( 	cmwidth, cmheight, cmresolution,
								cmstartx, cmstarty );
	//ROS_INFO("mpo_costmap has been reset \n");
	unsigned char* pmap = mpo_costmap->getCharMap() ;
	//ROS_INFO("w h datlen : %d %d %d \n", cmwidth, cmheight, cmdata.size() );

	for(uint32_t ridx = 0; ridx < cmheight; ridx++)
	{
		for(uint32_t cidx=0; cidx < cmwidth; cidx++)
		{
			uint32_t idx = ridx * cmwidth + cidx ;
			signed char val = cmdata[idx];
			pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
		}
	}

///////////////////////////////////////////////////////////////////////
// 1. estimate dist to each goal using euclidean distance heuristic (we need sorting here)
///////////////////////////////////////////////////////////////////////
	float fstartx = static_cast<float>( start.pose.position.x ) ;
	float fstarty = static_cast<float>( start.pose.position.y ) ;
	float fmindist = DIST_HIGH ;
	size_t min_heuristic_idx = 0;

//////////////////////////////////////////////////////////////////////////////////
// 2. use the fp corresponds to the min distance as the init fp. epsilon = A*(fp)
// 	i)  We first sort fpts based on their euc heuristic(), then try makePlan() for each of fpts in turn.
// 	ii) We need to sort them b/c the one with best heuristic could fail
//////////////////////////////////////////////////////////////////////////////////
#ifdef DEBUG_MODE
	static int ndebugframeidx = 0;
	// 1. save cost-map
	string strofs_mpbb = (boost::format("%s/mpbb_state%04d.txt") % m_str_debugpath % ndebugframeidx ).str() ;
	ofstream ofs_mpbb(strofs_mpbb) ;
#endif

	alignas(64) float fupperbound ;
	alignas(64) size_t best_idx;
	//alignas(64) float fXbest ;
	//alignas(64) float fYbest ;

	std::vector<geometry_msgs::PoseStamped> initplan;
	fupperbound = static_cast<float>(DIST_HIGH) ;
	best_idx	= static_cast<size_t>(0) ;

	float fendpot = 0.f; //POT_HIGH;

///////////////////////// /////////////////////////////////////////////////////////
// 3. Do BB based openmp search
//////////////////////////////////////////////////////////////////////////////////

	//set<> fpoints = m_valid_frontier_set ;
	vector< uint32_t > gplansizes( m_curr_frontier_set.size(), 0 ) ;

	GlobalPlanningHandler o_gph( *mpo_costmap, m_worldFrameId, m_baseFrameId );
	std::vector<geometry_msgs::PoseStamped> plan;
	uint32_t fptidx;
	int tid;
	geometry_msgs::PoseStamped goal;
	nav_msgs::Path msg_frontierpoints ;

	for( const auto & pi : m_curr_frontier_set)
	{
		geometry_msgs::PoseStamped tmp_goal = StampedPosefromSE2( pi.p[0], pi.p[1], 0.f );
		tmp_goal.header.frame_id = m_worldFrameId ;
		msg_frontierpoints.poses.push_back(tmp_goal);
	}


ros::WallTime GPstartTime = ros::WallTime::now();

	omp_set_num_threads(mn_numthreads);
	omp_init_lock(&m_mplock);

	//ROS_INFO("begining BB A*\n");
//	vector<cv::Point2f> cvgoalcands;
//	for (const auto & pi : m_curr_frontier_set)
//		cvgoalcands.push_back( cv::Point2f( pi.p[0], pi.p[1] ) );

	#pragma omp parallel firstprivate( o_gph, msg_frontierpoints, plan, tid, start, goal, fendpot ) shared( fupperbound,  best_idx, ofs_mpbb )
	{

		#pragma omp for
		//for (const auto & pi : m_valid_frontier_set)
		for( fptidx=0; fptidx < msg_frontierpoints.poses.size() ; fptidx++)
		{
			tid = omp_get_thread_num() ;

	//ROS_INFO("processing (%f %f) with thread %d/%d : %d", p.x, p.y, omp_get_thread_num(), omp_get_num_threads(), idx );
			//fendpot = POT_HIGH ;
			//fendpot = 0.f;
			o_gph.reinitialization( ) ;
			geometry_msgs::PoseStamped goal = msg_frontierpoints.poses[fptidx];
			goal.header.frame_id = m_worldFrameId ;
	//ROS_INFO("goal: %f %f \n", fpoints[fptidx].x, fpoints[fptidx].y );

			string str_astar  = (boost::format("/media/data/results/autoexplorer/mpbb/astar_%04d_%04d_%02d.txt") % ndebugframeidx % fptidx % tid ).str() ;

			int bplansuccess = o_gph.makePlan(str_astar, tid, fupperbound, true, start, goal, plan);
			fendpot    = o_gph.getEndPot();
			{
				omp_set_lock(&m_mplock);
				cv::Point goal_gm = world2gridmap(cv::Point2f(goal.pose.position.x, goal.pose.position.y)) ;
				ofs_mpbb << "[tid "<<tid <<"] ["<<bplansuccess << "] processed " << fptidx << " th point (" << start_gm.x << ", " << start_gm.y << ") to (" <<
														goal_gm.x << ", " << goal_gm.y << ") marked \t" << fendpot << " / " << fupperbound << " potential \n " << endl;
				omp_unset_lock(&m_mplock);
			}

			if( fendpot < fupperbound )
			{
				omp_set_lock(&m_mplock);
				fupperbound = fendpot; // set new bound;
				best_idx = fptidx;
				omp_unset_lock(&m_mplock);
			}
		}
	}

ros::WallTime GPendTime = ros::WallTime::now();
double planning_time = (GPendTime - GPstartTime ).toNSec() * 1e-6;

	// publish goalexclusive fpts
	int tmpcnt = 0;
	nav_msgs::Path goalexclusivefpts ;
	goalexclusivefpts.header.frame_id = m_worldFrameId;
	goalexclusivefpts.header.seq = m_curr_frontier_set.size() -1 ;
	goalexclusivefpts.header.stamp = ros::Time::now();
	for (const auto & pi : m_curr_frontier_set)
	{
		if (tmpcnt != best_idx)
		{
			geometry_msgs::PoseStamped fpt = StampedPosefromSE2( pi.p[0], pi.p[1], 0.f ) ;;
			fpt.header.frame_id = m_worldFrameId ;
			fpt.header.stamp = goalexclusivefpts.header.stamp ;
			goalexclusivefpts.poses.push_back(fpt);
		}
		tmpcnt++;
	}

	//cv::Point2f cvbestgoal = cvgoalcands[best_idx] ;  // just for now... we need to fix it later
	geometry_msgs::PoseStamped best_goal = msg_frontierpoints.poses[best_idx] ; //ps.p[0], ps.p[1], 0.f );
	{
		const std::unique_lock<mutex> lock(mutex_currgoal);
		m_targetgoal.header.frame_id = m_worldFrameId ;
		m_targetgoal.pose.pose = best_goal.pose ;
	}

//////////////////////////////////////////////////////////////////////////////
// 						Publish frontier pts to Rviz						//
//////////////////////////////////////////////////////////////////////////////
	// ROS_INFO("publishing out messages \n");

	// refresh fpts in Rviz
	visualization_msgs::MarkerArray ftmarkers_old = m_frontier_points ;
	for(size_t idx=0; idx < ftmarkers_old.markers.size(); idx++)
		ftmarkers_old.markers[idx].action = visualization_msgs::Marker::DELETE; //SetVizMarker( idx, visualization_msgs::Marker::DELETE, 0.f, 0.f, 0.5, "map", 0.f, 1.f, 0.f );
	m_markerfrontierPub.publish(ftmarkers_old);
	m_frontier_points.markers.resize(0);

	// publish fpt regions to Rviz
	m_markerfrontierregionPub.publish(vizfrontier_regions);

	// publish fpts to Rviz
	for (const auto & pi : m_curr_frontier_set)
	{
		visualization_msgs::Marker vizmarker = SetVizMarker( mn_FrontierID, visualization_msgs::Marker::ADD, pi.p[0], pi.p[1], (float)FRONTIER_MARKER_SIZE, m_worldFrameId, 0.f, 1.f, 0.f );
		m_frontier_points.markers.push_back(vizmarker);
		mn_FrontierID++ ;
	}
	m_markerfrontierPub.publish(m_frontier_points);

	// publish goal to Rviz
	m_exploration_goal.points.clear();
	m_exploration_goal = SetVizMarker( -1, visualization_msgs::Marker::ADD, m_targetgoal.pose.pose.position.x, m_targetgoal.pose.pose.position.y, (float)TARGET_MARKER_SIZE,
			m_worldFrameId,	1.f, 0.f, 1.f, 1.f);
	m_makergoalPub.publish(m_exploration_goal); // for viz


#ifdef DEBUG_MODE
//	static int ndebugframeidx = 0;
	// 1. save cost-map
	string strcostmap = (boost::format("%s/costmap%04d.txt") % m_str_debugpath % ndebugframeidx ).str() ;
	string strcminfo  = (boost::format("%s/cminfo%04d.txt" ) % m_str_debugpath % ndebugframeidx ).str() ;
	saveMap( globalcostmap, strcminfo, strcostmap );

	// 2. save frontier points / planning res
	string strfrontierfile = (boost::format("%s/frontier%04d.txt") % m_str_debugpath % ndebugframeidx ).str() ;

	// publish fpts to Rviz
	saveFrontierPoints( globalcostmap, msg_frontierpoints, best_idx, strfrontierfile  );
	// 3. save planning res


	ndebugframeidx++ ;
#endif


	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::ROBOT_IS_READY_TO_MOVE;
	}

	{
		const std::unique_lock<mutex> lock_curr(mutex_curr_frontier_set);
		const std::unique_lock<mutex> lock_prev(mutex_prev_frontier_set);
		set<pointset, pointset> frontier_set_tmp ;
		frontier_set_tmp = m_curr_frontier_set ;
		m_prev_frontier_set.clear() ;
		m_curr_frontier_set.clear() ;
		m_prev_frontier_set = frontier_set_tmp;
	}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Lastly we publish the goal and other frontier points ( hands over the control to move_base )
////////////////////////////////////////////////////////////////////////////////////////////////////

	m_otherfrontierptsPub.publish(goalexclusivefpts);
	m_currentgoalPub.publish(m_targetgoal);		// for control

ros::WallTime mapCallEndTime = ros::WallTime::now();
double mapcallback_time = (mapCallEndTime - mapCallStartTime).toNSec() * 1e-6;
ROS_DEBUG("\n "
		 " ************************************************************************* \n "
		 "	 \t mapDataCallback exec time (ms): %f ( %f planning time) \n "
		 " ************************************************************************* \n "
		, mapcallback_time, planning_time);

}


void FrontierDetectorDMS::gobalPlanCallback(const visualization_msgs::Marker::ConstPtr& msg)
{

}


void FrontierDetectorDMS::doneCB( const actionlib::SimpleClientGoalState& state )
{
    ROS_INFO("@DONECB: simpleClientGoalState [%s]", state.toString().c_str());

//	bool bis_target_covered = false ;
//	// The 1st thing is to check whether the current target goal is covered or not
//	{
//		const std::unique_lock<mutex> lock(mutex_currgoal) ;
//		float fx_world = m_targetgoal.pose.pose.position.x ;
//		float fy_world = m_targetgoal.pose.pose.position.y ;
//
//		std::vector<signed char> Data=m_gridmap.data;
//		int ngmx = static_cast<int>( (fx_world - m_gridmap.info.origin.position.x) / m_gridmap.info.resolution ) ;
//		int ngmy = static_cast<int>( (fy_world - m_gridmap.info.origin.position.x) / m_gridmap.info.resolution ) ;
//		int ngmwidth = static_cast<int> (m_gridmap.info.width) ;
//		if( !frontier_sanity_check(ngmx, ngmy, ngmwidth, Data) )
//			bis_target_covered = true ;
//	}

    if (m_move_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
         // do something as goal was reached
    	ROS_INFO("touch down  \n");
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			me_robotstate = ROBOT_STATE::ROBOT_IS_NOT_MOVING ;
		}
    }
    else if (m_move_client.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        // do something as goal was canceled
    	ROS_ERROR("Aboarted ... \n");
		{
			const std::unique_lock<mutex> lock(mutex_robot_state) ;
			me_robotstate = ROBOT_STATE::ROBOT_IS_NOT_MOVING ;
		}
    }
    else
    {
    	ROS_ERROR("unknown state \n");
    	exit(-1);
    }
}


void FrontierDetectorDMS::moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
// call actionlib
// robot is ready to move
	ROS_INFO("@moveRobotCallback Robot is < %s > \n ",  robot_state[me_robotstate+1] );

	if( me_robotstate >= ROBOT_STATE::FORCE_TO_STOP   )
		return;

	geometry_msgs::PoseWithCovarianceStamped goalpose = *msg ;

	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::ROBOT_IS_MOVING ;
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
//////////////////////////////////////////////////////////////////////////////////////////////
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, sending a goal +++++++++++++++++++++++++++++++++++++\n");
	m_move_client.sendGoal(goal, boost::bind(&FrontierDetectorDMS::doneCB, this, _1), SimpleMoveBaseClient::SimpleActiveCallback() ) ;
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, a goal is sent +++++++++++++++++++++++++++++++++++++\n");
	m_move_client.waitForResult();
}

void FrontierDetectorDMS::unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg )
{
	ROS_INFO("@unreachablefrontierCallback: The robot is at [%s] state\n ",  robot_state[me_robotstate+1] );

	geometry_msgs::PoseStamped unreachablepose = *msg ;
	pointset ufpt(unreachablepose.pose.position.x, unreachablepose.pose.position.y );

ROS_WARN("@unreachablefrontierCallback Registering (%f %f) as the unreachable pt \n", ufpt.p[0], ufpt.p[1] );

	// first we refresh/update viz markers
	visualization_msgs::MarkerArray ftmarkers_old = m_unreachable_points ;
	for(size_t idx=0; idx < ftmarkers_old.markers.size(); idx++)
		ftmarkers_old.markers[idx].action = visualization_msgs::Marker::DELETE; //SetVizMarker( idx, visualization_msgs::Marker::DELETE, 0.f, 0.f, 0.5, "map", 0.f, 1.f, 0.f );
	m_unreachpointPub.publish(ftmarkers_old);
	m_unreachable_points.markers.resize(0);

	// create new markers and publish them to Rviz

	for (const auto & pi : m_curr_frontier_set)
	{
		visualization_msgs::Marker vizmarker = SetVizMarker( mn_UnreachableFptID, visualization_msgs::Marker::ADD, pi.p[0], pi.p[1], (float)FRONTIER_MARKER_SIZE, m_worldFrameId, 0.f, 1.f, 0.f );
		m_unreachable_points.markers.push_back(vizmarker);
		mn_UnreachableFptID++ ;
	}
	m_unreachpointPub.publish(m_unreachable_points);

	// this case, we only check the frontier sanity check... don't care about physical path plan to the target.
	// if still valid frontier, we don't register this point to unreachable list.
	if( !mb_strict_unreachable_decision)
	{
		nav_msgs::OccupancyGrid globalcostmap;
		std::vector<signed char> cmdata;
		{
			const std::unique_lock<mutex> lock(mutex_costmap);
			globalcostmap = m_globalcostmap;
			cmdata = globalcostmap.data;
		}
		int ncmx = static_cast<int>( (ufpt.p[0] - globalcostmap.info.origin.position.x) / globalcostmap.info.resolution ) ;
		int ncmy = static_cast<int>( (ufpt.p[1] - globalcostmap.info.origin.position.y) / globalcostmap.info.resolution ) ;
		if( frontier_sanity_check(ncmx, ncmy, globalcostmap.info.width, cmdata) )
			return ;
	}

	// append the incoming unreachable fpt to the list
	{
		const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
		m_unreachable_frontier_set.insert( ufpt ) ;
		visualization_msgs::Marker viz_marker = SetVizMarker( mn_UnreachableFptID, visualization_msgs::Marker::ADD, ufpt.p[0], ufpt.p[1], (float)UNREACHABLE_MARKER_SIZE, "map",	1.f, 1.f, 0.f);
		m_unreachable_points.markers.push_back(viz_marker);
		m_unreachpointPub.publish( m_unreachable_points );
		mn_UnreachableFptID++ ;
	}

	//ROS_INFO("removing ufpt from curr_frontier_set \n") ;
// eliminate the unreachable pts from prev/curr fpt list
	{
		const std::unique_lock<mutex> lock(mutex_curr_frontier_set);
		auto it = m_curr_frontier_set.begin() ;
		while (it != m_curr_frontier_set.end() )
		{
			auto it_element = it++;
			float fdist = std::sqrt( (ufpt.p[0] - (*it_element).p[0]) * (ufpt.p[0] - (*it_element).p[0]) + (ufpt.p[1] - (*it_element).p[1]) * (ufpt.p[1] - (*it_element).p[1]) ) ;
			if( fdist < mf_neighoringpt_decisionbound )
			{
				ROS_WARN("removing ufpt (%f %f) from curr_frontier_set \n", (*it_element).p[0], (*it_element).p[1]) ;
				m_curr_frontier_set.erase(it_element);
			}
		}
	}

//ROS_INFO("removing ufpt from prev_frontier_set \n") ;

	{
		const std::unique_lock<mutex> lock(mutex_prev_frontier_set);
		auto it = m_prev_frontier_set.begin() ;
		while (it != m_prev_frontier_set.end() )
		{
			auto it_element= it++;
			float fdist = std::sqrt( (ufpt.p[0] - (*it_element).p[0]) * (ufpt.p[0] - (*it_element).p[0]) + (ufpt.p[1] - (*it_element).p[1]) * (ufpt.p[1] - (*it_element).p[1]) ) ;
			if( fdist < mf_neighoringpt_decisionbound )
			{
				ROS_WARN("removing ufpt (%f %f) from prev_frontier_set \n", (*it_element).p[0], (*it_element).p[1]) ;
				m_prev_frontier_set.erase(it_element);
			}
		}
	}


//	for (const auto & di : m_unreachable_frontier_set)
//	    ROS_WARN("unreachable pts: %f %f\n", di.d[0], di.d[1]);

	// stop the robot and restart frontier detection procedure

	ROS_WARN("+++++++++++Canceling the unreachable goal +++++++++++++\n");

	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::FORCE_TO_STOP ;
	}

	m_move_client.stopTrackingGoal();
	m_move_client.waitForResult();
	m_move_client.cancelGoal();
	m_move_client.waitForResult();

	ROS_INFO("+++++++++++ Robot is ready for motion +++++++++++++\n");
	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		me_robotstate = ROBOT_STATE::ROBOT_IS_READY_TO_MOVE ;
	}

}


}

