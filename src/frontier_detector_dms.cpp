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

#include "frontier_detector_dms.hpp"


namespace autoexplorer
{

FrontierDetectorDMS::FrontierDetectorDMS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
m_nglobalcostmapidx(0), mn_numthreads(16),
m_isInitMotionCompleted(false),
mp_cost_translation_table(NULL)
{
	float fcostmap_conf_thr, fgridmap_conf_thr ;
	m_nh.getParam("/autoexplorer/debug_data_save_path", m_str_debugpath);
	m_nh.param("/autoexplorer/costmap_conf_thr", fcostmap_conf_thr, 0.1f);
	m_nh.param("/autoexplorer/gridmap_conf_thr", fgridmap_conf_thr, 0.8f);
	m_nh.param("/autoexplorer/occupancy_thr", m_noccupancy_thr, 50);
	m_nh.param("/autoexplorer/lethal_cost_thr", m_nlethal_cost_thr, 80);
	m_nh.param("/autoexplorer/global_width", m_nGlobalMapWidth, 4000) ;
	m_nh.param("/autoexplorer/global_height", m_nGlobalMapHeight, 4000) ;
	m_nh.param("/move_base_node/global_costmap/resolution", m_fResolution, 0.05f) ;

	int _nWeakCompThreshold ;
	m_nh.param("/autoexplorer/weak_comp_thr", _nWeakCompThreshold, 10);
	m_nh.param("/autoexplorer/num_downsamples", m_nNumPyrDownSample, 0);
	m_nh.param("/autoexplorer/frame_id", m_worldFrameId, std::string("map"));
	m_nh.param("move_base_node/global_costmap/robot_radius", m_fRobotRadius, 0.3);

	m_nScale = pow(2, m_nNumPyrDownSample) ;
	m_frontiers_region_thr = _nWeakCompThreshold / m_nScale ;
	m_nROISize = static_cast<int>( round( m_fRobotRadius / m_fResolution ) ) * 2 ; // we never downsample costmap !!! dont scale it with roisize !!

	m_nGlobalMapCentX = m_nGlobalMapWidth  / 2 ;
	m_nGlobalMapCentY = m_nGlobalMapHeight / 2 ;

	m_targetspub = m_nh.advertise<geometry_msgs::PointStamped>("detected_points", 10);
	m_currentgoalpub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("curr_goalpose", 10);
	m_makergoalpub = m_nh.advertise<visualization_msgs::Marker>("curr_goal_shape",10);
	m_markercandpub = m_nh.advertise<visualization_msgs::Marker>("detected_shapes", 10);
	m_markerfrontierpub = m_nh.advertise<visualization_msgs::Marker>("filtered_shapes", 10);
	m_unreachpointpub = m_nh.advertise<visualization_msgs::Marker>("unreachable_shapes", 10);

	m_velpub		= m_nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
	m_donepub		= m_nh.advertise<std_msgs::Bool>("exploration_is_done",1);

	m_mapframedataSub  	= m_nh.subscribe("map", 1, &FrontierDetectorDMS::mapdataCallback, this); // kmHan
	m_globalplanSub 	= m_nh.subscribe("curr_goalpose",1 , &FrontierDetectorDMS::moveRobotCallback, this) ; // kmHan
	m_globalCostmapSub 	= m_nh.subscribe("move_base_node/global_costmap/costmap", 1, &FrontierDetectorDMS::globalCostmapCallBack, this );

	m_poseSub		   	= m_nh.subscribe("pose", 10, &FrontierDetectorDMS::robotPoseCallBack, this);
	m_velSub			= m_nh.subscribe("cmd_vel", 10, &FrontierDetectorDMS::robotVelCallBack, this);
	m_unreachablefrontierSub = m_nh.subscribe("unreachable_frontier", 1, &FrontierDetectorDMS::unreachablefrontierCallback, this);
	m_makeplan_client = m_nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");

	m_uMapImg  	  = cv::Mat(m_nGlobalMapHeight, m_nGlobalMapWidth, CV_8U, cv::Scalar(127));

	int ncostmap_roi_size = m_nROISize / 2 ;
	int ngridmap_roi_size = m_nROISize ;
	m_nCorrectionWindowWidth = m_nScale * 2 + 1 ; // the size of the correction search window

	m_oFrontierFilter = FrontierFilter(
			ncostmap_roi_size, ngridmap_roi_size, m_str_debugpath, m_nNumPyrDownSample,
			fgridmap_conf_thr, fcostmap_conf_thr, m_noccupancy_thr, m_nlethal_cost_thr,
			m_nGlobalMapWidth, m_nGlobalMapHeight,
			m_fResolution);
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

	SetVizMarkers( m_worldFrameId, 1.f, 0.f, 0.f, 0.2, m_cands );
	SetVizMarkers( m_worldFrameId, 0.f, 1.f, 0.f, 0.3, m_points );
	SetVizMarkers( m_worldFrameId, 1.f, 0.f, 1.f, 0.5, m_exploration_goal );
	SetVizMarkers( m_worldFrameId, 1.f, 1.f, 0.f, 0.5, m_unreachable_points );
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


cv::Point2f FrontierDetectorDMS::gridmap2world( cv::Point img_pt_roi  )
{
	float fgx =  static_cast<float>(img_pt_roi.x) * m_fResolution + m_gridmap.info.origin.position.x  ;
	float fgy =  static_cast<float>(img_pt_roi.y) * m_fResolution + m_gridmap.info.origin.position.y  ;

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
	std_msgs::Bool done_task;
	done_task.data = true;
	m_donepub.publish( done_task );
}

void FrontierDetectorDMS::publishResetGazebo( )
{
	std_msgs::Empty reset_gazebo;
	m_resetgazebopub.publish( reset_gazebo );
}


void FrontierDetectorDMS::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	const std::unique_lock<mutex> lock(mutex_costmap);
//ROS_INFO("cm callback is called \n");
	m_globalcostmap = *msg ;
	m_globalcostmap_rows = m_globalcostmap.info.height ;
	m_globalcostmap_cols = m_globalcostmap.info.width ;
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

// mapcallback for dynamic mapsize (i.e for the cartographer)
void FrontierDetectorDMS::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) //const octomap_server::mapframedata& msg )
{
	if(!m_isInitMotionCompleted)
	{
		ROS_WARN("FD has not fully instantiated yet !");
		return;
	}

	if(m_robotvel.linear.x == 0 && m_robotvel.angular.z == 0 ) // robot is physically stopped
		m_eRobotState = ROBOT_STATE::ROBOT_IS_NOT_MOVING;

	if(m_eRobotState >= ROBOT_STATE::FORCE_TO_STOP )
	{
		ROS_WARN("Force to stop flag is up cannot proceed mapdataCallback() \n");
		return;
	}
ros::WallTime mapCallStartTime, mapCallendTime;
mapCallStartTime = ros::WallTime::now();

	float gmresolution ;
	uint32_t gmheight, gmwidth;

	nav_msgs::OccupancyGrid globalcostmap;
	float cmresolution, cmstartx, cmstarty;
	uint32_t cmwidth, cmheight;
	std::vector<signed char> cmdata;

	{
		const std::unique_lock<mutex> lock(mutex_gridmap);
		m_gridmap = *msg ;
		gmresolution = m_gridmap.info.resolution ;
		gmheight = m_gridmap.info.height ;
		gmwidth = m_gridmap.info.width ;
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
		ROS_WARN("unreliable grid map input h/w (%d, %d) gcostmap h/w (%d, %d) \n",
				gmheight, gmwidth,
				cmheight, cmwidth);
		return;
	}

	m_nroi_origx = m_nGlobalMapCentX ; // - (int)round( m_gridmap.info.origin.position.x / m_fResolution ) ;
	m_nroi_origy = m_nGlobalMapCentY ; //- (int)round( m_gridmap.info.origin.position.y / m_fResolution ) ;
	cv::Rect roi( m_nroi_origx, m_nroi_origy, gmwidth, gmheight );

	m_uMapImgROI = m_uMapImg(roi);

	for( int ii =0 ; ii < gmheight; ii++)
	{
		for( int jj = 0; jj < gmwidth; jj++)
		{
			int8_t occupancy = m_gridmap.data[ ii * gmwidth + jj ]; // dynamic gridmap size
			int y_ = (m_nroi_origy + ii) ;
			int x_ = (m_nroi_origx + jj) ;

			if ( occupancy < 0 )
			{
				m_uMapImg.data[ y_ * m_nGlobalMapWidth + x_ ] = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
			}
			else if( occupancy >= 0 && occupancy < m_noccupancy_thr)
			{
				m_uMapImg.data[ y_ * m_nGlobalMapWidth + x_ ] = static_cast<uchar>(ffp::MapStatus::FREE) ;
			}
			else
			{
				m_uMapImg.data[ y_ * m_nGlobalMapWidth + x_ ] = static_cast<uchar>(ffp::MapStatus::OCCUPIED) ;
			}
		}
	}

	m_markerfrontierpub.publish(m_points); // Publish frontiers to renew Rviz
	m_makergoalpub.publish(m_exploration_goal);

// The robot is not moving (or ready to move)... we can go ahead plan the next action...
// i.e.) We locate frontier points again, followed by publishing the new goal

	cv::Mat img_ ;
	img_ = m_uMapImg( roi ); //m_uMapImgROI.clone();

	if( m_nNumPyrDownSample > 0)
	{
		// be careful here... using pyrDown() interpolates occ and free, making the boarder area (0 and 127) to be 127/2 !!
		// 127 reprents an occupied cell !!!
		for(int iter=0; iter < m_nNumPyrDownSample; iter++ )
		{
			int nrows = img_.rows; //% 2 == 0 ? img.rows : img.rows + 1 ;
			int ncols = img_.cols; // % 2 == 0 ? img.cols : img.cols + 1 ;
			//ROS_INFO("sizes orig: %d %d ds: %d %d \n", img_.rows, img_.cols, nrows/2, ncols/2 );
			pyrDown(img_, img_, cv::Size( ncols/2, nrows/2 ) );
		}
	}
	clusterToThreeLabels( img_ );

// We need to zero-pad around img b/c m_gridmap dynamically increases
	uint8_t ukn = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
	cv::Mat img = cv::Mat( img_.rows + ROI_OFFSET*2, img_.cols + ROI_OFFSET*2, CV_8U, cv::Scalar(ukn) ) ;
	cv::Rect myroi( ROI_OFFSET, ROI_OFFSET, img_.cols, img_.rows );
	cv::Mat img_roi = img(myroi) ;
	img_.copyTo(img_roi) ;

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

#ifdef FD_DEBUG_MODE
	string outfilename =  m_str_debugpath + "/global_mapimg.png" ;
//	cv::imwrite( outfilename.c_str(), m_uMapImg);
//	cv::imwrite(m_str_debugpath + "/labeled_img.png", img);
//	cv::imwrite(m_str_debugpath + "/img_frontiers.png",img_frontiers);
#endif

	if( contours.size() == 0 )
		return;

	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0;
	for( ; idx >= 0; idx = hierarchy[idx][0] )
	{
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
	vector<FrontierPoint> voFrontierCands;

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

		CV_Assert(nmindistidx >= 0);
		cv::Point frontier = contour[nmindistidx];

		if(
			(ROI_OFFSET > 0) &&
			(frontier.x <= ROI_OFFSET || frontier.y <= ROI_OFFSET ||
			 frontier.x >= gmwidth + ROI_OFFSET || frontier.y >= gmheight + ROI_OFFSET)
		   )
		{
			continue;
		}

		frontier.x = frontier.x - ROI_OFFSET ;
		frontier.y = frontier.y - ROI_OFFSET ;

		FrontierPoint oPoint( frontier, gmheight, gmwidth,
								m_gridmap.info.origin.position.y, m_gridmap.info.origin.position.x,
					   gmresolution, m_nNumPyrDownSample );

/////////////////////////////////////////////////////////////////////
// 				We need to run position correction here
/////////////////////////////////////////////////////////////////////
		cv::Point init_pt 		= oPoint.GetInitGridmapPosition() ; 	// position @ ds0 (original sized map)
		cv::Point corrected_pt	= oPoint.GetCorrectedGridmapPosition() ;
		correctFrontierPosition( m_gridmap, init_pt, m_nCorrectionWindowWidth, corrected_pt  );

		oPoint.SetCorrectedCoordinate(corrected_pt);
		voFrontierCands.push_back(oPoint);
	}

//ROS_INFO("costmap msg width: %d \n", gmwidth );

	geometry_msgs::Point p;
	m_cands.points.clear();
	m_exploration_goal.points.clear();
	m_points.points.clear();
	//	m_unreachable_points.points.clear();

	const float fcm_conf = m_oFrontierFilter.GetCostmapConf() ;
	const float fgm_conf = m_oFrontierFilter.GetGridmapConf() ;

	for(size_t idx=0; idx < voFrontierCands.size(); idx++)
	{
		cv::Point2f frontier_in_world = voFrontierCands[idx].GetCorrectedWorldPosition() ;
		p.x = frontier_in_world.x ;
		p.y = frontier_in_world.y ;
		p.z = 0.0 ;
		m_cands.points.push_back(p);
	}

	// eliminate frontier points at obtacles
	vector<size_t> valid_frontier_indexs;
	if( globalcostmap.info.width > 0 )
	{
		m_oFrontierFilter.measureCostmapConfidence(globalcostmap, voFrontierCands);
		m_oFrontierFilter.measureGridmapConfidence(m_gridmap, voFrontierCands);

		for(size_t idx=0; idx < voFrontierCands.size(); idx++)
			voFrontierCands[idx].SetFrontierFlag( fcm_conf, fgm_conf );

		set<pointset, pointset> unreachable_frontiers;
		{
			const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
			unreachable_frontiers = m_unreachable_frontier_set ;
			m_oFrontierFilter.computeReachability( unreachable_frontiers, voFrontierCands );
		}
	}
	else
	{
		//ROS_INFO("costmap hasn't updated \n");
		//frontiers = frontiers_cand ; // points in img coord
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

	for (size_t idx=0; idx < voFrontierCands.size(); idx++)
	{
		if( voFrontierCands[idx].isConfidentFrontierPoint() )
			valid_frontier_indexs.push_back( idx );
	}

	if( valid_frontier_indexs.size() == 0 )
	{
		ROS_WARN("no valid frontiers \n");
		mb_explorationisdone = true;
		return;
	}

	// set exploration goals
	for(size_t idx=0; idx < valid_frontier_indexs.size(); idx++)
	{

//ROS_INFO("frontier pts found: %d %d \n",frontier.x,frontier.y);
#ifdef FD_DEBUG_MODE
		cv::circle(dst, frontier, 3, CV_RGB(255,0,0), 2);
#endif

		// scale then conv 2 gridmap coord
		size_t vidx = valid_frontier_indexs[idx];
		cv::Point2f frontier_in_world = voFrontierCands[vidx].GetCorrectedWorldPosition();
		geometry_msgs::PoseWithCovarianceStamped mygoal ; // float64
		mygoal.header.stamp=ros::Time(0);
		mygoal.header.frame_id = m_worldFrameId;
		mygoal.pose.pose.position.x= frontier_in_world.x ;
		mygoal.pose.pose.position.y= frontier_in_world.y ;
		mygoal.pose.pose.position.z=0.0;
		//m_exploration_goal.push_back(mygoal) ;

		m_targetspub.publish(mygoal);
		p.x = mygoal.pose.pose.position.x ;
		p.y = mygoal.pose.pose.position.y ;
		p.z = 0.0 ;
		m_points.points.push_back(p);

	}

#ifdef FFD_DEBUG_MODE
		imwrite(m_str_debugpath+"/frontier_cents.png", dst);
#endif

// generate a path trajectory
// call make plan service

	geometry_msgs::PoseStamped start = GetCurrRobotPose( );

//ROS_INFO("resizing mpo_costmap \n");
mpo_costmap->resizeMap( cmwidth, cmheight, cmresolution,
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

	alignas(64) float fupperbound;
	alignas(64) size_t best_idx;

	std::vector<geometry_msgs::PoseStamped> initplan;
	fupperbound = static_cast<float>(DIST_HIGH) ;
	best_idx	= static_cast<size_t>(0) ;
	float fendpot = POT_HIGH;

	vector< uint32_t > gplansizes( m_points.points.size(), 0 ) ;
///////////////////////// /////////////////////////////////////////////////////////
// 3. Do BB based openmp search
//////////////////////////////////////////////////////////////////////////////////

std::vector<geometry_msgs::Point> fpoints = m_points.points ;

GlobalPlanningHandler o_gph( *mpo_costmap );
std::vector<geometry_msgs::PoseStamped> plan;
uint32_t fptidx;
int tid;
geometry_msgs::PoseStamped goal;

omp_set_num_threads(mn_numthreads);
omp_init_lock(&m_mplock);

ros::WallTime GPstartTime = ros::WallTime::now();

//ROS_INFO("begining BB A*\n");

#pragma omp parallel firstprivate( o_gph, fpoints, plan, tid, start, goal ) shared( fupperbound, best_idx )
{

	#pragma omp for
	for (fptidx=0; fptidx < m_points.points.size(); fptidx++)
	{
		tid = omp_get_thread_num() ;

//ROS_INFO("processing (%f %f) with thread %d/%d : %d", p.x, p.y, omp_get_thread_num(), omp_get_num_threads(), idx );
		//fendpot = POT_HIGH ;
		float fendpot;
		o_gph.reinitialization( ) ;

		geometry_msgs::PoseStamped goal = StampedPosefromSE2( fpoints[fptidx].x, fpoints[fptidx].y, 0.f );
		goal.header.frame_id = m_worldFrameId ;
//ROS_INFO("goal: %f %f \n", fpoints[fptidx].x, fpoints[fptidx].y );
		bool bplansuccess = o_gph.makePlan(tid, fupperbound, true, start, goal, plan, fendpot);

//ROS_INFO("[tid %d: [%d] ] processed %d th point (%f %f) to (%f %f) marked %f potential \n ", tid, bplansuccess, fptidx,
//										  start.pose.position.x, start.pose.position.y,
//										  goal.pose.position.x, goal.pose.position.y, fendpot);
		//gplansizes[fptidx] = myplan.size();
		if( fendpot < fupperbound )
		{
			omp_set_lock(&m_mplock);
			fupperbound = fendpot; // set new bound;
			best_idx = fptidx;
			omp_unset_lock(&m_mplock);
		}
///////////////////////////////////////////////////////////////////////////
	}
}

	std::vector<geometry_msgs::PoseStamped> best_plan ;

	p = m_points.points[best_idx];  // just for now... we need to fix it later
	geometry_msgs::PoseStamped best_goal = StampedPosefromSE2( p.x, p.y, 0.f );
	m_bestgoal.header.frame_id = m_worldFrameId ;
	m_bestgoal.pose.pose = best_goal.pose ;

	p.x = m_bestgoal.pose.pose.position.x ;
	p.y = m_bestgoal.pose.pose.position.y ;
	p.z = 0.0 ;
	m_exploration_goal.points.push_back(p);

	m_markercandpub.publish(m_cands);
	m_markerfrontierpub.publish(m_points);		// for viz
//////////////////////////////////////////////////////
// lets disable publishing goal if Fetch robot is not used
	m_currentgoalpub.publish(m_bestgoal);		// for control
/////////////////////////////////////////////////////
	m_makergoalpub.publish(m_exploration_goal); // for viz

// publish the best goal of the path plan
//	ROS_INFO("@mapDataCallback start(%f %f) found the best goal(%f %f) best_len (%u)\n",
//			start.pose.position.x, start.pose.position.y,
//			m_bestgoal.pose.pose.position.x, m_bestgoal.pose.pose.position.y, best_len);

	{
		const std::unique_lock<mutex> lock(mutex_robot_state) ;
		m_eRobotState == ROBOT_STATE::ROBOT_IS_READY_TO_MOVE;
	}

//ros::WallTime mapCallEndTime = ros::WallTime::now();

//double execution_time = (mapCallEndTime - mapCallStartTime).toNSec() * 1e-6;
//ROS_INFO("\n "
//		 " ************************************************************************* \n "
//		 "	 \t mapDataCallback exec time (ms): %f \n "
//		 " ************************************************************************* \n "
//		, execution_time);

}

void FrontierDetectorDMS::doneCB( const actionlib::SimpleClientGoalState& state )
{
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    if (m_move_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
         // do something as goal was reached
    	ROS_INFO("touch down  \n");
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


void FrontierDetectorDMS::moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
// call actionlib
// robot is ready to move
	ROS_INFO("Robot state in moveRobotCallback: %d \n ",  m_eRobotState);

	if( m_eRobotState >= ROBOT_STATE::FORCE_TO_STOP   )
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
//////////////////////////////////////////////////////////////////////////////////////////////
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, sending a goal +++++++++++++++++++++++++++++++++++++\n");
	m_move_client.sendGoal(goal, boost::bind(&FrontierDetectorDMS::doneCB, this, _1), SimpleMoveBaseClient::SimpleActiveCallback() ) ;
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, a goal is sent +++++++++++++++++++++++++++++++++++++\n");
	m_move_client.waitForResult();
}

void FrontierDetectorDMS::unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg )
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
		m_unreachpointpub.publish( m_unreachable_points );
	}

	for (const auto & di : m_unreachable_frontier_set)
	    ROS_WARN("unreachable pts: %f %f\n", di.d[0], di.d[1]);

	// stop the robot and restart frontier detection procedure

	ROS_WARN("+++++++++++Canceling the unreachable goal +++++++++++++\n");

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


}

