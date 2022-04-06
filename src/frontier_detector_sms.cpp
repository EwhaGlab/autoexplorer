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
 *  frontier_detector_sms.cpp is still on dev. It is for static map generated from Gmapping SLAM
 *  Created on: Apr, 2022
 *      Author: Kyungmin Han (hankm@ewha.ac.kr)
*/


#include "frontier_detector_sms.hpp"

namespace autoexplorer
{

FrontierDetectorSMS::FrontierDetectorSMS(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_):
m_nh_private(private_nh_),
m_nh(nh_),
m_nglobalcostmapidx(0),
m_isInitMotionCompleted(false)
//m_worldFrameId("map"), m_baseFrameId("base_link"),
//m_globalcostmap_rows(0), m_globalcostmap_cols(0), m_eRobotState(ROBOT_STATE::ROBOT_IS_NOT_MOVING),
//m_move_client("move_base", true),
//m_fRobotRadius(0.3), isdone(false), m_nroi_origx(0), m_nroi_origy(0), m_nrows(0), m_ncols(0),
//m_str_inputparams( inputparams )
//m_move_client("move_base")
{
	// gridmap generated from octomap might be downsampled !!

	float fcostmap_conf_thr, fgridmap_conf_thr ;
	m_nh.getParam("/autoexplorer/debug_data_save_path", m_str_debugpath);
	m_nh.param("/autoexplorer/costmap_conf_thr", fcostmap_conf_thr, 0.1f);
	m_nh.param("/autoexplorer/gridmap_conf_thr", fgridmap_conf_thr, 0.8f);
	m_nh.param("/autoexplorer/occupancy_thr", m_noccupancy_thr, 50);
	m_nh.param("/autoexplorer/lethal_cost_thr", m_nlethal_cost_thr, 80);
	m_nh.param("/autoexplorer/global_width", m_nGlobalMapWidth, 4000) ;
	m_nh.param("/autoexplorer/global_height", m_nGlobalMapHeight, 4000) ;
	m_nh.param("/move_base_node/global_costmap/resolution", m_fResolution, 0.05f) ;

	int _nWeakCompThreshold; //	= m_fs["WEAK_COMP_THR"];

	m_nh.param("/autoexplorer/weak_comp_thr", _nWeakCompThreshold, 10);
	m_nh.param("/autoexplorer/num_downsamples", m_nNumPyrDownSample, 0);
	m_nh.param("/autoexplorer/frame_id", m_worldFrameId, m_worldFrameId);
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
	m_donepub		= m_nh.advertise<std_msgs::Bool>("move_base_simple/mapping_is_done",1);

	//---------------------------------------------------------------
	//m_mapsub = m_nh.subscribe("map", 1, &FrontierDetectorDMS::gridmapCallBack, this);  // "projected_map" if octomap is on
	m_mapframedataSub  	= m_nh.subscribe("map", 1, &FrontierDetectorSMS::mapdataCallback, this); // kmHan
	//m_globalplanSub 	= m_nh.subscribe("move_base_node/NavfnROS/plan",1 , &FrontierDetectorDMS::moveRobotCallback, this) ; // kmHan
	m_globalplanSub 	= m_nh.subscribe("curr_goalpose",1 , &FrontierDetectorSMS::moveRobotCallback, this) ; // kmHan
	m_globalCostmapSub 	= m_nh.subscribe("move_base_node/global_costmap/costmap", 1, &FrontierDetectorSMS::globalCostmapCallBack, this );
	m_globalCostmapUpdateSub
				= m_nh.subscribe("move_base_node/global_costmap/costmap_updates", 1, &FrontierDetectorSMS::globalCostmapUpdateCallback, this );

	m_poseSub		   	= m_nh.subscribe("pose", 10, &FrontierDetectorSMS::robotPoseCallBack, this);
	m_velSub			= m_nh.subscribe("cmd_vel", 10, &FrontierDetectorSMS::robotVelCallBack, this);
	m_unreachablefrontierSub = m_nh.subscribe("unreachable_frontier", 1, &FrontierDetectorSMS::unreachablefrontierCallback, this);
	m_makeplan_client = m_nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");
ROS_WARN("allocating map buff \n");
ROS_WARN("nscale: %d \n", m_nScale);

	m_uMapImg  	  = cv::Mat::zeros(m_nGlobalMapHeight, m_nGlobalMapWidth, CV_8U);

	int ncostmap_roi_size = m_nROISize / 2 ;
	int ngridmap_roi_size = m_nROISize ;
	m_nCorrectionWindowWidth = m_nScale * 2 + 1 ; // the size of the correction search window

	m_oFrontierFilter = FrontierFilter(
			ncostmap_roi_size, ngridmap_roi_size, m_str_debugpath, m_nNumPyrDownSample,
			fgridmap_conf_thr, fcostmap_conf_thr, m_noccupancy_thr, m_nlethal_cost_thr,
			m_nGlobalMapWidth, m_nGlobalMapHeight,
			m_fResolution);

	while(!m_move_client.waitForServer(ros::Duration(5.0)))
	//while(!m_move_client.waitForActionServerToStart())
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
ROS_WARN("move_base action server is up");


// Set markers

	SetVizMarkers( m_worldFrameId, 1.f, 0.f, 0.f, 0.2, m_cands );
	SetVizMarkers( m_worldFrameId, 0.f, 1.f, 0.f, 0.3, m_points );
	SetVizMarkers( m_worldFrameId, 1.f, 0.f, 1.f, 0.5, m_exploration_goal );
	SetVizMarkers( m_worldFrameId, 1.f, 1.f, 0.f, 0.5, m_unreachable_points );


//	m_cands.header.frame_id= m_worldFrameId;
//	m_cands.header.stamp=ros::Time(0);
//	m_cands.ns= "markers";
//	m_cands.id = 0;
//	m_cands.type = m_cands.POINTS;
//
//	m_cands.action = m_cands.ADD;
//	m_cands.pose.orientation.w =1.0;
//	m_cands.scale.x=0.2;
//	m_cands.scale.y=0.2;
//
//	m_cands.color.r = 255.0/255.0;
//	m_cands.color.g = 0.0/255.0;
//	m_cands.color.b = 0.0/255.0;
//	m_cands.color.a=1.0;
//	m_cands.lifetime = ros::Duration();
//
//	m_points.header.frame_id= m_worldFrameId;
//	m_points.header.stamp=ros::Time(0);
//	m_points.ns= "markers";
//	m_points.id = 0;
//	m_points.type = m_points.POINTS;
//
//	//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
//	m_points.action = m_points.ADD;
//	m_points.pose.orientation.w =1.0;
//	m_points.scale.x= 0.3;
//	m_points.scale.y= 0.3;
//
//	m_points.color.r = 0.0/255.0;
//	m_points.color.g = 255.0/255.0;
//	m_points.color.b = 0.0/255.0;
//	m_points.color.a = 1.0;
//	m_points.lifetime = ros::Duration();
//
//	// set the goal marker
//	m_exploration_goal.header.frame_id= m_worldFrameId;
//	m_exploration_goal.header.stamp=ros::Time(0);
//	m_exploration_goal.ns= "markers";
//	m_exploration_goal.id = 0;
//	m_exploration_goal.type = m_exploration_goal.POINTS;
//
//	m_exploration_goal.action = m_exploration_goal.ADD;
//	m_exploration_goal.pose.orientation.w =1.0;
//	m_exploration_goal.scale.x=0.5;
//	m_exploration_goal.scale.y=0.5;
//	m_exploration_goal.color.r = 255.0/255.0;
//	m_exploration_goal.color.g = 0.0/255.0;
//	m_exploration_goal.color.b = 255.0/255.0;
//	m_exploration_goal.color.a = 1.0;
//	m_exploration_goal.lifetime = ros::Duration();
//
//	// set invalid frontier marker
//	m_unreachable_points.header.frame_id= m_worldFrameId;
//	m_unreachable_points.header.stamp=ros::Time(0);
//	m_unreachable_points.ns= "markers";
//	m_unreachable_points.id = 0;
//	m_unreachable_points.type = m_unreachable_points.POINTS;
//
//	m_unreachable_points.action = m_unreachable_points.ADD;
//	m_unreachable_points.pose.orientation.w =1.0;
//	m_unreachable_points.scale.x=0.5;
//	m_unreachable_points.scale.y=0.5;
//	m_unreachable_points.color.r = 1.0;
//	m_unreachable_points.color.g = 1.0;
//	m_unreachable_points.color.b = 0.0; //255.0/255.0;
//	m_unreachable_points.color.a = 1.0;
//	m_unreachable_points.lifetime = ros::Duration();

}

FrontierDetectorSMS::~FrontierDetectorSMS()
{
	//delete m_poFrontierFilter ;
}

void FrontierDetectorSMS::initmotion( )
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


cv::Point2f FrontierDetectorSMS::gridmap2world( cv::Point img_pt_ds0  )
{
	CV_Assert( m_nGlobalMapWidth == m_gridmap.info.width ); // this is not valid when the gridmap size changes

	float fResolution=m_gridmap.info.resolution;
//	float fXstart =m_gridmap.info.origin.position.x  ;
//	float fYstart =m_gridmap.info.origin.position.y  ;

	float fpx = static_cast<float>(img_pt_ds0.x) - m_nGlobalMapCentX;
	float fpy = static_cast<float>(img_pt_ds0.y) - m_nGlobalMapCentY;

//	ROS_INFO("%f %f %f %d %d\n", fpx, fResolution, fXstart, m_nScale, m_nNumPyrDownSample );
//	ROS_INFO("%f %f %f %d \n", fpy, fResolution, fYstart, m_nScale );
	float fgx = ( fpx * fResolution ); // + fXstart ) ;
	float fgy = ( fpy * fResolution ); // - fYstart ) ;

	return cv::Point2f( fgx, fgy );
}
//
cv::Point FrontierDetectorSMS::world2gridmap( cv::Point2f grid_pt)
{
	float fResolution=m_gridmap.info.resolution;

	float fx = grid_pt.x / fResolution + m_nGlobalMapCentX;
	float fy = grid_pt.y / fResolution + m_nGlobalMapCentY;

	int nx_ds0 = static_cast<int>( fx );
	int ny_ds0 = static_cast<int>( fy );

	return cv::Point2f( nx_ds0, ny_ds0 );
}

//
//vector<cv::Point> FrontierDetectorSMS::eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize)
//{
//ROS_INFO("eliminating suprious frontiers \n");
//	//ROS_INFO("in func width: %d %d %f\n", m_globalcostmap.info.width, m_globalcostmap_cols, m_globalcostmap.info.resolution);
//
//	vector<cv::Point> outFrontiers ;
//
//	cv::Point outFrontier;
//	for(size_t idx=0; idx < frontierCandidates.size(); idx++)
//	{
//		correctFrontierPosition( m_gridmap, frontierCandidates[idx], 5, outFrontier  );
//	}
//
//	float fXstartx=costmapData.info.origin.position.x; // world coordinate in the costmap
//	float fXstarty=costmapData.info.origin.position.y; // world coordinate in the costmap
//	float resolution = costmapData.info.resolution ;
//
//	int width= static_cast<int>(costmapData.info.width) ;
//	int height= static_cast<int>(costmapData.info.height) ;
//
//	std::vector<signed char> Data=costmapData.data;
//ROS_INFO("front cand size: %d \n", frontierCandidates.size());
//
//ROS_INFO("cost map: (%f %f %d %d) gridmap: (%f %f %d %d)",
//									 costmapData.info.origin.position.x, costmapData.info.origin.position.y,
//									 costmapData.info.width, costmapData.info.height,
//									 m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y,
//									 m_gridmap.info.width, m_gridmap.info.height);
//
//#ifdef FD_DEBUG_MODE
//	m_nglobalcostmapidx++;
//
//	char tmp0[200], tmp1[200], tmp2[200];
//	sprintf(tmp0, "%s/front_in_costmap%05d.txt", m_str_debugpath.c_str(), m_nglobalcostmapidx) ;
//	sprintf(tmp1, "%s/costmap%05d.txt", m_str_debugpath.c_str(), m_nglobalcostmapidx) ;
//	sprintf(tmp2, "%s/gridmap%05d.txt", m_str_debugpath.c_str(), m_nglobalcostmapidx) ;
//
//	string str_front_in_costmap(tmp0);
//	string str_costmap_file(tmp1);
//	string str_gridmap_file(tmp2);
//
//	//ROS_INFO("saving %s\n", str_front_in_costmap.c_str());
//
//	ofstream ofs_incostmap(str_front_in_costmap) ;
//	ofstream ofs_costmap(str_costmap_file) ;
////	ofstream ofs_gridmap(str_gridmap_file) ;
//
//	for(int ridx = 0; ridx < height; ridx++)
//	{
//		for(int cidx = 0; cidx < width; cidx++)
//		{
//			int cost = static_cast<int>( Data[ridx * width + cidx] ) ;
//			ofs_costmap << cost << " ";
////			ofs_gridmap << m_gridmap.data[ ridx * width + cidx ] << " ";
//	//		costmap.data[ridx * width + cidx] = cost ;
////			if(cost < 0 )
////				ofs_costmap << 127 << " ";
////			else if(cost == 0)
////				ofs_costmap << 0 << " ";
////			else
////				ofs_costmap << cost * 2 + 50 << " ";
//		}
//		ofs_costmap << endl;
////		ofs_gridmap << endl;
//	}
//	ofs_costmap.close();
////	ofs_gridmap.close();
//#endif
//
//
//	for( size_t idx =0; idx < frontierCandidates.size(); idx++) // frontiers in image coord
//	{
//		cv::Point2f frontier_in_Gridmap = img2gridmap( frontierCandidates[idx] * m_nScale );
//		//returns grid value at "Xp" location
//		//map data:  100 occupied      -1 unknown       0 free
////ROS_INFO("frontier in gridmap: %f %f origin: %f %f\n", frontier_in_Gridmap.x, frontier_in_Gridmap.y, fXstarty, fXstartx );
//		int py_c= floor( (frontier_in_Gridmap.y - fXstarty ) / resolution  ) ; // px in costmap
//		int px_c= floor( (frontier_in_Gridmap.x - fXstartx ) / resolution  ) ;
//		int8_t cost ;
//		int32_t ncost = 0;
//
//		int sx = MAX(px_c - winsize, 0);
//		int ex = MIN(px_c + winsize, width) ;
//		int sy = MAX(py_c - winsize, 0);
//		int ey = MIN(py_c + winsize, height) ;
//
//#ifdef FD_DEBUG_MODE
////	std::ostringstream ss_fptroi_;
////	ss_fptroi_ = m_str_debugpath + "/costmap" << std::setw(5)
////			 					   << std::setfill(0) <<  m_nglobalcostmapidx << "_candpt_"
////								   << std::setw(5) << std::setfill(0) <<  idx << ".txt" ;
//	char tmp[200];
//	sprintf(tmp,"%s/costmap%05d_candptroi_%05d.txt",m_str_debugpath.c_str(),m_nglobalcostmapidx, idx);
//	std::string str_fptroi( tmp );
//
//	ofstream ofs_fptroi(str_fptroi) ;
//	ofs_incostmap << px_c << " " << py_c << " " ;
//
//#endif
////ROS_INFO(" idx px py %u %d %d\n", idx, px_c, py_c);
//		cv::Mat roi = cv::Mat::zeros(ey - sy + 1, ex - sx + 1, CV_8S);
//
//		int costcnt = 0;
//		int totcost = 0;
//		for( int ridx =sy; ridx < ey; ridx++)
//		{
//			for( int cidx=sx; cidx < ex; cidx++)
//			{
//				//int dataidx = px_c + cidx + (py_c + ridx) * width ;
//				int dataidx = ridx * width + cidx ;
////ROS_INFO("ind rix cidx %d %d %d ", idx, ridx, cidx);
//				cost = Data[dataidx] ; // orig 0 ~ 254 --> mapped to 0 ~ 100
//				if(cost >= 0 )// m_nlethal_cost_thr) //LEATHAL_COST_THR ) // unknown (-1)
//				{
//					//ncost++;
//					totcost += static_cast<int>(cost);
//				}
//#ifdef FD_DEBUG_MODE
//	ofs_fptroi << cost << " ";
//#endif
//				//roi.data[ ridx * width + cidx ] = cost ;
//				costcnt++;
//			}
//
//#ifdef FD_DEBUG_MODE
//	ofs_fptroi << endl;
//#endif
//
//		}
//		float fcost = static_cast<float>(totcost) / ( static_cast<float>( costcnt ) * 100  );
////		float fcost = static_cast<float>(ncost) / static_cast<float>( (winsize*2) * (winsize*2) ) ;
//		if( fcost < m_frontier_cost_thr  ) // 0 ~ 1
//		{
//			// move back to image coord
//			//ROS_INFO("pts in cost map:  (%d  %d) in gridmap (%f %f) frontierCandidates: (%d  %d) res: %f  sx sy (%f %f)\n",
////					px_c, py_c, frontier_in_Gridmap.x, frontier_in_Gridmap.y, frontierCandidates[idx].x, frontierCandidates[idx].y,
////					resolution, fXstartx, fXstarty ) ;
//
//			cv::Point frontier_in_img = gridmap2img( frontier_in_Gridmap ) ;
////ROS_INFO("in gridmap: %d %d in image: %f %f\n", frontier_in_Gridmap.x, frontier_in_Gridmap.y, frontier_in_img.x/m_nScale, frontier_in_img.y/m_nScale );
//			outFrontiers.push_back( cv::Point(frontier_in_img.x / m_nScale, frontier_in_img.y / m_nScale)  );
//		}
//
//#ifdef FD_DEBUG_MODE
//		ofs_fptroi.close();
//		ofs_incostmap.close();
//#endif
//
//	}
//
////ROS_INFO("eliminating unreachable pts \n" ) ;
//	// Eliminate unreachable frontiers
//
//	set<pointset, pointset> unreachable_frontiers;
//	{
//		const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
//		unreachable_frontiers = m_unreachable_frontier_set ;
//	}
//	if( !unreachable_frontiers.empty() )
//	{
//		vector<cv::Point> validFrontiers ;
//		cv::Point pt_img;
//		for (size_t i=0; i < outFrontiers.size(); i++ )
//		{
//			bool isValidPt = true ;
//			pt_img = outFrontiers[i];
//			cv::Point2f pt = img2gridmap( pt_img * m_nScale );
//			float fx = pt.x ;
//			float fy = pt.y ;
//			for (const auto & di : unreachable_frontiers)
//			{
//				float fdist = std::sqrt( (fx - di.d[0]) * (fx - di.d[0]) + (fy - di.d[1]) * (fy - di.d[1]) ) ;
//				if(fdist < 0.05)
//				{
//					ROS_WARN("(%f %f) and (%f %f) are the same? unreachable point? \n",
//								fx, fy, di.d[0], di.d[1]);
//					isValidPt = false;
//				}
//			}
//			if(isValidPt)
//			{
//				validFrontiers.push_back(pt_img);
//			}
//			else
//			{
//				cv::Point2f frontier_in_Gridmap = img2gridmap( pt_img * m_nScale );
//				geometry_msgs::Point p;
//				p.x = frontier_in_Gridmap.x ;
//				p.y = frontier_in_Gridmap.y ;
//				p.z = 0.0 ;
//				m_unreachable_points.points.push_back(p) ;
//				//ROS_ERROR("(%f %f) is in the unreachable goal list \n", pt.x, pt.y);
//			}
//		}
//
//		return validFrontiers ;  // in image coord frame
//	}
//	else
//	{
//		return outFrontiers;
//	}
//}
//

//int FrontierDetectorSMS::displayMapAndFrontiers( const cv::Mat& mapimg, const vector<cv::Point>& frontiers, const int winsize)
//{
//	//ROS_INFO("weird maprows: %d %d\n", mapimg.rows, mapimg.cols);
//	if(		mapimg.empty() ||
//			mapimg.rows == 0 || mapimg.cols == 0 || m_globalcostmap.info.width == 0 || m_globalcostmap.info.height == 0)
//		return 0;
//
//	float fXstartx=m_globalcostmap.info.origin.position.x; // world coordinate in the costmap
//	float fXstarty=m_globalcostmap.info.origin.position.y; // world coordinate in the costmap
//	float resolution = m_globalcostmap.info.resolution ;
//	int cmwidth= static_cast<int>(m_globalcostmap.info.width) ;
//	//auto Data=m_globalcostmap.data ;
//
//	int x = winsize ;
//	int y = winsize ;
//	int width = mapimg.cols  ;
//	int height= mapimg.rows  ;
//	cv::Mat img = cv::Mat::zeros(height + winsize*2, width + winsize*2, CV_8UC1);
//
////ROS_INFO("costmap size: %d %d \n",  m_globalcostmap.info.width, m_globalcostmap.info.height);
//
//	cv::Mat tmp = img( cv::Rect( x, y, width, height ) ) ;
//
//ROS_INFO("mapsize: %d %d %d %d %d %d\n", tmp.cols, tmp.rows, mapimg.cols, mapimg.rows, img.cols, img.rows );
//
//	mapimg.copyTo(tmp);
//	cv::Mat dst;
//	cvtColor(tmp, dst, cv::COLOR_GRAY2BGR);
//
//	for( size_t idx = 0; idx < frontiers.size(); idx++ )
//	{
//		int x = frontiers[idx].x - winsize ;
//		int y = frontiers[idx].y - winsize ;
//		int width = winsize * 2 ;
//		int height= winsize * 2 ;
//		cv::rectangle( dst, cv::Rect(x,y,width,height), cv::Scalar(0,255,0) );
//	}
//
//	cv::Mat dstroi = dst( cv::Rect( mapimg.cols/4, mapimg.rows/4, mapimg.cols/2, mapimg.rows/2 ) );
//	cv::pyrDown(dstroi, dstroi, cv::Size(dstroi.cols/2, dstroi.rows/2) );
//
//#ifdef FD_DEBUG_MODE
////	cv::namedWindow("mapimg", 1);
////	cv::imshow("mapimg",dstroi);
////	cv::waitKey(30);
//#endif
//}

bool FrontierDetectorSMS::isValidPlan( vector<cv::Point>  )
{

}

void FrontierDetectorSMS::publishDone( )
{
	std_msgs::Bool done_task;
	done_task.data = true;
	m_donepub.publish( done_task );
}

void FrontierDetectorSMS::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	m_globalcostmap = *msg ;
	m_globalcostmap_rows = m_globalcostmap.info.height ;
	m_globalcostmap_cols = m_globalcostmap.info.width ;
}

void FrontierDetectorSMS::globalCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg )
{
	map_msgs::OccupancyGridUpdate cm_updates = *msg;
	std::vector<signed char> Data=cm_updates.data;
	int x = cm_updates.x ;
	int y = cm_updates.y ;
	int height = cm_updates.height ;
	int width  = cm_updates.width ;
	int cmwidth = m_globalcostmap.info.width ;
	int dataidx = 0 ;
//	ROS_INFO("cm updates size: %d \n", Data.size() );
//	ROS_INFO("info: x y h w (%d %d %d %d) \n", cm_updates.x, cm_updates.y,
//				cm_updates.height, cm_updates.width  );
	{
		//const std::unique_lock<mutex> lock(mutex_global_costmap) ;

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


void FrontierDetectorSMS::robotPoseCallBack( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	m_robotpose = *msg ;
	//ROS_INFO("callback pose:  %f %f \n", m_robotpose.pose.pose.position.x, m_robotpose.pose.pose.position.y);
}

void FrontierDetectorSMS::robotVelCallBack( const geometry_msgs::Twist::ConstPtr& msg )
{
	m_robotvel = *msg ;
}

void FrontierDetectorSMS::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) //const octomap_server::mapframedata& msg )
{

	if(m_robotvel.linear.x == 0 && m_robotvel.angular.z == 0 ) // robot is physically stopped
		m_eRobotState = ROBOT_STATE::ROBOT_IS_NOT_MOVING;

	ROS_INFO("Robot state in mapdataCallback: %d \n ",  m_eRobotState);

	if(!m_isInitMotionCompleted)
	{
		ROS_WARN("FD has not fully instantiated yet !");
		return;
	}

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

	for( int ii =0 ; ii < nrows; ii++)
	{
		for( int jj = 0; jj < ncols; jj++)
		{
			int8_t occupancy = m_gridmap.data[ ii * ncols + jj ];

			if ( occupancy < 0 )
			{
				m_uMapImg.data[ii * ncols + jj] = static_cast<uchar>(ffp::MapStatus::UNKNOWN) ;
			}
			else if( occupancy == 0 )
			{
				m_uMapImg.data[ii * ncols + jj] = static_cast<uchar>(ffp::MapStatus::FREE) ;
			}
			else
			{
				m_uMapImg.data[ii * ncols + jj] = static_cast<uchar>(ffp::MapStatus::OCCUPIED) ;
			}
		}
	}

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

#ifdef FD_DEBUG_MODE
	//cv::imwrite(m_str_debugpath+"/input_img.png", m_uMapImg);
	//cv::imwrite(m_str_debugpath+"/labeled_img.png", img);
	//cv::imwrite(m_str_debugpath+"/img_frontiers.png",img_frontiers);
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

	vector<FrontierPoint> voFrontierCands;

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

		CV_Assert(nmindistidx >= 0);
		cv::Point frontier = contour[nmindistidx];
		//frontiers_cand.push_back(frontier) ;
		FrontierPoint oPoint( frontier, m_gridmap.info.height, m_gridmap.info.width,
					   //m_nGlobalMapCentY, m_nGlobalMapCentX,
					   m_gridmap.info.origin.position.y, m_gridmap.info.origin.position.x,
					   m_gridmap.info.resolution, m_nNumPyrDownSample );

// //////////////////////////////////////////////////////////////////
// 				We need to run position correction here
/////////////////////////////////////////////////////////////////////
		cv::Point init_pt 		= oPoint.GetInitGridmapPosition() ; 	// position @ ds0 (original sized map)
		cv::Point corrected_pt	= oPoint.GetCorrectedGridmapPosition() ;
		correctFrontierPosition( m_gridmap, init_pt, m_nCorrectionWindowWidth, corrected_pt  );

		oPoint.SetCorrectedCoordinate(corrected_pt);
		voFrontierCands.push_back(oPoint);
	}

// save gridmap and frontier candidates
//ROS_INFO("costmap msg width: %d \n", m_globalcostmap.info.width );

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
		//ROS_INFO("frontier cands: %f %f \n", p.x, p.y);
	}

	// eliminate frontier points at obtacles
	vector<int> valid_frontier_indexs;
	if( m_globalcostmap.info.width > 0 )
	{
ROS_INFO( "eliminating supurious frontiers \n" );
		//frontiers = eliminateSupriousFrontiers( m_globalcostmap, frontiers_cand, m_nROISize) ;

		m_oFrontierFilter.measureCostmapConfidence(m_globalcostmap, voFrontierCands);
		m_oFrontierFilter.measureGridmapConfidence(m_gridmap, 		voFrontierCands);

		for(size_t idx=0; idx < voFrontierCands.size(); idx++)
			voFrontierCands[idx].SetFrontierFlag( fcm_conf, fgm_conf );

		set<pointset, pointset> unreachable_frontiers;
		{
			const std::unique_lock<mutex> lock(mutex_unreachable_points) ;
			unreachable_frontiers = m_unreachable_frontier_set ;
			m_oFrontierFilter.computeReachability( unreachable_frontiers, voFrontierCands );

			for( size_t idx=0; idx < voFrontierCands.size(); idx++)
			{
				if( !voFrontierCands[idx].isConfidentFrontierPoint() )
					continue ;

				cv::Point frontier_in_gridmap = voFrontierCands[idx].GetCorrectedGridmapPosition();
				geometry_msgs::Point p;
				p.x = frontier_in_gridmap.x ;
				p.y = frontier_in_gridmap.y ;
				p.z = 0.0 ;
				m_unreachable_points.points.push_back(p) ;
			}
		}
	}
	else
	{
		ROS_INFO("costmap hasn't updated \n");
		//frontiers = frontiers_cand ; // points in img coord
	}

static int mapidx = 0;
char cgridmapfile[100];
char costmapfile[100];
char cfptfile[100];
sprintf(cgridmapfile,"%s/tmp/gm%04d.txt", m_str_debugpath.c_str(), mapidx );
std::string strgridmapfile(cgridmapfile) ;

sprintf(costmapfile,"%s/tmp/cm%04d.txt", m_str_debugpath.c_str(), mapidx );
std::string strcostmapfile(costmapfile) ;

sprintf(cfptfile,"%s/tmp/fpt%04d.txt", m_str_debugpath.c_str(), mapidx );
std::string strfptfile(cfptfile) ;

saveGridmap( strgridmapfile, m_gridmap ) ;
saveGridmap( strcostmapfile, m_globalcostmap ) ;
saveFrontierCandidates( strfptfile, voFrontierCands );
mapidx++;


	for (size_t idx=0; idx < voFrontierCands.size(); idx++)
	{
		if( voFrontierCands[idx].isConfidentFrontierPoint() )
			valid_frontier_indexs.push_back( idx );
	}

//	for(int idx=0; idx < frontiers.size(); idx++)
//		ROS_INFO("Valid frontier points: %f %f \n", frontiers[idx].x, frontiers[idx].y);

	if( valid_frontier_indexs.size() == 0 )
	{
		ROS_WARN("no valid frontiers \n");
		//isdone = true;
		return;
	}

//	if(img_frontiers.rows > 0 && img_frontiers.cols > 0 )
//	{
//		displayMapAndFrontiers( img_frontiers, frontiers, m_nROISize );
//	}

	// set exploration goals
	for(size_t idx=0; idx < valid_frontier_indexs.size(); idx++)
	{
		//cv::Point frontier = frontiers[idx] ;
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

//		ROS_INFO("frontier in img: (%d %d) in gridmap: (%f %f) scale: %d\n",
//				frontier.x, frontier.y, p.x, p.y, m_nScale );

	}

#ifdef FD_DEBUG_MODE
		//imwrite(m_str_debugpath+"/frontier_cents.png", dst);
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


void FrontierDetectorSMS::doneCB( const actionlib::SimpleClientGoalState& state )
{
    ROS_INFO("DONECB: Finished in state [%s]", state.toString().c_str());
    if (m_move_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
         // do something as goal was reached
    	ROS_INFO("touch down \n");
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


void FrontierDetectorSMS::moveRobotCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
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
	m_move_client.sendGoal(goal, boost::bind(&FrontierDetectorSMS::doneCB, this, _1), SimpleMoveBaseClient::SimpleActiveCallback() ) ;
//ROS_INFO("+++++++++++++++++++++++++ @moveRobotCallback, a goal is sent +++++++++++++++++++++++++++++++++++++\n");
	//m_move_client.sendGoalAndWait(goal, ros::Duration(0, 0), ros::Duration(0, 0)) ;
	m_move_client.waitForResult();
}

void FrontierDetectorSMS::unreachablefrontierCallback(const geometry_msgs::PoseStamped::ConstPtr& msg )
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

//void FrontierDetectorSMS::saveGridmap( string filename, const nav_msgs::OccupancyGrid &mapData )
//{
//	ofstream ofs_map(filename) ;
//	int height = mapData.info.height ;
//	int width  = mapData.info.width ;
//	std::vector<signed char> Data=mapData.data;
//	ofs_map << height << " " << width << " ";
//	for(int ridx = 0; ridx < height; ridx++)
//	{
//		for(int cidx = 0; cidx < width; cidx++)
//		{
//			int value = static_cast<int>( Data[ridx * width + cidx] ) ;
//			ofs_map << value << " ";
//		}
//	}
//	ofs_map.close();
//}
//
//void FrontierDetectorSMS::saveFrontierCandidates( string filename, vector<FrontierPoint> voFrontierCandidates )
//{
//	ofstream ofs_fpts(filename) ;
//	for(size_t idx=0; idx < voFrontierCandidates.size(); idx++)
//	{
//		FrontierPoint oFP = voFrontierCandidates[idx];
//		cv::Point initposition = oFP.GetInitGridmapPosition() ;
//		cv::Point correctedposition = oFP.GetCorrectedGridmapPosition() ;
//		float fcmconf = oFP.GetCMConfidence() ;
//		float fgmconf = oFP.GetGMConfidence() ;
//		ofs_fpts << fcmconf << " " << fgmconf << " " << oFP.isConfidentFrontierPoint() << " " <<
//		initposition.x << " " << initposition.y << " " << correctedposition.x << " " << correctedposition.y << endl;
//	}
//	ofs_fpts.close();
//}

}

