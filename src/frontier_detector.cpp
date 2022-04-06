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

#include "frontier_detector.hpp"

namespace autoexplorer
{

FrontierDetector::FrontierDetector():
m_worldFrameId("map"), m_baseFrameId("base_link"),
m_globalcostmap_rows(0), m_globalcostmap_cols(0), m_eRobotState(ROBOT_STATE::ROBOT_IS_NOT_MOVING),
m_move_client("move_base", true),
m_fRobotRadius(0.3), mb_explorationisdone(false), m_nroi_origx(0), m_nroi_origy(0), m_nrows(0), m_ncols(0)
{

}

FrontierDetector::~FrontierDetector(){}

cv::Point2f FrontierDetector::img2gridmap( cv::Point img_pt ){};
cv::Point FrontierDetector::gridmap2img( cv::Point2f grid_pt ){};

void FrontierDetector::mapdataCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_ERROR("this shouldn't be called \n");
}
vector<cv::Point> FrontierDetector::eliminateSupriousFrontiers( nav_msgs::OccupancyGrid &costmapData, vector<cv::Point> frontierCandidates, int winsize = 25)
{}


bool FrontierDetector::correctFrontierPosition( const nav_msgs::OccupancyGrid &gridmap, const cv::Point& frontierCandidate, const int& winsize, cv::Point& correctedPoint  )
{
// pts found in pyrdown sampled map might be slightly off owing to the coarse level analysis
// we need to correct this point. Bring this point to its nearby border line

	correctedPoint = frontierCandidate;

	CV_Assert( winsize % 2 > 0 ); // must be an odd number

	int height = gridmap.info.height ;
	int width  = gridmap.info.width ;
	std::vector<signed char> Data=gridmap.data;

	int w = winsize ;
	int h = winsize ;

	int yc = winsize - (winsize-1)/2 ;
	int xc = winsize - (winsize-1)/2 ;
	int y = yc - 1;
	int x = xc - 1;

	int gy_ = frontierCandidate.y;
	int gx_ = frontierCandidate.x;
	int gx = gx_;
	int gy = gy_;

	//vector<vector<int>> dirs = { {0, -1}, {-1, 0}, {0, 1}, {1, 0} } ;

	int numcells = w * h ;

	int i = 0;
	int curridx = x + y * width ;
	int cnt = 0;

	int idx = gx_ + (gy_ ) * width ;

	int8_t fpt_hat_occupancy = Data[idx] ;

	//ROS_INFO("orig idx: %d (%d,%d) (%d,%d)", idx, (gx_), (gy_), x, y );

	if( fpt_hat_occupancy == 0 ) // is at the free region. thus, the closest unknown cell is the corrected fpt.
	{
		//ROS_INFO("cent occupancy is 0\n");
		while ( cnt < numcells )
		{
			for( int j = (i%2)*2; j < (i%2)*2+2; j++ )
			{
				int dx = nccxidx[j];
				int dy = nccyidx[j];

				for( int k=0; k < i+1; k++ )
				{
					x = x + dx ;
					y = y + dy ;
					if( (0 <= x && x < w ) && (0 <= y && y < h ) )
					{
						gx = gx + dx;
						gy = gy + dy;
						idx = gx + gy * width ;
						int8_t out = Data[idx] ;
						//ROS_INFO(" %d (%d,%d) (%d,%d)", idx, gx, gy, dx, dy );
						if( out == -1 ) // fpt_hat is a free cell. Thus, this pt is the corrected fpt.
						{
							//ROS_INFO(" corrected pixel is %d %d \n", gx, gy );
							correctedPoint.x = gx ;
							correctedPoint.y = gy ;
							return true;
						}
					}
					cnt++ ;
				}
			}
			i++ ;
		}
	}
	// if fpt_hat is already at the unknown region, we might need to shift this position to a boundary cell position
	else if( fpt_hat_occupancy < 0 )
	{
		//ROS_INFO("cent occupancy is -1\n");

		// see if there is a neighboring free cell
		for(int ii=-1; ii <2; ii++ )
		{
			for(int jj=-1; jj <2; jj++ )
			{
				if(ii == 0 && jj == 0)
					continue;

				int8_t nn = Data[ gx + jj + (gy + ii)*width];
				if(nn == 0)
				{
					//ROS_INFO("nn pix %d %d is free, thus no need to do any correction \n", gx+jj, gy+ii);
					return true;
				}
			}
		}

		while ( cnt < numcells )
		{
			for( int j = (i%2)*2; j < (i%2)*2+2; j++ )
			{
				int dx = nccxidx[j];
				int dy = nccyidx[j];

				for( int k=0; k < i+1; k++ )
				{
					x = x + dx ;
					y = y + dy ;
					//ROS_INFO("x y h w i cnt (%d %d) (%d %d) %d %d %d | ", x, y, h, w, j, i, cnt);
					if( (0 <= x && x < w ) && (0 <= y && y < h ) )
					{
						gx = gx + dx;
						gy = gy + dy;
						idx = gx + gy * width ;
						int8_t out = Data[idx] ;
					//	ROS_INFO(" %d (%d,%d) (%d,%d)", idx, gx, gy, dx, dy );

						// ------------ //
						// oooooooooooo //
						// oooo x ooooo //

						if( out == 0 ) // We found the nn (free) border pixel. go ahead check its 7 neighbors
						{
							//ROS_INFO(" found a free pixel at %d %d \n", gx, gy );
							for(int ii=-1; ii <2; ii++ )
							{
								for(int jj=-1; jj <2; jj++ )
								{
									if(ii == 0 && jj == 0)
										continue;

									int8_t nn = Data[ gx + jj + (gy + ii)*width];
									if(nn < 0)
									{
										gx = gx + jj ;
										gy = gy + ii ;
										//ROS_INFO(" corrected pixel is %d %d \n", gx, gy );
										correctedPoint.x = gx ;
										correctedPoint.y = gy ;
										return true;
									}
								}
							}
						}
					}
					cnt++ ;
				}
			}
			i++ ;
		}
	}

	else
	{
		return false ;
	}

	return true ;
}


void FrontierDetector::SetVizMarkers( const string& frame_id,
					const float& fR, const float& fG, const float& fB, const float& fscale, visualization_msgs::Marker&  viz_marker)
{
	viz_marker.header.frame_id= frame_id;
	viz_marker.header.stamp=ros::Time(0);
	viz_marker.ns= "markers";
	viz_marker.id = 0;
	viz_marker.type = viz_marker.POINTS;

	viz_marker.action = viz_marker.ADD;
	viz_marker.pose.orientation.w =1.0;
	viz_marker.scale.x= fscale;
	viz_marker.scale.y= fscale;

	viz_marker.color.r = fR;
	viz_marker.color.g = fG;
	viz_marker.color.b = fB;
	viz_marker.color.a=1.0;
	viz_marker.lifetime = ros::Duration();
}

//void accessFrontierPoint( ){}

void FrontierDetector::saveGridmap( string filename, const nav_msgs::OccupancyGrid &mapData )
{
	ofstream ofs_map(filename) ;
	int height = mapData.info.height ;
	int width  = mapData.info.width ;
	float origx = mapData.info.origin.position.x ;
	float origy = mapData.info.origin.position.y ;
	float resolution = mapData.info.resolution ;

	std::vector<signed char> Data=mapData.data;
	ofs_map << width << " " << height << " " << origx << " " << origy << " " << resolution;
	for(int ridx = 0; ridx < height; ridx++)
	{
		for(int cidx = 0; cidx < width; cidx++)
		{
			int value = static_cast<int>( Data[ridx * width + cidx] ) ;
			ofs_map << value << " ";
		}
	}
	ofs_map.close();
}

void FrontierDetector::saveFrontierCandidates( string filename, vector<FrontierPoint> voFrontierCandidates )
{
	ofstream ofs_fpts(filename) ;
	for(size_t idx=0; idx < voFrontierCandidates.size(); idx++)
	{
		FrontierPoint oFP = voFrontierCandidates[idx];
		cv::Point initposition = oFP.GetInitGridmapPosition() ;
		cv::Point correctedposition = oFP.GetCorrectedGridmapPosition() ;
		float fcmconf = oFP.GetCMConfidence() ;
		float fgmconf = oFP.GetGMConfidence() ;
		ofs_fpts << fcmconf << " " << fgmconf << " " << oFP.isConfidentFrontierPoint() << " " <<
		initposition.x << " " << initposition.y << " " << correctedposition.x << " " << correctedposition.y << endl;
	}
	ofs_fpts.close();
}


}

