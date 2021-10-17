/*
 * frontier_detector.cpp
 *
 *  Created on: Sep 29, 2021
 *      Author: hankm
 */


#include "frontier_detector.hpp"


namespace frontier_detector
{

FrontierDetector::FrontierDetector():
m_worldFrameId("map"), m_baseFrameId("base_link"),
m_globalcostmap_rows(0), m_globalcostmap_cols(0), m_eRobotState(ROBOT_STATE::ROBOT_IS_NOT_MOVING),
m_move_client("move_base", true),
m_fRobotRadius(0.3), isdone(false), m_nroi_origx(0), m_nroi_origy(0), m_nrows(0), m_ncols(0)
{}

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

	CV_Assert( winsize % 2 > 0 ); // must be odd number

	int height = gridmap.info.height ;
	int width  = gridmap.info.width ;
	std::vector<signed char> Data=gridmap.data;

	int w = winsize ;
	int h = winsize ;

	int r = winsize - (winsize-1)/2 ;
	int c = winsize - (winsize-1)/2 ;
	int y = r - 1;
	int x = c - 1;
	int gy_ = frontierCandidate.y;
	int gx_ = frontierCandidate.x;
	int gx, gy;

	//vector<vector<int>> dirs = { {0, -1}, {-1, 0}, {0, 1}, {1, 0} } ;

	int size = w * h ;

	int i = 0;
	int curridx = x + y * width ;
	int cnt = 0;

	int idx = gx_ + x + ( gy_ + y ) * width ;

	int8_t fpt_hat_occupancy = Data[idx] ;

	//ROS_WARN(" width: %d \t w: %d \n", width, w);
	//ROS_INFO(" %d (%d,%d) (%d,%d)", idx, (gx+x), (gy+y), 0, 0 );

	if( fpt_hat_occupancy == 0 ) // is at the free region. thus, the closest unknown cell is the corrected fpt.
	{
		while ( cnt < size )
		{
			for( int j = (i%2)*2; j < (i%2)*2+2; j++ )
			{
				int dx = nccxidx[j];
				int dy = nccyidx[j];

				for( int k=0; k < i+1; k++ )
				{
					x = x + dx;
					y = y + dy;
					if( (0 <= x && x < w) && (0 <= y && y < h) )
					{
						gx = gx_ + x;
						gy = gy_ + y;
						idx = gx + gy * width ;
						int8_t out = Data[idx] ;
						//ROS_INFO(" %d (%d,%d) (%d,%d)", idx, (gx+x), (gy+y), dx, dy );
						if( out == -1 ) // fpt_hat is a free cell. Thus, this pt is the corrected fpt.
						{
							break;
						}
					}
				}
			}
			i++ ;
			cnt++ ;
		}
	}
	// if fpt_hat is already at the unknown region, we might need to correct the position of this point
	else if( fpt_hat_occupancy < 0 )
	{
		while ( cnt < size )
		{
			for( int j = (i%2)*2; j < (i%2)*2+2; j++ )
			{
				int dx = nccxidx[j];
				int dy = nccyidx[j];

				for( int k=0; k < i+1; k++ )
				{
					x = x + dx;
					y = y + dy;
					if( (0 <= x && x < w) && (0 <= y && y < h) )
					{
						gx = gx_ + x ;
						gy = gy_ + y ;
						idx = gx + gy * width ;
						int8_t out = Data[idx] ;
						//ROS_INFO(" %d (%d,%d) (%d,%d)", idx, (gx+x), (gy+y), dx, dy );

						// ------------ //
						// oooooooooooo //
						// oooo x ooooo //

						if( out == 0 ) // We found the nn (free) border line. go ahead check its 7 neighbors
						{
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
										break;
									}
								}
							}

//							int8_t lu = Data[ gx-1 + (gy-1)*width ];
//							int8_t cu = Data[ gx   + (gy-1)*width ];
//							int8_t ru = Data[ gx+1 + (gy-1)*width ];
//
//							int8_t lc = Data[ gx-1 + (gy  )*width ];
//							int8_t rc = Data[ gx+1 + (gy  )*width ];
//
//							int8_t ld = Data[ gx-1 + (gy+1)*width ];
//							int8_t cd = Data[ gx   + (gy+1)*width ];
//							int8_t rd = Data[ gx+1 + (gy+1)*width ];
						}
					}
				}
			}
			i++ ;
			cnt++ ;
		}
	}

	else
	{
		return false ;
	}

	correctedPoint.x = gx ;
	correctedPoint.y = gy ;

	return true ;
}



//void accessFrontierPoint( ){}

}

