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

#ifndef INCLUDE_FFP_HPP_
#define INCLUDE_FFP_HPP_

#include<cmath>
#include<cstring>
#include<algorithm>
#include<vector>
#include<queue>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

//#define DRAW_CONTOUR

namespace ffp
{

constexpr char WIND_NAME[] = "Image";

enum MapStatus   {UNKNOWN=127, FREE=0, OCCUPIED=255};
enum MarchStatus {KNOWN=0, TRIAL, FAR};
enum LevelSetStatus {INSIDE=0, OUTSIDE=1};

class FrontPropagation
{
public:
	FrontPropagation( const cv::Mat& image  ):
	m_rows(image.rows), m_cols(image.cols),
	m_neighbor(6)
	{
		m_cvFrontierContour = cv::Mat::zeros(m_rows, m_cols, CV_8U);
		m_len = m_rows * m_cols ;
		m_lattice = std::vector<int>(m_len, 2); //resize( m_len ) ;
		m_image = image.clone();

		m_frontierContour = std::vector<cv::Point>();
	};
	~FrontPropagation(  ){} ;

	int nncolidx[6] = {-1,1,0,0,-1,1} ;
	int nnrowidx[6] = {0,0,-1,1,-1,1} ;

	void MarchFront( const cv::Mat& uImage, cv::Point seeds )
	{

		int nid = seeds.x + seeds.y * m_cols ;
		m_que.push_back(nid);
		while( !m_que.empty() )
		{
//printQ();
			int pid = m_que.front() ;
			int colidx = pid % m_cols ;
			int rowidx = static_cast<int>( std::floor( (float)pid / (float)m_cols) ) ;
			m_que.pop_front() ;
			// update march status
			m_lattice[pid] = MarchStatus::KNOWN ;

//printf(": %d %d %d \n", pid, rowidx, colidx);

			// each neighboring pixels Q of P
			// l,r,t,b,tl,br

			// now check if we can march
			for(size_t i=0; i < 6; i++)
			{
				int nrowidx = rowidx + nnrowidx[i];
				int ncolidx = colidx + nncolidx[i];
				int qid = nrowidx * m_cols + ncolidx ;

//printf("pid %d qid: %d \n",pid, qid);

				if( nrowidx < 0 || ncolidx < 0 || nrowidx >= m_rows || ncolidx >= m_cols || qid == pid)
				{
					continue;
				}

				if( m_lattice[ qid ] != MarchStatus::KNOWN ) // meaning that we need to check for update
				{
					if( uImage.data[qid] == MapStatus::UNKNOWN  ) // if we can physically march front
					{
						if(m_lattice[qid] == MarchStatus::FAR)
						{
							m_que.push_back( qid );
							m_lattice[qid] = MarchStatus::TRIAL ;
						}
					}
					else // this case mean that we are facing a boundary
					{
						m_contour.push_back(pid);

						int colidx = pid % m_cols ;
						int rowidx = static_cast<int>( std::floor( (float)pid / (float)m_cols) ) ;
					}
				}
				// if we cannot march,
			}
//
//cv::namedWindow("tmp");
//cv::imshow("tmp", m_cvStatus);
//cv::waitKey(1);
		}

	}

	void printQ()
	{
		printf("Q: ");
		for( size_t i=0; i < m_que.size(); i++ )
			printf(" %d ", m_que[i]);
		printf("\n");
	}


	void update(const cv::Mat& gray_float, cv::Point seeds)
	{
	    MarchFront(gray_float, seeds);

//	    cv::namedWindow("after the fast marching",1);
//	    cv::imshow("after the fast marching", m_cvStatus);
//	    cv::waitKey(10);
	}
	std::vector<int> getlattice()
	{
		return m_lattice;
	}

	void pop_front( std::vector<int>& vec)
	{
		assert(!vec.empty());
		vec.front() = std::move(vec.back());
		vec.pop_back();
	}

	std::vector<int> GetContour()
	{
		return m_contour ;
	}

	cv::Point ind2sub( int index )
	{
		int colidx = index % m_cols ;
		int rowidx = static_cast<int>( std::floor( (float)index / (float)m_cols) ) ;
		return cv::Point( colidx, rowidx );
	}

	void extractFrontierRegion( const cv::Mat& mapimage  ) // unsigned char
	{
		#ifdef DRAW_CONTOUR
			cv::Mat cvFrontier = mapimage.clone() ;
			cv::cvtColor(cvFrontier, cvFrontier, cv::COLOR_GRAY2RGB);
		#endif

		for( size_t idx=0; idx < m_contour.size() ; idx++ )
		{
			cv::Point pt = ind2sub( m_contour[idx] ) ;

		// check for 8 neighboring pts

			int u = pt.x ;
			int v = pt.y ;

		// left
			int lu = MAX(u - 1, 0);
		// right
			int ru = MIN(u + 1, m_cols);
		// top
			int tv = MAX(v - 1, 0);
		// bottom
			int bv = MIN(v + 1, m_rows);

			uint8_t x0  = mapimage.data[ tv * m_cols + lu ] ;
			uint8_t x1 	= mapimage.data[ tv * m_cols + u ] ;
			uint8_t x2	= mapimage.data[ tv * m_cols + ru ] ;

			uint8_t x3	= mapimage.data[ v * m_cols + lu] ;
			uint8_t x4	= mapimage.data[ v * m_cols + u] ;
			uint8_t x5	= mapimage.data[ v * m_cols + ru] ;

			uint8_t x6	= mapimage.data[ bv * m_cols + lu] ;
			uint8_t x7	= mapimage.data[ bv * m_cols + u ] ;
			uint8_t x8	= mapimage.data[ bv * m_cols + ru ] ;

			if( x0 == MapStatus::OCCUPIED || x1 == MapStatus::OCCUPIED || x2 == MapStatus::OCCUPIED  ||
				x3 == MapStatus::OCCUPIED || x4 == MapStatus::OCCUPIED || x5 == MapStatus::OCCUPIED  ||
				x6 == MapStatus::OCCUPIED || x7 == MapStatus::OCCUPIED || x8 == MapStatus::OCCUPIED  )
			{
				continue;
			}
			else
			{
				m_cvFrontierContour.data[ v * m_cols + u ] = 255; //MapStatus::OCCUPIED ;
				m_frontierContour.push_back(cv::Point(u, v) ) ;
				#ifdef DRAW_CONTOUR
					cvFrontier.at<cv::Vec3b>( v, u )[0] = 0 ;
					cvFrontier.at<cv::Vec3b>( v, u )[1] = 255 ;
					cvFrontier.at<cv::Vec3b>( v, u )[2] = 0 ;
				#endif
			}
		}

		//printf("tot frontiers: %u \n", m_frontierContour.size());
		#ifdef DRAW_CONTOUR
//			cv::namedWindow("frontier contour", 1);
//			cv::imshow("frontier contour", cvFrontier);
//			cv::waitKey(10);

			cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/after_fm.png", m_cvFrontierContour);
			cv::imwrite("/home/hankm/catkin_ws/src/frontier_detector/images/frontier_contour.png", cvFrontier);
		#endif
	}

	cv::Mat GetFrontierContour( )
	{
		return m_cvFrontierContour ;
	}

	std::vector<cv::Point> GetFrontiers() const { return m_frontierContour; }


private:

	std::vector<cv::Point> m_seeds ;
	std::vector<int> m_lattice ;
	std::deque<int> m_que ;
	std::vector<int> m_contour ;
	std::vector<cv::Point> m_frontierContour ;
	std::vector<int> m_neighbor ;

	//cv::Mat m_cvStatus ;
	cv::Mat m_cvLevelSet ;
	cv::Mat m_cvFrontierContour ;

	int m_rows, m_cols, m_len ;
	cv::Mat m_image;

//	uchar* pMarchStatus ;

};

}



#endif /* INCLUDE_FFP_HPP_ */
