/*********************************************************************
Copyright 2022 The Ewha Womans University.
All Rights Reserved.

Permission to use, copy, modify OR distribute this software and its
documentation for educational, research and non-profit purposes, without
fee, and without a written agreement is hereby granted, provided that the
above copyright notice and the following three paragraphs appear in all
copies.

IN NO EVENT SHALL THE EWHA WOMANS UNIVERSITY BE
LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE EWHA WOMANS UNIVERSITY
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

THE EWHA WOMANS UNIVERSITY SPECIFICALLY DISCLAIM ANY
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE EWHA WOMANS UNIVERSITY
HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
UPDATES, ENHANCEMENTS, OR MODIFICATIONS.


The authors may be contacted via:


Mail:        Y. J. Kim, Kyung Min Han
             Computer Graphics Lab
             Department of Computer Science and Engineering
             Ewha Womans University
             11-1 Daehyun-Dong Seodaemun-gu, Seoul, Korea 120-750


Phone:       +82-2-3277-6798


EMail:       kimy@ewha.ac.kr
             hankm@ewha.ac.kr
*/

#ifndef INCLUDE_DFFP_HPP_
#define INCLUDE_DFFP_HPP_


#include<cmath>
#include<cstring>
#include<algorithm>
#include<vector>
#include<queue>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

//#define DRAW_CONTOUR

namespace dffp
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
		//m_lattice = std::vector<int>(m_len, 2); //resize( m_len ) ;

		initLattice();

		m_image = image.clone();

		m_frontierContour = std::vector<cv::Point>();
	};
	~FrontPropagation(  ){} ;

	int nncolidx[6] = {-1,1,0,0,-1,1} ;
	int nnrowidx[6] = {0,0,-1,1,-1,1} ;

	void MarchFrontFromInside( const cv::Mat& uImage, cv::Point seeds )
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

//printf(": %d %d %d  /(%d %d) \n", pid, rowidx, colidx, m_rows, m_cols);

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
					if( uImage.data[qid] == MapStatus::FREE  ) // if we can physically march front
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


	void InnerContourCorrection( const cv::Mat& mapimage )
	{
		for( int idx = 0; idx < m_inner_contour.size(); idx++)
		{
			cv::Point pt = ind2sub( m_inner_contour[idx] ) ;

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

			int i0 = 	tv * m_cols + lu ;
			int i1 =  	tv * m_cols + u ;
			int i2 = 	tv * m_cols + ru ;

			int i3 =	v * m_cols + lu ;
			//int i4 =	v * m_cols + u ;
			int i5 = 	v * m_cols + ru ;

			int i6 =	bv * m_cols + lu ;
			int i7 = 	bv * m_cols + u ;
			int i8 =	bv * m_cols + ru ;

			uint8_t x0  = mapimage.data[ i0 ] ;
			uint8_t x1 	= mapimage.data[ i1 ];
			uint8_t x2	= mapimage.data[ i2 ] ;

			uint8_t x3	= mapimage.data[ i3 ] ;
			//uint8_t x4	= mapimage.data[  ] ;
			uint8_t x5	= mapimage.data[ i5 ] ;

			uint8_t x6	= mapimage.data[ i6 ] ;
			uint8_t x7	= mapimage.data[ i7 ] ;
			uint8_t x8	= mapimage.data[ i8 ] ;

			if( x0 == MapStatus::UNKNOWN )
				m_contour.push_back(i0);

			if( x1 == MapStatus::UNKNOWN )
				m_contour.push_back(i1);

			if( x2 == MapStatus::UNKNOWN )
				m_contour.push_back(i2);

			if( x3 == MapStatus::UNKNOWN )
				m_contour.push_back(i3);

			if( x5 == MapStatus::UNKNOWN )
				m_contour.push_back(i5);

			if( x6 == MapStatus::UNKNOWN )
				m_contour.push_back(i6);

			if( x7 == MapStatus::UNKNOWN )
				m_contour.push_back(i7);

			if( x8 == MapStatus::UNKNOWN )
				m_contour.push_back(i8);
		}
	}

	void MarchFrontFromOutside( const cv::Mat& uImage, cv::Point seeds )
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


	void update(const cv::Mat& gray_float, cv::Point seed_inner, cv::Point seed_outer )
	{
	    MarchFrontFromInside(gray_float, seed_inner); 	// required for DFFP
	    InnerContourCorrection( gray_float );			// required for DFFP

	    initLattice();
		MarchFrontFromOutside(gray_float, seed_outer);

//	    cv::namedWindow("after the fast marching",1);
//	    cv::imshow("after the fast marching", m_cvStatus);
//	    cv::waitKey(10);
	}

	inline void initLattice(){ m_lattice = std::vector<int>(m_len, 2);  }

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
	std::vector<int> m_inner_contour, m_contour ;
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






#endif /* INCLUDE_DFFP_HPP_ */
