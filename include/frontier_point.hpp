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


#ifndef INCLUDE_FRONTIER_POINT_HPP_
#define INCLUDE_FRONTIER_POINT_HPP_

#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "octomap_server/mapframedata.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include "nav_msgs/OccupancyGrid.h"

namespace autoexplorer
{

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


enum PointState{ NOT_TESTED = 0, GM_TESTED, CM_FILTERED, FULL_TESTED };

class FrontierPoint
{
public:
	FrontierPoint( cv::Point FrontierCandidate_dsn, int mapheight, int mapwidth,
			float fmaporigy, float fmaporigx, float fmapres, int num_pyrdown): //, float fgridmap_conf_thr, float fcostmap_conf_thr ):
	mf_gridmap_confidence(1.f), mf_costmap_confidence(1.f),
	//m_bcostmap_consent(true), m_bgridmap_consent(true),
	mn_initposition_dsgm( FrontierCandidate_dsn ), // dsgm refers to n down-sampled gridmap image
	mb_isfrontierpoint(true), mb_isreachable(true),
	m_ePointstate( PointState::NOT_TESTED ),
	//m_fgridmap_conf_thr(fgridmap_conf_thr), m_fcostmap_conf_thr(fcostmap_conf_thr),
	mn_height(mapheight), mn_width(mapwidth),
	mf_origx(fmaporigx), mf_origy(fmaporigy),
	mf_res(fmapres), m_nNumPyrDownSample(num_pyrdown)
	{
		//mn_globalcentx = mn_width / 2 ;
		//mn_globalcenty = mn_height/ 2 ;
		//SetWorldCoordiate();
		m_nScale = pow(2, m_nNumPyrDownSample) ;

		mn_initposition_gm 	   =	mn_initposition_dsgm * m_nScale ;
		mn_correctedposition_gm = 	mn_initposition_gm ;

		SetInitWorldCoordiate( );
		SetCorrectedCoordinate( mn_initposition_gm );
	}

	~FrontierPoint(){} ;

	void SetCostmapConfidence( float fconfidence){ mf_costmap_confidence = fconfidence ; }
	void SetGridmapConfidence( float fconfidence){ mf_gridmap_confidence = fconfidence ; }
	float GetCMConfidence() const { return mf_costmap_confidence; }
	float GetGMConfidence() const { return mf_gridmap_confidence; }

	cv::Point world2gridmap( cv::Point2f point_w)
	{
		int ngmx = static_cast<int>( (point_w.x - mf_origx) / mf_res ) ;
		int ngmy = static_cast<int>( (point_w.y - mf_origy) / mf_res ) ;
		return cv::Point( ngmx, ngmy );
	}

	cv::Point2f gridmap2world( cv::Point img_pt_ds0 )
	{
		float fgmx = static_cast<float>(img_pt_ds0.x) ;
		float fgmy = static_cast<float>(img_pt_ds0.y) ;

		float fgx = ( fgmx * mf_res ) + mf_origx ; // + fXstart ) ;
		float fgy = ( fgmy * mf_res ) + mf_origy ; // - fYstart ) ;
		return cv::Point2f( fgx, fgy );
	}

	// given frontier input convert it to world coord
	void SetInitWorldCoordiate( )
	{
		mf_initposition_w = gridmap2world( mn_initposition_gm  );
	}

	void SetCorrectedCoordinate( const cv::Point& pt )
	{
		mn_correctedposition_gm = pt;
		mf_correctedposition_w = gridmap2world( mn_correctedposition_gm  );
	}


	void SetFrontierFlag( const float& fcm_conf, const float& fgm_conf)
	{
		if( mf_gridmap_confidence < fgm_conf || mf_costmap_confidence < fcm_conf)
			mb_isfrontierpoint = false;
	};

	void SetReachability( const bool& bisreachable )
	{
		mb_isreachable = bisreachable;
	}

	bool isConfidentFrontierPoint() const { return mb_isfrontierpoint; }
	bool isReachable() const { return mb_isreachable; }

	cv::Point GetInitGridmapPosition()		const {return mn_initposition_gm; };
	cv::Point GetCorrectedGridmapPosition() const {return mn_correctedposition_gm; };
	cv::Point2f GetCorrectedWorldPosition() 	const {return mf_correctedposition_w; };

private:

	//bool m_bcostmap_consent, m_bgridmap_consent;
	float mf_gridmap_confidence ;
	float mf_costmap_confidence ;
	float mf_gridmap_conf_thr ;
	float mf_costmap_conf_thr ;

	cv::Point2f mf_initposition_w ;			// on world (meter) coord
	cv::Point2f mf_correctedposition_w ;		// on world (meter) coord

	cv::Point mn_initposition_dsgm ;			// on downsampled image (gm/cm) coord. --i.e) smaller than the orig size
	cv::Point mn_initposition_gm ; 			// on image (gm/cm) coord ( ex. 4K x 4K original size )
	cv::Point mn_correctedposition_gm ;		// on image (gm/cm) coord
	int8_t m_ePointstate;

	bool mb_isfrontierpoint ;
	bool mb_isreachable ;
	int mn_height;
	int mn_width;
	int mn_globalcentx ;
	int mn_globalcenty ;
	float mf_origx ;
	float mf_origy ;
	float mf_res ;
	int m_nNumPyrDownSample ;
	int m_nScale ;
};

}

#endif /* INCLUDE_FRONTIER_POINT_HPP_ */
