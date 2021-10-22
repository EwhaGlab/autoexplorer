/*
 * frontier_point.hpp
 *
 *  Created on: Oct 17, 2021
 *      Author: hankm
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

//typedef struct AutoexplorerParameters
//{
//	std::string str_debugpath;
//	float ffrontier_cost_thr ;
//	int noccupancy_thr;
//	int nlethal_cost_thr;
//	int nGlobalMapWidth ;
//	int nGlobalMapHeight ;
//
//	int nWeakCompThreshold;
//	int nNumPyrDownSample ;
//}AP;

enum PointState{ NOT_TESTED = 0, GM_TESTED, CM_FILTERED, FULL_TESTED };

class FrontierPoint
{
public:
	FrontierPoint( cv::Point FrontierCandidate_dsn, int mapheight, int mapwidth,
			float fmaporigy, float fmaporigx, float fmapres, int num_pyrdown): //, float fgridmap_conf_thr, float fcostmap_conf_thr ):
	mf_gridmap_confidence(1.f), mf_costmap_confidence(1.f),
	//m_bcostmap_consent(true), m_bgridmap_consent(true),
	mn_initposition_dsgm( FrontierCandidate_dsn ), // dsgm refers to n down-sampled gridmap image
	mb_isfrontierpoint(true),
	m_ePointstate( PointState::NOT_TESTED ),
	//m_fgridmap_conf_thr(fgridmap_conf_thr), m_fcostmap_conf_thr(fcostmap_conf_thr),
	mn_height(mapheight), mn_width(mapwidth),
	mf_origx(fmaporigx), mf_origy(fmaporigy),
	mf_res(fmapres), m_nNumPyrDownSample(num_pyrdown)
	{
		mn_globalcentx = mn_width / 2 ;
		mn_globalcenty = mn_height/ 2 ;
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
	{}

	cv::Point2f gridmap2world( cv::Point img_pt_ds0 )
	{
		float fpx = static_cast<float>(img_pt_ds0.x) - mn_globalcentx;
		float fpy = static_cast<float>(img_pt_ds0.y) - mn_globalcenty;

	//	ROS_INFO("%f %f %f %d %d\n", fpx, fResolution, fXstart, m_nScale, m_nNumPyrDownSample );
	//	ROS_INFO("%f %f %f %d \n", fpy, fResolution, fYstart, m_nScale );
		float fgx = ( fpx * mf_res ); // + fXstart ) ;
		float fgy = ( fpy * mf_res ); // - fYstart ) ;
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


	void SetFrontierFlag()
	{
		if( mf_gridmap_confidence < mf_gridmap_conf_thr || mf_costmap_confidence < mf_costmap_conf_thr)
			mb_isfrontierpoint = false;
	};

	bool isConfidentFrontierPoint() const { return mb_isfrontierpoint; }

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
