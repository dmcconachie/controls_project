/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0
*
*	This a wrapper class that adds visited and inCluster fields to the cv::KeyPoint class. 
*	This class is heavily implemented in the cloth tracking code, but has caused numerous issue. 
*	If possible this class should be removed so all the code can function around the cv::Point and cv::KeyPoint classes.
*/
#pragma once
#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
/**
* \brief This a wrapper class that adds visited and inCluster fields to the cv::KeyPoint class. 
*/
class PtDBSCAN: public cv::KeyPoint{
public:
	/** \brief The default constructor*/
	PtDBSCAN();
	/** \brief Constructor: Generates PtDBSCAN at given x and y coordinates.*/
	PtDBSCAN(int x, int y);
	/** \brief Constructor: Generates PtDBSCAN from provided cv::KeyPoint*/
	PtDBSCAN(cv::KeyPoint k);
	/** \brief Constructor: Generates PtDBSCAN from provided cv::Point*/
	PtDBSCAN(cv::Point p);
	/** \brief draws the provided set of PtDBSCAN on the provided image given a color*/
	static void drawPointSet(cv::Mat& img, vector<PtDBSCAN> data, cv::Scalar color);
	/** \brief Converts a set of cv::KeyPoint's to a set of PtDBSCAN*/
	static vector<PtDBSCAN> keyPoint2PtDBSCAN(vector<cv::KeyPoint> data);


	/** \overload \brief inplicit copy overload. Converts a PtDBSCAN in a cv::Point */
	operator cv::Point() { return cv::Point(kPt.pt.x, kPt.pt.y); };
	/** \brief The data storage for the PtDBSCAN*/
	cv::KeyPoint kPt;
	/** \brief Visisted Flag for use in DBSCAN algorithm*/
	bool visited;
	/** \brief In Cluster Flag for use in DBSCAN algorithm*/
	bool inCluster;
};

