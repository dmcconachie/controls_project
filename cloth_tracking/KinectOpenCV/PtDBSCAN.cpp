/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0
*
*	This a wrapper class that adds visited and inCluster fields to the cv::KeyPoint class.
*	This class is heavily implemented in the cloth tracking code, but has caused numerous issue.
*	If possible this class should be removed so all the code can function around the cv::Point and cv::KeyPoint classes.
*/
#include "stdafx.h"
#include "PtDBSCAN.h"

/** The default constructor*/
PtDBSCAN::PtDBSCAN()
{
	visited = false;
	inCluster = false;
	kPt.pt.x = 0;
	kPt.pt.y = 0;
}
/** Constructor: Generates PtDBSCAN at given x and y coordinates.
*	@param x The x coordinate of the point to be created.
*	@param y The y coordinate of the point to be created.
*/
PtDBSCAN::PtDBSCAN(int x, int y)
{
	visited = false;
	inCluster = false;
	kPt.pt.x = x;
	kPt.pt.y = y;
}
/** Constructor: Generates PtDBSCAN from provided cv::KeyPoint
*	@param k The cv::KeyPoint that is to be converted into a PtDBSCAN
*/
PtDBSCAN::PtDBSCAN(cv::KeyPoint k)
{
	visited = false;
	inCluster = false;
	kPt = k;
}
/** Constructor: Generates PtDBSCAN from provided cv::Point
*	@param p The cv::Point that is to be converted into a PtDBSCAN
*/
PtDBSCAN::PtDBSCAN(cv::Point p)
{
	visited = false;
	inCluster = false;
	kPt.pt.x = p.x;
	kPt.pt.y = p.y;
}
/** Converts a set of cv::KeyPoint's to a set of PtDBSCAN
*	@param data The set of cv::KeyPoint's to be converted into a set of PtDBSCAN. Represented as a vector of cv::KeyPoint.
*	@return vector<PtDBSCAN> The set of PtDBSCAN that is equivalent to the provided set of cv::KeyPoint's
*/
vector<PtDBSCAN> PtDBSCAN::keyPoint2PtDBSCAN(vector<cv::KeyPoint> data)
{
	vector<PtDBSCAN> points;
	for (int i = 0; i < data.size(); i++)
	{
		points.push_back(PtDBSCAN(data[i]));
	}
	return points;
}
/** Draws the provided set of PtDBSCAN on the provided image given a color
*	@param img A reference to the cv::Mat image that the point set will be drawn onto.
*	@param data The set of PtDBSCAN do be drawn on the image. Represented as a vector of PtDBSCAN
*	@param color The BGR color to draw the points as. Represented as a cv::Scalar(B, G, R)
*/
void PtDBSCAN::drawPointSet(cv::Mat& img, vector<PtDBSCAN> data, cv::Scalar color)
{
	for (int i = 0; i < data.size(); i++)
	{
		cv::circle(img, cv::Point(data[i].kPt.pt.x, data[i].kPt.pt.y), 4, color);
	}
}
