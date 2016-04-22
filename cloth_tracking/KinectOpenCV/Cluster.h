#pragma once
#include <opencv2/core/core.hpp>
using namespace std;
class Cluster
{
public:
	Cluster();
	Cluster(vector<cv::KeyPoint> data);
	//vector<cv::Pt> cluster;
	cv::Point2f midPoint;
	cv::Rect2f bounds;
private:
};
