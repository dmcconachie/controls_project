#include "Cluster.h"
#include "PtDBSCAN.h"
Cluster::Cluster()
{
	
}
Cluster::Cluster(vector<cv::KeyPoint> data)
{
	cluster = data;
	int minX, minY, maxX, maxY;
	int sumX = 0, sumY = 0;
	for (int i = 0; i < cluster.size; i++)
	{
		if (i == 0)
		{
			minX = data[i].pt.x;
			minY = data[i].pt.y;
			maxX = data[i].pt.x;
			maxY = data[i].pt.y;
		}
		sumX += data[i].pt.x;
		sumY += data[i].pt.y;
		if (data[i].pt.x < minX)
		{
			minX = data[i].pt.x;
		}
		else if (data[i].pt.x > maxX)
		{
			maxX = data[i].pt.x;
		}

		if (data[i].pt.y < minY)
		{
			minY = data[i].pt.y;
		}
		else if (data[i].pt.y > maxY)
		{
			maxY = data[i].pt.y;
		}
	}
	midPoint = cv::Point2f(sumX / data.size, sumY / data.size);
	bounds = cv::Rect(cv::Point2f(minX, minY), cv::Point2f(maxX, maxY));
	PtDBSCAN p;
}