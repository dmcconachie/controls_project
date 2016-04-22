/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0
*
*	This class represents an implementation of the DBSCAN clustering algorithm as explained at link below
*	https://en.wikipedia.org/wiki/DBSCAN
*	I believe my implementation is flawed, but I cannot find an exact issue. It seems additional steps had to be take to prevent duplication of points. This should not have been needed. 
*/
#pragma once
#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PtDBSCAN.h"
using namespace std;

/** Typedef to simplify readability of code.*/
typedef vector<PtDBSCAN> Cluster;
/** \brief This class represents an implementation of the DBSCAN clustering algorithm.*/
class DBSCAN
{
public:
	/** \brief Run an DBSCAN on a PtDBSCAN data and return a set of identified Clusters.*/
	vector<Cluster> scan(vector<PtDBSCAN> data, int dist, int minPts);
	/** \brief Expand the provided cluster to find not critical points.*/
	void expandCluster(PtDBSCAN& pt, vector<PtDBSCAN*>& neighborPts, Cluster& cluster, int dist, int minPts);
	/** \brief Find all the neighbor points of a PtDBSCAN*/
	vector<PtDBSCAN*> regionQuery(PtDBSCAN& pt, vector<PtDBSCAN>& data, int dist);
	/** \brief Find all the neighbor points of a pointer to a PtDBSCAN*/
	vector<PtDBSCAN*> regionQueryPointer(PtDBSCAN& pt, vector<PtDBSCAN*>& data, int dist);
private:
	/** \brief Get the euclidian distance between two PtDBCAN points*/
	double getDistance(PtDBSCAN p1, PtDBSCAN p2);
	/** \brief Remove and duplicate points from a Cluster*/
	void removeDuplicates(vector<PtDBSCAN*>& set1);



};

