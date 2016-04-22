/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0
*
*	This class represents an implementation of the DBSCAN clustering algorithm as explained at link below
*	https://en.wikipedia.org/wiki/DBSCAN
*	I believe my implementation is flawed, but I cannot find an exact issue. It seems additional steps had to be take to prevent duplication of points. This should not have been needed.
*/
#include "stdafx.h"
#include "DBSCAN.h"
#include <iostream>
/** Run an DBSCAN on a PtDBSCAN data and return a set of identified Clusters.
*	@param data The set of PtDBSCAN to be clustered
*	@param dist The minimum distance between data points to consider them in a cluster
*	@param minPts the minimum number of points needed to form a Cluster
*	@return A set of all detected Cluster's
*/
vector<Cluster> DBSCAN::scan(vector<PtDBSCAN> data, int dist, int minPts)
{
	vector<Cluster> clusters;
	for (int i = 0; i < data.size(); i++)
	{
		if (data[i].visited){/*do Nothing*/}
		else
		{
			data[i].visited = true;
			//data[i].inCluster = true;
			vector<PtDBSCAN*> neighborPts = regionQuery(data[i], data, dist); //change to references
			if (neighborPts.size() < minPts){/*Pt is Noise*/}
			else
			{
				Cluster cluster;
				expandCluster(data[i], neighborPts, cluster, dist, minPts);//add references
				if (cluster.size() >= minPts) // remove to allow single points to become clusters when close to other clusters.
				{
					clusters.push_back(cluster);
				}
			}
		}
	}
	return clusters;
}
/** Expand the provided cluster to find not critical points.
*	@param pt The point that the Cluster will be expanded from
*	@param neighborPts The set of PtDBSCAN that are within reach of the Point pt.
*	@param dist The minimum distance between data points to consider them in a cluster
*	@param minPts the minimum number of points needed to form a Cluster
*/
void DBSCAN::expandCluster(PtDBSCAN& pt, vector<PtDBSCAN*>& neighborPts, Cluster& cluster, int dist, int minPts)
{
	for (int i = 0; i < neighborPts.size(); i++)
	{
		if (!(*neighborPts[i]).visited) //Not visited
		{
			(*neighborPts[i]).visited = true;
			vector<PtDBSCAN*> nPts = regionQueryPointer((*neighborPts[i]), neighborPts, dist);// object sent into here is not in a cluster 
			removeDuplicates(nPts);
			if (nPts.size() >= minPts)
			{
				neighborPts.insert(neighborPts.end(), nPts.begin(), nPts.end());
			}
		}
		if (!neighborPts[i]->inCluster)
		{
			neighborPts[i]->inCluster = true;
			cluster.push_back(*neighborPts[i]);
		}

	}
}
/** Remove and duplicate points from a Cluster
*	@param The Cluster that will have duplicate points removed from it.
*/
void DBSCAN::removeDuplicates(vector<PtDBSCAN*>& set1)
{
	for (int i = 0; i < set1.size(); i++)
	{
		for (int j = i+1; j < set1.size(); j++)
		{
			if (set1[i]->kPt.pt.x == set1[j]->kPt.pt.x && set1[j]->kPt.pt.y == set1[j]->kPt.pt.y) //Check for duplicate elements
			{
				set1.erase(set1.begin() + j); // erase the duplicate
			}
		}
	}
}
/** Find all the neighbor points of a PtDBSCAN.
*	@param pt The point that neighbors will be compared to.
*	@param ata the set of all points.
*	@param dist The minimum distance between data points to be condidered neighbors
*/
vector<PtDBSCAN*> DBSCAN::regionQuery(PtDBSCAN& pt, vector<PtDBSCAN>& data, int dist)
{
	vector<PtDBSCAN*> regionPts;
	for (int i = 0; i < data.size(); i++)
	{
		if (getDistance(pt, data[i]) <= dist)
		{
			regionPts.push_back(&data[i]);
		}
	}
	return regionPts;
}
/** Find all the neighbor points of a pointer to a PtDBSCAN
*	@param pt The point that neighbors will be compared to.
*	@param ata the set of all points.
*	@param dist The minimum distance between data points to be condidered neighbors
*/
vector<PtDBSCAN*> DBSCAN::regionQueryPointer(PtDBSCAN& pt, vector<PtDBSCAN*>& data, int dist)
{
	vector<PtDBSCAN*> regionPts;
	for (int i = 0; i < data.size(); i++)
	{
		if (getDistance(pt, *data[i]) <= dist)
		{
			regionPts.push_back(data[i]);
		}
	}
	return regionPts;
}
/** Get the euclidian distance between two PtDBCAN points
*	@param p1 The first point
*	@param p2 The second point
*	@return The distance between the two points as a double
*/
double DBSCAN::getDistance(PtDBSCAN p1, PtDBSCAN p2)
{
	return sqrt(pow(p2.pt.x - p1.pt.x, 2) + pow(p2.pt.y - p1.pt.y, 2));
}

/* The Base Algorithm used in this class 
DBSCAN(D, eps, MinPts) {
	C = 0
		for each point P in dataset D{
			if P is visited
			continue next point
				mark P as visited
				NeighborPts = regionQuery(P, eps)
				if sizeof(NeighborPts) < MinPts
					mark P as NOISE
				else {
					C = next cluster
						expandCluster(P, NeighborPts, C, eps, MinPts)
				}
		}
}

expandCluster(P, NeighborPts, C, eps, MinPts) {
	add P to cluster C
		for each point P' in NeighborPts {
			if P' is not visited {
				mark P' as visited
				NeighborPts' = regionQuery(P', eps)
				if sizeof(NeighborPts') >= MinPts
					NeighborPts = NeighborPts joined with NeighborPts'
}
if P' is not yet member of any cluster
add P' to cluster C
   }
}

regionQuery(P, eps)
return all points within P's eps-neighborhood (including P)
*/
