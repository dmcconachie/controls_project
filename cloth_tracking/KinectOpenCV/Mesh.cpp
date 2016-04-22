/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0
*
*	This class represents a surface interpolation. The points are ordered from top left to bottom right row wise.
*/
#include "stdafx.h"
#include "Mesh.h"
using namespace std;
/**The default constructor*/
Mesh::Mesh()
{
	//Do Nothing Constructor. Calls to this should be limited
}
/** \overload Constructor: Builds rectangular mesh from retangluar bounds with the given number of segments on each side.
*	@param bounds The cv::Rect bounding box which is to be interpolated with mesh points. 
*	@param segments The number of segments that the resulting mesh rectangle should contain on each side.
*/
Mesh::Mesh(const cv::Rect bounds, int segments)
{
	makeInterpolatedMesh(bounds, segments);
}
/** \overloadConstructor: Builds mesh from vector of 2d points with the provided number of rows and cols. This is not interpolated.
*	@param vec The vector of 2d points that is to be converted into a Mesh Object. Vec should have length row*cols. 
*	@param rows The number of rows that will excist in the Mesh created from vec.
*	@param cols The number of cols that will excist in the Mesh created from vec.
*/
Mesh::Mesh(const vector<cv::Point2f>& vec, int rows, int cols)
{
	makeMeshFromVector(vec, cols, rows);
}
/** \overload Copy Contructor: Builds new mesh from provided mesh.
*	@param m The reference to the Mesh that will be coppied into this object.
*/
Mesh::Mesh(const Mesh& m)
{
	meshPoints = m.meshPoints.clone(); //Copy data Mat
	controlIndex = m.controlIndex; //Copy control index vector
}
/** Generates a vector of 2d points from the mesh. Vector has length rows * cols.
*	@return A vector of 2d points cv::Point2f. This vector will have length Mesh.rows * Mesh.cols. The vecotr is in Row Major Order
*/
vector<cv::Point2f> Mesh::getVectorFromMesh()
{
	vector<cv::Point2f> points;
	for (int r = 0; r < meshPoints.rows; r++)
	{
		for (int c = 0; c < meshPoints.cols; c++)
		{
			points.push_back(cv::Point2f(meshPoints.at<cv::Vec3f>(r, c)[0], meshPoints.at<cv::Vec3f>(r, c)[1]));
		}
	}
	return points;
}

/** Builds mesh from vector of 2d points with the provided number of rows and cols. This is not interpolated.
*	@param vec The vector of 2d points that is to be converted into a Mesh Object. Vec should have length row*cols.
*	@param rows The number of rows that will excist in the Mesh created from vec.
*	@param cols The number of cols that will excist in the Mesh created from vec.
*/
void Mesh::makeMeshFromVector(const vector<cv::Point2f>& vec, int cols, int rows)
{
	meshPoints = cv::Mat(rows, cols, CV_32FC3).clone();
	int count = 0;
	for (int r = 0; r < meshPoints.rows; r++)
	{
		for (int c = 0; c < meshPoints.cols; c++)
		{
			if (count < vec.size())
			{
				meshPoints.at<cv::Vec3f>(r, c)[0] = vec[count].x;
				meshPoints.at<cv::Vec3f>(r, c)[1] = vec[count].y;
				meshPoints.at<cv::Vec3f>(r, c)[2] = 0;
			}
			count++;
		}
	}
}
/** Builds rectangular mesh from retangluar bounds with the given number of segments on each side.
*	@param bounds The cv::Rect bounding box which is to be interpolated with mesh points.
*	@param segments The number of segments that the resulting mesh rectangle should contain on each side.
*/
void Mesh::makeInterpolatedMesh(const cv::Rect& bounds, int segments)
{
	meshPoints.create(segments + 1, segments + 1, CV_32FC3); //The matrix which stores all the points
	float x_inc = bounds.width / (float)segments;
	float y_inc = bounds.height / (float)segments;
	for (int r = 0; r < meshPoints.rows; r++)
	{
		for (int c = 0; c < meshPoints.cols; c++)
		{
			meshPoints.at<cv::Vec3f>(r, c)[0] = x_inc * c + bounds.x; //x
			meshPoints.at<cv::Vec3f>(r, c)[1] = y_inc * r + bounds.y;//y
			meshPoints.at<cv::Vec3f>(r, c)[2] = 0;                //z
		}
	}
}
/** Returns the number of Rows in the Mesh
*	@return The number of rows in the Mesh as an int.
*/
int Mesh::getRows()
{
	return meshPoints.rows;
}
/** Returns the number of Cols in the Mesh
*	@return The number of cols in the Mesh as an int.
*/
int Mesh::getCols()
{
	return meshPoints.cols;
}
/** Returns THe distance between two 2d points
*	@param p1 The first point.
*	@param p2 the second point.
*	@return THe distance between the two provided points as a float
*/
float Mesh::distanceToPoint(cv::Point p1, cv::Point p2)
{
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}
/** Finds the nearest Mesh index with 2d coordinates closet to the provided point.
*	@param p The point that is being compared to every point in the Mesh.
*	@param mesh The mesh object whose data points are being compared to point p.
*	@return cv::Vec2i A 2d vector representing the row and col index of the nearest point in mesh to point p. [0] = rows; [1] = cols
*/
cv::Vec2i Mesh::getNearestPointCoordinates(cv::Point p, cv::Mat& mesh)
{
	int min = 0;
	cv::Vec2i loc;
	for (int r = 0; r < mesh.rows; r++)
	{
		for (int c = 0; c < mesh.cols; c++)
		{
			float distance = distanceToPoint(cv::Point(mesh.at<cv::Vec3f>(r, c)[0], mesh.at<cv::Vec3f>(r, c)[1]), p);
			if (r == 0 && c == 0)
			{
				min = distance;
				loc[0] = r;
				loc[1] = c;
			}
			else if (distance < min)
			{
				min = distance;
				loc[0] = r;
				loc[1] = c;
			}
		}
	}
	return loc;
}
/** Draws the mesh onto the provided image Mat
*	@param img the cv::Mat that will have the mesh drawn onto it.
*/
void Mesh::drawMesh(cv::Mat& img)
{
	for (int r = 0; r < meshPoints.rows; r++)
	{
		for (int c = 0; c < meshPoints.cols; c++)
		{
			cv::circle(img, cv::Point(meshPoints.at<cv::Vec3f>(r, c)[0], meshPoints.at<cv::Vec3f>(r, c)[1]), 4, cv::Scalar(0, 255, 0));
			if (c < meshPoints.cols - 1)
			{
				cv::line(img, cv::Point(meshPoints.at<cv::Vec3f>(r, c)[0], meshPoints.at<cv::Vec3f>(r, c)[1]), cv::Point(meshPoints.at<cv::Vec3f>(r, c + 1)[0], meshPoints.at<cv::Vec3f>(r, c + 1)[1]), cv::Scalar(0, 255, 0));
			}
			if (r < meshPoints.rows - 1)
			{
				cv::line(img, cv::Point(meshPoints.at<cv::Vec3f>(r, c)[0], meshPoints.at<cv::Vec3f>(r, c)[1]), cv::Point(meshPoints.at<cv::Vec3f>(r + 1, c)[0], meshPoints.at<cv::Vec3f>(r + 1, c)[1]), cv::Scalar(0, 255, 0));
			}
		}
	}
}
/** Returns the 3d Point Vector at the provided Mesh coordinates.
*	@param r The row index of the desired point within the mesh.
*	@param c The col index of the desired point within the mesh.
*	@return cv::Vec3f a 3d point vector representing the 3d point stored in the mesh at the given index.
*/
cv::Vec3f& Mesh::at(int r, int c)
{
	return meshPoints.at<cv::Vec3f>(r, c);
}
/** Returns the rectangular bounds of a vector of 2d points.
*	@param data The vector of 2d points whose bounds are to be found.
*	@return cv::Rect A rectangle with bounds equal to the min and max bounds of the 2d points within the data vector. 
*/
cv::Rect Mesh::getVectorBounds(vector<cv::Point>& data)
{
	//Find MidPoint and Bounding Box
	int minX, minY, maxX, maxY;
	int sumX = 0, sumY = 0;
	for (int i = 0; i < data.size(); i++)
	{
		if (i == 0)
		{
			minX = data[i].x;
			minY = data[i].y;
			maxX = data[i].x;
			maxY = data[i].y;
		}
		sumX += data[i].x;
		sumY += data[i].y;
		if (data[i].x < minX)
		{
			minX = data[i].x;
		}
		else if (data[i].x > maxX)
		{
			maxX = data[i].x;
		}

		if (data[i].y < minY)
		{
			minY = data[i].y;
		}
		else if (data[i].y > maxY)
		{
			maxY = data[i].y;
		}
	}
	//cv::Point midPoint(sumX / data.size, sumY / data.size);
	cv::Rect bounds = cv::Rect(cv::Point(minX, minY), cv::Point(maxX, maxY));
	return bounds;
}