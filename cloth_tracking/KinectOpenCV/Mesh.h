/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0
*
*	This class represents a surface interpolation. The points are ordered from top left to bottom right row wise. 
*/
#pragma once
#ifndef MESH_H
#define MESH_H
#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2\shape\shape_transformer.hpp"
#include <iostream>
using namespace std;
/**
* \brief The Mesh class reprents a multi point interpolation. The points are ordered from top left to bottom right row wise. 
*/
class Mesh
{
public:
	/** \brief The default constructor*/
	Mesh();
	/** \overload \brief Constructor: Builds rectangular mesh from retangluar bounds with the given number of segments on each side.*/
	Mesh(cv::Rect bounds, int segments);
	/** \overload \brief Constructor: Builds mesh from vector of 2d points with the provided number of rows and cols. This is not interpolated.*/
	Mesh(const vector<cv::Point2f>& vec, int rows, int cols);
	/** \overload \brief Copy Contructor: Builds new mesh from provided mesh.*/
	Mesh(const Mesh& m);
	/** \brief Builds rectangular mesh from retangluar bounds with the given number of segments on each side.*/
	void makeInterpolatedMesh(const cv::Rect& bounds, int segments);
	/** \brief Draws the mesh onto the provided image Mat*/
	void drawMesh(cv::Mat& img);
	/** \brief Generates a vector of 2d points from the mesh. Vector has length rows * cols.*/
	vector<cv::Point2f> getVectorFromMesh();
	/** \brief  Builds mesh from vector of 2d points with the provided number of rows and cols. This is not interpolated.*/
	void makeMeshFromVector(const vector<cv::Point2f>& vec, int cols, int rows);
	/** \brief  Returns the 3d Point Vector at the provided Mesh coordinates.*/
	cv::Vec3f& at(int r, int c);
	/** \brief  Returns the number of Rows in the Mesh*/
	int getRows();
	/** \brief  Returns the number of Cols in the Mesh*/
	int getCols();
	/** \brief  Finds the nearest Mesh index with 2d coordinates closet to the provided point.*/
	static cv::Vec2i getNearestPointCoordinates(cv::Point p, cv::Mat& mesh);
	/** \brief  Returns THe distance between two 2d points*/
	static float distanceToPoint(cv::Point p1, cv::Point p2);
	/** \brief  Returns the rectangular bounds of a vector of 2d points.*/
	static cv::Rect getVectorBounds(vector<cv::Point>& data);

	/** The Data Mat for the Mesh Object. Each index holds a 3d Point Vector representing that points 3d locations.*/
	cv::Mat meshPoints;
	/** A vector which holds the control (measured) points of the mesh. The value stored in each index coresponds to that points index in meshPoints. */
	vector<cv::Vec2i> controlIndex; //Each point Vec2i in this vector stores the row and col of a control point in the mesh.

private:
	
	
	
	
	
};

#endif
