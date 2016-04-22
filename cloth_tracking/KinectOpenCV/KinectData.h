/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0 and Kinect SDK 1.8
*	
*	This is a wrapper class to simplify the accquisition of Kinect Data from the Kinect v1 Sensor.
*	Collected Color and Depth frames are converted to OpenCV Mat objects for use with that package.
*	The example code this was based off of can be found at http://www.cs.princeton.edu/~edwardz/tutorials/kinect/kinect1.html 
*	Class Files: KinectData.h , KinectData.cpp
*/
#ifndef KINECTDATA_H
#define KINECTDATA_H

#include <Windows.h>
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace std;

/**
* \brief The Kinect Data class provides simplified calling functions to access Kinect Color and Depth data in OpenCV 3.0
*/
class KinectData
{
public:
	/** \brief The default constructor*/
	KinectData();
	/** \brief The destructor*/
	~KinectData();
	/** \brief Intializes the Kinect Sensor. Must be called directly after construction*/
	bool initKinect();
	/** \brief Returns an OpenCV mat of the Kinect Color Data: Mat type is CV_8UC4 */
	cv::Mat getColorData();
	/** \brief Returns an OpenCV mat of the rectified Kinect Depth Data: Mat type is CV_16UC1 */
	cv::Mat getDepthImage();
	/** \brief Convienance Function to print out the relevant connection Errors of type HRESULT*/
	void printHRESULT(HRESULT error);
private:
	/*! The width of both the Depth and Color frames from the sensor.*/
	int width = 640;
	/*! The height of both the Depth and Color frames from the sensor.*/
	int height = 480;
	/*! The total pixel count of the Kinect frames. = width * height*/
	const int pxCount = 307200;
	/*! A dynamically allocated array which holds the output px location of the nth index of the depthPixel array.
		Used for rectification of the depth data with the color frame*/
	NUI_COLOR_IMAGE_POINT* mappedPoint;
	/*! A dynamically allocated array which holds the depth in mm of the nth pixel in the Kinect depth frame.
		Used for rectification of the depth data with the color frame*/
	NUI_DEPTH_IMAGE_PIXEL* depthPixel;
	/*! The rgb color stream used to fetch color frames from the Kinect*/
	HANDLE rgbStream;
	/*! The depth color stream used to fetch depth frames from the Kinect*/
	HANDLE depthStream;           
	/*! The Kinect SDK sensor object that is wrapped by this (KinectData) class*/
	INuiSensor* kSensor; 
};


#endif