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
#include "stdafx.h"
#include "KinectData.h"
#include <comdef.h>
/** The default constructor intializes the kSensor object to NULL*/
KinectData::KinectData():
	kSensor(NULL)
{
}

/** The destructor sends the shutdown command to the KinectSensor*/
KinectData::~KinectData()
{
	if (kSensor)
	{
		kSensor->NuiShutdown();
	}

	delete kSensor;
}
/** Intializes the Kinect Sensor. Must be called directly after construction.
*	@return Returns TRUE for successful intialization. Returns FALSE on failure.
*/
bool KinectData::initKinect() {
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) {
		cout << "No Sensors" << endl; return false;
	}
	if (NuiCreateSensorByIndex(0, &kSensor) < 0) {
		cout << "Failed to create sensor by index" << endl;  return false;
	}

	// Initialize sensor
	kSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	HRESULT error = kSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,            // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,    // Image resolution
		0,      // Image stream flags, e.g. near mode
		2,      // Number of frames to buffer
		NULL,   // Event handle
		&rgbStream);
	HRESULT errorDepth = kSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,            // Depth camera
		NUI_IMAGE_RESOLUTION_640x480,    // Image resolution
		0,      // Image stream flags, e.g. near mode
		2,      // Number of frames to buffer
		NULL,   // Event handle
		&depthStream);
	//Dynamically allocate Depth Mapping arrays
	mappedPoint = (NUI_COLOR_IMAGE_POINT*) malloc(sizeof(NUI_COLOR_IMAGE_POINT) * pxCount);
	depthPixel = (NUI_DEPTH_IMAGE_PIXEL*) malloc(sizeof(NUI_DEPTH_IMAGE_PIXEL) * pxCount);
	return kSensor;
}

/** Function to returns an OpenCV mat of the Kinect Color Data.
*	@return cv::Mat of type CV_8UC4 representing the Color Image frame from the sensor.*/
cv::Mat KinectData::getColorData()
{
	cv::Mat colorImg(height, width, CV_8UC4); //Create Mat to hold color data.
	//Kinect Frame Variables
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;
	//Get the next color frame from the rgb stream. Use timeout of 200ms
	HRESULT hr = kSensor->NuiImageStreamGetNextFrame(rgbStream, 200, &imageFrame);
	if (hr != S_OK) //Check that the frame is valid.
	{
		cout << "NoStream" << endl;
		return colorImg;
	}
	INuiFrameTexture* texture = imageFrame.pFrameTexture; //Get the texture data from the frame.
	texture->LockRect(0, &LockedRect, NULL, 0); //Lock the data
	if (LockedRect.Pitch != 0) //Check the image exists
	{
		byte* buffer = (byte*)LockedRect.pBits; //Copy the data directly into OpenCV mat object.
		colorImg = cv::Mat(height, width, CV_8UC4, buffer).clone();
	}

	texture->UnlockRect(0); //Unlock data
	kSensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame); //Release stream
	return colorImg; 
}

/** Function to returns an OpenCV mat of the rectified Kinect Dapth Data.
*	@return cv::Mat of type CV_16UC1 representing the Depth Image frame from the sensor.*/
cv::Mat KinectData::getDepthImage()
{
	
	cv::Mat depthImg(height, width, CV_16UC1);//Create Mat to hold depth data.
	//Kinect Frame Variables
	NUI_IMAGE_FRAME depthFrame;
	NUI_LOCKED_RECT LockedRect;
	//Get the next depth frame from the depth stream. Use timeout of 200ms
	HRESULT hr = kSensor->NuiImageStreamGetNextFrame(depthStream, 200, &depthFrame);
	if (hr != S_OK) //Check that frame is valid
	{
		//cout << "NoStream" << endl;
		printHRESULT(hr);
		return depthImg;
	}
	//Get a coordinate mapper to convert depth -> color coord. Rectify the images.
	INuiCoordinateMapper* coordMapper;
	kSensor->NuiGetCoordinateMapper(&coordMapper);
	//Get texture data
	INuiFrameTexture* texture = depthFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0); //Lock data
	if (LockedRect.Pitch != 0) //Check that data is valid
	{
		//Get starting and ending data locations
		const USHORT* curr = (const USHORT*)LockedRect.pBits;
		const USHORT* dataEnd = curr + (width*height);
		int i = 0;
		//Search through the data
		while (curr < dataEnd) {
			// Get depth in millimeters
			USHORT depth = NuiDepthPixelToDepth(*curr++);
			depth *= 8; //Map from range 0-2^13 to range 0-2^16. This is used to allows for better visualization (gray scale) of depth data.
			depthPixel[i].depth = depth; //Copy the depth data to the depth array.
			i++;
		}
		//Map the depth data to the color image.
		hr = coordMapper->MapDepthFrameToColorFrame(NUI_IMAGE_RESOLUTION_640x480, pxCount, depthPixel, NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, pxCount, mappedPoint);
		//Copy the rectified depth data to the output Mat
		for (int n = 0; n < pxCount; n++)
		{
			depthImg.at<USHORT>(mappedPoint[n].y, mappedPoint[n].x) = (USHORT)depthPixel[n].depth;
		}
	}
	//Unlock and release frame.
	texture->UnlockRect(0); 
	kSensor->NuiImageStreamReleaseFrame(depthStream, &depthFrame);
	return depthImg;
}

/** Convienance Function to print out the relevant connection Errors of type HRESULT
*	@param error The HRESULT error that is to be printed. Not all HRESULTs are covered. Only those with application in this class. 
*/
void KinectData :: printHRESULT(HRESULT error)
{
	switch (error) {
	case S_OK:
		cout << "S_OK" << endl; break;
	case E_FAIL:
		cout << "E_FAIL" << endl; break;
	case E_INVALIDARG:
		cout << "E_INVALIDARG" << endl; break;
	case E_NUI_DEVICE_NOT_READY:
		cout << "E_NUI_DEVICE_NOT_READY" << endl; break;
	case E_OUTOFMEMORY:
		cout << "E_OUTOFMEMORY" << endl; break;
	case E_POINTER:
		cout << "E_POINTER" << endl; break;
	case S_FALSE:
		cout << "S_FALSE" << endl; break;
	case E_NUI_FRAME_NO_DATA:
		cout << "E_NUI_FRAME_NO_DATA" << endl; break;
	default:
		cout << "SomethingWentWrong" << endl; break;
	}
}
