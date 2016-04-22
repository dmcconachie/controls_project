/**
*	Author Michael McConnell
*	Date: 10/9/2015
*	SDK's: OpenCV 3.0, Kinect SDK 1.8, Python C API
*
*	This file contains the main execution code for the cloth tracking project.
*/

#include "stdafx.h"

#include <Python.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include "KinectData.h"
#include "DBSCAN.h"
#include "PtDBSCAN.h"
#include "Mesh.h"
#include <iostream>
#include <fstream>

using namespace std;

#define TARGET_BOUND_WIDTH 313 /**The expected width of the cloth being  tracked on start up in pixels.*/
#define BOUND_WIDTH_BUFFER 30 /** The range +/- from the TARGET_BOUND_WIDTH that the cloth can be in at start up.*/
#define MAX_POINT_COUNT 9 /**The maximum number of detectable points that can be found on the cloth.*/

//Kalman filer state variables
const int KF_stateSize = 6; /**The number of variables in the kalman state. x_pos, y_pos, x_vel, y_vel, x_acc, y_acc*/
const int KF_meassureSize = 2;/**The number of variable in the measurement state of the kalman filter. x_pos, y_pos*/
const int KF_contrSize = 0; /**The number of variables in the kalman control input (none it is purely detection based)*/
const unsigned int KF_type = CV_32F; /**The data type used for the Kalman filter */

/** \brief Helper function to filter the image by HSV color.*/
void filterImage(cv::Mat& src, cv::Mat& dst);
/** \brief Function to detect the markers on the cloth*/
vector<cv::KeyPoint> blobDetection(cv::Mat& im, cv::SimpleBlobDetector::Params& params);
/** \brief Gets the bounding box of the points in a cluster.*/
cv::Rect getClusterBounds(Cluster& data);
/** \brief Gets the scale between two rectangles*/
cv::Vec2f getScale(cv::Rect r1, cv::Rect r2);
/** \brief Applys the x_y scale to all the points in a cluster.*/
void applyScaleToCluster(Cluster& c, cv::Vec2f scale);
/** \brief Returns the manually entered cloth marker locations from the template image.*/
Cluster getTemplateImgPointsManual();
/** \brief Function loads in a template image from the provided file.*/
void loadTemplateImage(string& path);
/** \brief Displays a countdown on the comand prompt till till the porgram starts*/
void countDown(int ticks);
/** \brief Moves a cluster to the provided x_y location*/
void moveClusterToPoint(Cluster& c, int x, int y);
/** \brief Draws a cluster on the screen.*/
void drawCluster(cv::Mat& img, const Cluster& c, cv::Scalar color);
/** \brief Identifies correlated points with the input cluster and the video feed.*/
void correlatePoints(const Cluster& controlCluster, Cluster& outputCluster);
/** \brief Calculates the distance between two PtDBSCAN*/
float distanceToPoint(PtDBSCAN p1, PtDBSCAN p2);
/** \brief draws the correlation number on the points in a cluster.*/
void drawNumbersOnPoints(cv::Mat& img, const Cluster& c);
/** \brief Updates the Kalman Filter*/
cv::Mat updateKalmanFilter(cv::KalmanFilter& KF, cv::Mat_<float>& measurement, int measureX, int measureY);
/** \brief Finds the index of the nearest point in a cluster to the provided point.*/
int getNearestPointIndex(PtDBSCAN p, const Cluster& c);
/** \brief Finds the index of the nearest point in a vector of Matrices*/
int getNearestPointIndex(PtDBSCAN p, const vector<cv::Mat_<float>>& c);
/** \brief Intializes the Kalman Filter*/
void initKalmanFilter(vector<cv::KalmanFilter>& filters, vector<cv::Mat_<float>>& measurements, vector<cv::Mat_<float>>& states, const Cluster& cluster);
/** \brief Gets the cluster resized from the template image.*/
Cluster  getResizedCluster(const Cluster& cluster, vector<cv::Mat_<float>> prevStates, int maxPointCount);
/** \brief Gets the intial mesh from a template image. (Might not be used)*/
Mesh getInitialMeshFromTemplate(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, cv::Rect bounds, int segments, Cluster templatePoints);
/** \brief Converts a cluster to a vector of points*/
vector<cv::Point> clusterToPointVector(Cluster& c);
/** \brief Generates a new mesh based on the TPS warp.*/
void updateMeshInitial(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, Mesh& mesh, vector<cv::Point> newPoints, vector<cv::DMatch>& good_matches);
/** \brief Assigns the depth information to a mesh*/
void assignDepthToMesh(Mesh& mesh, const cv::Mat& depthFrame);
/** \brief Assigns the depth information to a mesh with a frame count*/
void assignDepthToMesh(Mesh& mesh, const cv::Mat& depthFrame, int frameCount);

void display2DTPSRawDepthInPython(Mesh& mesh, PyObject* function);
/** Builds a python object representing the tacked control points of the mesh from a vector of those control points.
	@param vec A vector of the 3d locations of the control points used for TPS warping.
	@return A python list object containing the same data.
*/
PyObject* buildPythonControlListFromVector(vector<cv::Vec3f> vec)
{
	PyObject* myList = PyList_New(0);
	for (int i = 0; i < vec.size(); i++)
	{
		PyObject* xValue = PyFloat_FromDouble((double)vec.at(i)[0]);
		PyObject* yValue = PyFloat_FromDouble((double)vec.at(i)[1]);
		PyObject* zValue = PyFloat_FromDouble((double)vec.at(i)[2]);
		PyObject* pointList = PyList_New(3);
		PyList_SetItem(pointList, 0, xValue);
		PyList_SetItem(pointList, 1, yValue);
		PyList_SetItem(pointList, 2, zValue);
		PyList_Append(myList, pointList);
	}
	return myList;
}
/** Builds a python 2d list object representing the mesh object data stored for the porvided 3d axis (x, y, or z)
	@param mesh A mesh object representing the cloth
	@param axis the of the mesh data that the python list will contain
	@return A python 2d list object containing data for the provided axis of that mesh
*/
PyObject* buildPythonMeshForAxis(Mesh& mesh, int axis)
{

		PyObject* rowList = PyList_New(0);
		for (int r = 0; r < mesh.getRows(); r++)
		{
			PyObject* colList = PyList_New(0);
			for (int c = 0; c < mesh.getCols(); c++)
			{
				PyObject* value = PyFloat_FromDouble((double)mesh.at(r, c)[axis]);
				PyList_Append(colList, value);
			}
			PyList_Append(rowList, colList);
		}
		return rowList;
}
/** Function updates a provided mesh object with the 3D TPS reprsentation. This calls a external python file to perform the calculations.
	This function is not complete and at this time only calls the python file. This will create a visual representation of the 3d tps but has not yet been integrated back into the program
	@param mesh the current cloth mesh object
	@param meshControlNew the set of new control points
	@param goodMatches the list of control points that match up with old control points and can therefore be used to update the mesh.
	@param function a pointer to the python function that will be called to do the calculations.
	*/
void updateMeshTPS3D(Mesh& mesh, vector<cv::Vec3f> meshControlNew, vector<cv::DMatch> goodMatches, PyObject* function)
{
	//cout << "Size of meshControlNew = " << meshControlNew.size() << endl;
	//cout << "GoodMatches Are" << endl;
	for (int i = 0; i < goodMatches.size(); i++)
	{
		cout << "(a,b) = ( " << goodMatches[i].queryIdx << " , " << goodMatches[i].trainIdx<< " )" << endl;
	}

	if (goodMatches.size() < 3) //Must be at least 3 points to calculate a TPS
	{
		return;
	}

	vector<cv::Vec3f> oldPoints;
	
	for (int i = 0; i < mesh.controlIndex.size(); i++) //Get the control points last location
	{
		for (int j = 0; j < goodMatches.size(); j++)
		{
			if (i == goodMatches.at(j).queryIdx)//This point is know to be a good match
			{
				cv::Vec2i loc = mesh.controlIndex[i];
				oldPoints.push_back(cv::Vec3f(mesh.at(loc[0], loc[1])[0], mesh.at(loc[0], loc[1])[1], mesh.at(loc[0], loc[1])[2]));
				break;
			}
		}
	}
	//cout << "The old control Points are" << endl;
	for (int i = 0; i < oldPoints.size(); i++)
	{
		//cout << "(x,y,z) = ( " << oldPoints[i][0] << " , " << oldPoints[i][1] << " , " << oldPoints[i][2] << " )" << endl;
	}
	PyObject* oldControlPoints = buildPythonControlListFromVector(oldPoints);

	vector<cv::Vec3f> newPoints;
	for (int i = 0; i < meshControlNew.size(); i++) //Get the control point
	{
		for (int j = 0; j < goodMatches.size(); j++)
		{
			if (i == goodMatches.at(j).queryIdx)//This point is know to be a good match
			{
				newPoints.push_back(meshControlNew[i]);
				break;
			}
		}
	}
	//cout << "The new control Points are" << endl;
	for (int i = 0; i < newPoints.size(); i++)
	{
		//cout << "(x,y,z) = ( " << newPoints[i][0] << " , " << newPoints[i][1] << " , " << newPoints[i][2] << " )" << endl;
	}
	PyObject* newControlPoints = buildPythonControlListFromVector(newPoints);
	PyObject* xMesh = buildPythonMeshForAxis(mesh, 0);
	PyObject* yMesh = buildPythonMeshForAxis(mesh, 1);
	PyObject* zMesh = buildPythonMeshForAxis(mesh, 2);

	if (PyCallable_Check(function))
	{
		//cout << "The function is callable" << endl;
	}
	else
	{
		std::cout << "ERROR: The function is not a callable type" << std::endl;
	}
	if (function && PyCallable_Check(function)) //Call the function with the provided paramaters.
	{
		PyObject* pArgs = PyTuple_New(5);
		PyTuple_SetItem(pArgs, 0, oldControlPoints);
		PyTuple_SetItem(pArgs, 1, newControlPoints);
		PyTuple_SetItem(pArgs, 2, xMesh);
		PyTuple_SetItem(pArgs, 3, yMesh);
		PyTuple_SetItem(pArgs, 4, zMesh);
		PyObject* result = PyObject_CallObject(function, pArgs);
	}
	else
	{
		cout << "It was not a callable function" << endl;
		cout << "The function is = " << function;
	}
}
void display2DTPSRawDepthInPython(Mesh& mesh, PyObject* function)
{
	PyObject* X_mesh = buildPythonMeshForAxis(mesh, 0);
	PyObject* Y_mesh = buildPythonMeshForAxis(mesh, 1);
	PyObject* Z_mesh = buildPythonMeshForAxis(mesh, 2);

	PyObject* pArgs = PyTuple_New(3);
	PyTuple_SetItem(pArgs, 0, X_mesh);
	PyTuple_SetItem(pArgs, 1, Y_mesh);
	PyTuple_SetItem(pArgs, 2, Z_mesh);
	PyObject* result = PyObject_CallObject(function, pArgs);

}
/** The main execution function*/
int main(int argc, char** argv)
{
		
	Py_Initialize(); //Intialize communication with the python vm

	PyObject* myModuleString = PyString_FromString((char*)"tpsPython"); //Get module by name
	PyObject* myModule = PyImport_Import(myModuleString); //Import module
	PyObject* function;
	function = PyObject_GetAttrString(myModule, (char*)"displayWarp"); //Get function from module
	PyObject* displayFunction;
	displayFunction = PyObject_GetAttrString(myModule, (char*)"display2DWarpWithDepth"); //Get function from module
	if (displayFunction && PyCallable_Check(displayFunction)) //Call the function with the provided paramaters.
	{
		cout << "Hello we have Dfunction" << endl;
	}
	if (function && PyCallable_Check(function)) //Call the function with the provided paramaters.
	{
		cout << "Hello we have   function" << endl;
	}
	PyObject* initFunction = PyObject_GetAttrString(myModule, (char*)"init"); //Get function from module
	PyObject* result = PyObject_CallObject(initFunction, PyTuple_New(0));

    string refImgPath = "C:\\Users\\Stonemotmot\\Pictures\\3DTPSPython.png"; //The path to the template image. It is currently not being used as the template points are entered manually.
	string mainWindow = "MainWindow";
	DBSCAN myScan; //DBSCAN-er that will be used to find trackable points
	cv::Mat frame, depthFrame; //The matrices holding the video image data.
	KinectData myKinect; //The kinect object

	//loadTemplateImage(refImgPath); //Should only be used for dynamic point correlation. Currently points are entered manually

	Cluster templateCluster = getTemplateImgPointsManual(); //The cluster representing the template control points
	cv::Rect templateBounds = getClusterBounds(templateCluster); //The bounds of the template control points
	//calculates the warp of the template image when an actual template image is being used.
	cv::Ptr<cv::ThinPlateSplineShapeTransformer> myTPS = cv::createThinPlateSplineShapeTransformer(0);
	Mesh meshTemplate(getInitialMeshFromTemplate(myTPS, templateBounds, 10, templateCluster));
	cv::Mat templateImage = cv::imread(refImgPath);
	meshTemplate.drawMesh(templateImage);
	imshow("Template Image", templateImage);
	//Intialize kinect image. 
	myKinect.initKinect();// Show our image inside it.
	//Kalman Filter
	vector<cv::KalmanFilter> filters;
	vector<cv::Mat_<float>> measurements;
	vector<cv::Mat_<float>> states;
	vector<int> filterLostCount;
	// Set up the detector with default parameters.
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 0.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 120.0f; //15x15 =225
	params.maxArea = 500.0f; //20x20

	bool runOnce = true;//Boolean flips after the cloth is first detected
	bool hasRun = false;
	bool runOnceCountDown = true;
	int ticks = (double)cv::getTickCount();
	int frameCount = 0; //Number of frames that have passed.
	Mesh mesh; //The mesh object which represents the detected cloth.
	//Opperational Loop
	while (1)
	{
		double precTick = ticks;
		ticks = (double)cv::getTickCount();

		double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
		//Kinect Get Image-----------------------------------------------------
		myKinect.getColorData().copyTo(frame);
		myKinect.getDepthImage().copyTo(depthFrame);
		
		if (runOnceCountDown) //Waits until the user provides input to start the countdown.
		{
			cv::imshow("Kinect", frame);
			cv::waitKey(50);
			cin.get();
			countDown(8);
			runOnceCountDown = false;
		}
		//Filter Image---------------------------------------------------------
		cv::Mat imgHSV, biImg, grey, diImg;
		cv::cvtColor(frame, imgHSV, CV_BGR2HSV);
		filterImage(imgHSV, biImg);
		erode(biImg, diImg, cv::Mat());
		dilate(diImg, diImg, cv::Mat());
		dilate(diImg, diImg, cv::Mat());

		//DBSCAN and Clustering---------------------------------------------------------------
		vector<cv::KeyPoint> keyPoints = blobDetection(diImg, params);
		Cluster keyPointCluster = PtDBSCAN::keyPoint2PtDBSCAN(keyPoints);
		vector<Cluster> groups = myScan.scan(keyPointCluster, 80, 2);//here?
		if (groups.size() != 0)
		{
			int maxSize = 0;
			int bigCluster = 0;
			for (int i = 0; i < groups.size(); i++)
			{
				int size = groups[i].size();
				if (i == 0)
				{
					maxSize = size;
				}
				else if (size > maxSize)
				{
					maxSize = size;
					bigCluster = 1;
				}
			}
			cv::Rect bounds = getClusterBounds(groups[bigCluster]);
			int detectedPointCount = groups[bigCluster].size();

			//Correlation-----------------------------------------------------------------------------------------------------------
			if (runOnce && detectedPointCount == MAX_POINT_COUNT && bounds.width < (TARGET_BOUND_WIDTH + BOUND_WIDTH_BUFFER)
				&& bounds.width > (TARGET_BOUND_WIDTH - BOUND_WIDTH_BUFFER)) //Runs when all points on the cloth are detected for the first time.
			{
				cv::Mat show = frame.clone();
				//Overlay the scaled template on the detected cloth
				cv::Vec2f scale = getScale(bounds, templateBounds);
				applyScaleToCluster(templateCluster, scale);
				moveClusterToPoint(templateCluster, bounds.x, bounds.y);

				runOnce = false;
				hasRun = true;
				Cluster testVec = groups[bigCluster];

				drawCluster(show, testVec, cv::Scalar(0, 255, 0));
				correlatePoints(templateCluster, groups[bigCluster]); // The cloth Cluster should now be sorted in the poper order
				drawNumbersOnPoints(show, groups[bigCluster]);

			    //Kalman Filter Setup----------------------------------------------------------------------------------------------------
				initKalmanFilter(filters, measurements, states, groups[bigCluster]);
				filterLostCount.reserve(groups[bigCluster].size());
				cout << "Correlated Cluster" << endl;
				for (int i = 0; i < groups[bigCluster].size(); i++)
				{
					cout << "(x,y) = ( " << groups[bigCluster][i].kPt.pt.x << " , " << groups[bigCluster][i].kPt.pt.y << " )" << endl;
					//cv::circle(show, cv::Point(prevEstimates[i].kPt.pt.x, prevEstimates[i].kPt.pt.y), 4, cv::Scalar(0, 255, 0), 2);
				}
				cout << "Filter" << endl;
				for (int i = 0; i < filters.size(); i++)
				{
					cout << "(x,y,z) = ( " << filters[i].statePost.at<float>(0)<< " , " << filters[i].statePost.at<float>(1) << " , " <<depthFrame.at<USHORT>(filters[i].statePost.at<float>(1), filters[i].statePost.at<float>(0)) / 8 << " )" << endl;
					cv::circle(depthFrame, cv::Point2f(filters[i].statePost.at<float>(0), filters[i].statePost.at<float>(1)), 4, cv::Scalar(255), 2);
				}
				mesh = getInitialMeshFromTemplate(myTPS, bounds, 8, groups[bigCluster]); //Get the intial mesh from the template and tps
				mesh.drawMesh(show); 
				//cv::imshow("CorrelatedPooints", show);
				//cv::imshow("Depth", depthFrame);
				assignDepthToMesh(mesh, depthFrame, frameCount); //Add the depth data to the detected mesh
				cv::waitKey(50);
			//-----------------------------------------------------------------------------------------------------------------------
			}
			else if(hasRun) //Runs continuously after the cloth was first detected.
			{
				//Recorrelation Through Filter----------------------------------------------------------------------------------------
				Cluster clothCluster = getResizedCluster(groups[bigCluster], states, MAX_POINT_COUNT);
				vector<cv::Point> meshControlNew;
				vector<cv::DMatch> good_matches;
				//Update Kalman Filters
				for (int i = 0; i < filters.size(); i++)
				{

					states[i] = filters[i].predict();//need check for detection then statePost = state

					cv::circle(frame, cv::Point(clothCluster[i].kPt.pt.x, clothCluster[i].kPt.pt.y), 4, cv::Scalar(0, 0, 0), 2);
					if (clothCluster[i].kPt.pt.x == -1 && clothCluster[i].kPt.pt.y == -1) //No Point Detected
					{
						filters[i].statePost = states[i]; // The new state is the predicted state;
						int nearPointIndex = getNearestPointIndex(PtDBSCAN(states[i].at<float>(0), states[i].at<float>(1)), states);

						clothCluster[i] = PtDBSCAN(states[i].at<float>(0), states[i].at<float>(1));
						filterLostCount[i]++;
						if (filterLostCount[i] > 5) //Stop velocity after 5 frames of no detection
						{
							filters[i].statePost.at<float>(2) = 0; //Stop Movement
							filters[i].statePost.at<float>(3) = 0;
							filters[i].statePost.at<float>(4) = 0;
							filters[i].statePost.at<float>(5) = 0;
						}

					}
					else
					{
						measurements[i].at<float>(0) = clothCluster[i].kPt.pt.x;
						measurements[i].at<float>(1) = clothCluster[i].kPt.pt.y;
						filters[i].statePost = filters[i].correct(measurements[i]);
						good_matches.push_back(cv::DMatch(i, i, 0));
						filterLostCount[i] = 0;
					}

					cv::putText(frame, std::to_string(i), cv::Point(filters[i].statePost.at<float>(0), filters[i].statePost.at<float>(1)), cv::QT_FONT_NORMAL, 1, cv::Scalar(0, 0, 255));
					cv::circle(frame, cv::Point(filters[i].statePost.at<float>(0), filters[i].statePost.at<float>(1)), 4, cv::Scalar(0, 0, 255), 2);
					meshControlNew.push_back(cv::Point(filters[i].statePost.at<float>(0), filters[i].statePost.at<float>(1)));
				}


				
				vector<cv::Vec3f> newControlPointTPS;
				//cout << "Mesh control INdex size = " << mesh.controlIndex.size() << endl;
				for (int i = 0; i < mesh.controlIndex.size(); i++) //Get the control points last location
				{

					cv::Vec2i loc = mesh.controlIndex[i];
					newControlPointTPS.push_back(cv::Vec3f(meshControlNew.at(i).x, meshControlNew.at(i).y, mesh.at(loc[0], loc[1])[2]));
				}

				//updateMeshTPS3D(mesh, newControlPointTPS, good_matches, function); //Run the mesh update functions for the 3d TPS with python
				
				updateMeshInitial(myTPS, mesh, meshControlNew, good_matches); //Update mesh with 2d tps in opencv
				
				assignDepthToMesh(mesh, depthFrame, frameCount); //Add depth data to mesh
				display2DTPSRawDepthInPython(mesh, displayFunction);

				mesh.drawMesh(frame);
			
				//----------------------------------------------------------------------------------------------------------------------
				
			}
			
		}
		else
		{
			//no Clusters
		}
		cv::imshow("Kinect", frame);
		cv::waitKey(3); //This is a required line to allow everything to draw correctly.
		frameCount++;
	}                                          // Wait for a keystroke in the window
	Py_Finalize(); //Close python
	cin.get();
	
	return 0;
}
/** Assigns the depth information to a mesh with a frame count
	@param mesh the cloth mesh object that will have the depth data added to it
	@param depthFrame the depth information from the kinect
	@param frameCount the current frame number
*/

void assignDepthToMesh(Mesh& mesh, const cv::Mat& depthFrame, int frameCount)
{
	static bool run = false;
	ofstream myfile;
	myfile.open("data.csv", ios::app);
	if (run == false)
	{
		myfile << "frame, x, y, z" << endl;
		run = true;
	}
	int sum = 0;
	int count = 0;
	for (int r = 0; r < mesh.getRows(); r++)
	{
		for (int c = 0; c < mesh.getCols(); c++)
		{
			cv::Vec3f vec = mesh.at(r, c);
			if (vec[1] < depthFrame.rows && vec[0] < depthFrame.cols && vec[1] >= 0 && vec[0] >= 0)
			{
				vec[2] = depthFrame.at<USHORT>(vec[1], vec[0]) / 8;
				sum += vec[2];
			}
			else
			{
				vec[2] = (double)sum / (double)count;
			}
			mesh.at(r, c) = vec;
			myfile << frameCount <<"," << vec[0] << "," << vec[1] << "," << vec[2] << endl;
			count++;

		}
	}
	//myfile << mesh.meshPoints << endl;
	myfile.close();
}
/**  Gets the intial mesh from a template image
	@param tpsWarp The tps transformer that will be used to warp the mesh
	@param bounds a bounding rectangle 
	@param segments the number of vertices to add to the mesh on each side.
	@param templatePoints the points from the template image.
*/
Mesh getInitialMeshFromTemplate(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, cv::Rect bounds, int segments, Cluster templatePoints)
{
	Mesh mesh(bounds, segments);
	vector<cv::DMatch> good_matches;
	vector<cv::Point> points2;
	for (int i = 0; i < MAX_POINT_COUNT; i++)
	{
		good_matches.push_back(cv::DMatch(i, i, 0));
	}
	for (int i = 0; i < templatePoints.size(); i++)
	{
		cv::Vec2i loc = Mesh::getNearestPointCoordinates(templatePoints[i], mesh.meshPoints);
		mesh.controlIndex.push_back(loc);
		points2.push_back(cv::Point(mesh.at(loc[0], loc[1])[0], mesh.at(loc[0], loc[1])[1]));
	}
	tpsWarp->estimateTransformation(points2, clusterToPointVector(templatePoints), good_matches);
	vector<cv::Point2f> meshVec1 = mesh.getVectorFromMesh();
	vector<cv::Point2f> meshVec2(meshVec1.size());
	tpsWarp->applyTransformation(meshVec1, meshVec2);
	Mesh mesh2(meshVec2, mesh.getRows(), mesh.getCols());
	mesh2.controlIndex = mesh.controlIndex;
	return mesh2;
}
void updateMeshInitial(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, Mesh& mesh, vector<cv::Point> newPoints, vector<cv::DMatch>& good_matches)
{
	//new points are assumed to be ordered
	vector<cv::Point> points1;
	for (int i = 0; i < mesh.controlIndex.size(); i++) //Get the control points last location
	{
		cv::Vec2i loc = mesh.controlIndex[i];
		points1.push_back(cv::Point(mesh.at(loc[0], loc[1])[0], mesh.at(loc[0], loc[1])[1]));
	}
	tpsWarp->estimateTransformation(points1, newPoints, good_matches);
	vector<cv::Point2f> meshVec1 = mesh.getVectorFromMesh();
	vector<cv::Point2f> meshVec2(meshVec1.size());
	tpsWarp->applyTransformation(meshVec1, meshVec2);
	Mesh(meshVec2, mesh.getRows(), mesh.getCols()).meshPoints.copyTo(mesh.meshPoints); //Update the mesh locations

}
/**  Converts a cluster to a vector of points
@param c the cluster to be converted
@return a vector of 2d points derived from the cluster.
*/
vector<cv::Point> clusterToPointVector(Cluster& c)
{
	vector<cv::Point> vec;
	for (int i = 0; i < c.size(); i++)
	{
		vec.push_back((cv::Point)(c[i]));
	}
	return vec;
}
Cluster  getResizedCluster(const Cluster& cluster, vector<cv::Mat_<float>> prevStates, int maxPointCount)
{
	Cluster resizedCluster;
	for (int i = 0; i < maxPointCount; i++) //Put Place Holder Points in All parts of the vector.
	{
		resizedCluster.push_back(PtDBSCAN(-1, -1));
	}
	int size = cluster.size();

	if (size <= maxPointCount)
	{
		for (int i = 0; i < cluster.size(); i++)
		{
			int index = getNearestPointIndex(cluster[i], prevStates); //Find index of corresponding prevEstimate
			resizedCluster[index] = cluster[i]; //Put current point into that index on resized vector.
		}
		//All points not used are marked -1,-1	
		return resizedCluster;
	}
	else //There are more points than the desired cluster size
	{
		for (int i = 0; i < prevStates.size(); i++)
		{
			int index = getNearestPointIndex(PtDBSCAN(prevStates[i].at<float>(0), prevStates[i].at<float>(1)), cluster); //decide which points to keep based on the prevEstimates. 
			resizedCluster[i] = cluster[i];
		}
		return resizedCluster;
	}
}

int getNearestPointIndex(PtDBSCAN p, const Cluster& c)
{
	int min = 0;
	int minIndex = 0;
	for (int i = 0; i < c.size(); i++)
	{
		float distance = distanceToPoint(c[i], p);
		if (i == 0)
		{
			min = distance;
			minIndex = 0;
		}
		else if (distance < min)
		{
			min = distance;
			minIndex = i;
		}
	}
	return minIndex;
}
int getNearestPointIndex(PtDBSCAN p, const vector<cv::Mat_<float>>& c)
{
	int min = 0;
	int minIndex = 0;
	for (int i = 0; i < c.size(); i++)
	{
		float distance = distanceToPoint(PtDBSCAN(c[i].at<float>(0), c[i].at<float>(1)), p);
		if (i == 0)
		{
			min = distance;
			minIndex = 0;
		}
		else if (distance < min)
		{
			min = distance;
			minIndex = i;
		}
	}
	return minIndex;
}
cv::Rect getClusterBounds(Cluster& data)
{//Find MidPoint and Bounding Box
	int minX, minY, maxX, maxY;
	int sumX = 0, sumY = 0;
	for (int i = 0; i < data.size(); i++)
	{
		if (i == 0)
		{
			minX = data[i].kPt.pt.x;
			minY = data[i].kPt.pt.y;
			maxX = data[i].kPt.pt.x;
			maxY = data[i].kPt.pt.y;
		}
			sumX += data[i].kPt.pt.x;
			sumY += data[i].kPt.pt.y;
		if (data[i].kPt.pt.x < minX)
		{
			minX = data[i].kPt.pt.x;
		}
		else if (data[i].kPt.pt.x > maxX)
		{
			maxX = data[i].kPt.pt.x;
		}

		if (data[i].kPt.pt.y < minY)
		{
			minY = data[i].kPt.pt.y;
		}
		else if (data[i].kPt.pt.y > maxY)
		{
			maxY = data[i].kPt.pt.y;
		}
	}
	//cv::Point2f midPoint(sumX / data.size, sumY / data.size);
	cv::Rect bounds = cv::Rect(cv::Point2f(minX, minY), cv::Point2f(maxX, maxY));
	return bounds;
}


void filterImage(cv::Mat& src, cv::Mat& dst)
{
	cv::inRange(src, cv::Scalar(30, 150, 50), cv::Scalar(50, 255, 255), dst);	
}

vector<cv::KeyPoint> blobDetection(cv::Mat& im, cv::SimpleBlobDetector::Params& params)
{
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
	//	cv::Size(2 * 0 + 1, 2 * 0 + 1),
	//	cv::Point(0, 0));
	//cv::dilate(im, im, element);
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	//cout << im.depth << endl;
	// Detect blobs.
	std::vector<cv::KeyPoint> keypoints;
	
	detector->detect(im, keypoints);
	return keypoints;
	//cv::waitKey(0);
}

//need to adjust for my model
void initKalmanFilter(vector<cv::KalmanFilter>& filters, vector<cv::Mat_<float>>& measurements, vector<cv::Mat_<float>>& states, const Cluster& cluster)
{

	for (int i = 0; i < cluster.size(); i++)
	{
		cv::KalmanFilter KF = cv::KalmanFilter(KF_stateSize, KF_meassureSize, KF_contrSize);
		// intialization of KF...
		KF.transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 1, 0, .5, 0,
														0, 1, 0, 1, 0, .5,
														0, 0, 1, 0, 1,  0,
														0, 0, 0, 1, 0,  1,
														0, 0, 0, 0, 1,  0,
														0, 0, 0, 0, 0,  1);
		/*											 << 1, 0, 1, 0, .5, 0,
														0, 1, 0, 1, 0, .5,
														0, 0, 1, 0, 1,  0,
														0, 0, 0, 1, 0,  1,
														0, 0, 0, 0, 1,  0,
														0, 0, 0, 0, 0,  1);*/
		//cv::setIdentity(KF.transitionMatrix);
		cv::Mat state(KF_stateSize, 1, KF_type);  // [x,y,v_x,v_y,a_x,a_y]
		cv::Mat meas(KF_meassureSize, 1, KF_type);    // [x, y]
		KF.statePre.at<float>(0) = cluster[i].kPt.pt.x;
		KF.statePre.at<float>(1) = cluster[i].kPt.pt.y;
		KF.statePre.at<float>(2) = 0;
		KF.statePre.at<float>(3) = 0; 
		KF.statePre.at<float>(4) = 0;
		KF.statePre.at<float>(5) = 0;

		KF.statePost.at<float>(0) = cluster[i].kPt.pt.x;
		KF.statePost.at<float>(1) = cluster[i].kPt.pt.y;
		KF.statePost.at<float>(2) = 0;
		KF.statePost.at<float>(3) = 0;
		KF.statePost.at<float>(4) = 0;
		KF.statePost.at<float>(5) = 0;

		/*
		[1,0,0,0]
		[0,1,0,0]
		[0,0,0,0] //?
		[0,0,0,0] //?
		*/
		KF.measurementMatrix = cv::Mat::zeros(KF_meassureSize, KF_stateSize, KF_type);
		KF.measurementMatrix.at<float>(0) = 1.0f;
		KF.measurementMatrix.at<float>(7) = 1.0f;

		/*KF.processNoiseCov.at<float>(0) = 1e-2;
		KF.processNoiseCov.at<float>(5) = 1e-2;
		KF.processNoiseCov.at<float>(10) = 2.0f;
		KF.processNoiseCov.at<float>(15) = 1.0f;*/
		/// Ex or Q
		setIdentity(KF.processNoiseCov, cv::Scalar::all(100));


		/// Initial covariance estimate Sigma_bar(t) or P'(k)
		setIdentity(KF.errorCovPre, cv::Scalar::all(1));

		/// Sigma(t) or P(k)
		setIdentity(KF.errorCovPost, cv::Scalar::all(1));
		setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1));

		filters.push_back(KF);
		meas.at<float>(0) = cluster[i].kPt.pt.x;
		meas.at<float>(1) = cluster[i].kPt.pt.y;

		state.at<float>(0) = meas.at<float>(0);
		state.at<float>(1) = meas.at<float>(1);
		state.at<float>(2) = 0;
		state.at<float>(3) = 0;
		state.at<float>(4) = 0;
		state.at<float>(5) = 0;
		measurements.push_back(meas);
		states.push_back(state);
	}
}

cv::Mat updateKalmanFilter(cv::KalmanFilter& KF, cv::Mat_<float>& measurement, int measureX, int measureY)
{
	cv::Mat prediction = KF.predict();
	cv::Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

	measurement(0) = measureX;
	measurement(1) = measureY;

	// The update phase 
	cv::Mat estimated = KF.correct(measurement);
	return estimated;
}
void loadTemplateImage(string& path)
{
	DBSCAN myScan;
	cv::Mat refImg;
	refImg = cv::imread(path);
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 30.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 500.0f; //15x15 =225
	params.maxArea = 1400.0f; //20x20
	cv::Mat imgHSV, biImg, grey, diImg;
	cv::cvtColor(refImg, imgHSV, CV_BGR2HSV);
	cv::inRange(imgHSV, cv::Scalar(0, 90, 160), cv::Scalar(11, 255, 255), biImg);//filter the image
	dilate(biImg, diImg, cv::Mat());

	cv::Mat im_with_keypoints;
	vector<cv::KeyPoint> keypoints = blobDetection(diImg, params);
	cv::drawKeypoints(refImg, keypoints, im_with_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	// Show blobs
	vector<Cluster> groups = myScan.scan(PtDBSCAN::keyPoint2PtDBSCAN(keypoints), 80, 2);
	if (groups.size() != 0)
	{
		int maxSize = 0;
		int bigCluster = 0;
		for (int i = 0; i < groups.size(); i++)
		{
			int size = groups[i].size();
			if (i == 0)
			{
				maxSize = size;
			}
			else if (size > maxSize)
			{
				maxSize = size;
				bigCluster = 1;
			}
		}
		cv::Rect bounds = getClusterBounds(groups[bigCluster]);
		cv::rectangle(im_with_keypoints, bounds, cv::Scalar(0, 255, 0), 2);
		//cout << "Num Points On Template " << groups[bigCluster].size() << endl;
		//imshow("Template Image", im_with_keypoints);
		//	cv::waitKey(3);
		//cv::waitKey(0);
	}
}
Cluster getTemplateImgPointsManual()
{
	/*
		1	2	3
		4	5	6
		7	8	9
	*/
	Cluster organizedPoints;
	organizedPoints.push_back(PtDBSCAN(28, 12)); //1
	organizedPoints.push_back(PtDBSCAN(311, 27)); //2
	organizedPoints.push_back(PtDBSCAN(607, 36)); //3
	organizedPoints.push_back(PtDBSCAN(20, 209)); //4
	organizedPoints.push_back(PtDBSCAN(309, 213)); //5
	organizedPoints.push_back(PtDBSCAN(583, 213)); //6
	organizedPoints.push_back(PtDBSCAN(19, 383)); //7
	organizedPoints.push_back(PtDBSCAN(300, 393)); //8
	organizedPoints.push_back(PtDBSCAN(587, 390)); //9
	return organizedPoints;
}
cv::Vec2f getScale(cv::Rect r1, cv::Rect r2)
{
	cv::Vec2f scale;
	scale[0] = (float)r1.width / (float)r2.width;
	scale[1] = (float)r1.height / (float)r2.height;
	return scale;
}

void applyScaleToCluster(Cluster& c, cv::Vec2f scale)
{
	//Assuming c[0] is top left point
	int xOffset = c[0].kPt.pt.x;
	int yOffset = c[0].kPt.pt.y;
	for (int i = 0; i < c.size(); i++) // Move everything to (0,0) refernce and scale
	{
		c[i].kPt.pt.x = (c[i].kPt.pt.x - xOffset) * scale[0];
		c[i].kPt.pt.y = (c[i].kPt.pt.y - yOffset) * scale[1];
	}

}
//Assumes Cluster top left point is at 0,0
void moveClusterToPoint(Cluster& c, int x, int y)
{
	for (int i = 0; i < c.size(); i++) // Move everything to refernece point
	{
		c[i].kPt.pt.x += x;
		c[i].kPt.pt.y += y;
	}
}
void countDown(int ticks)
{
	for (int i = 1; i <= ticks; i++)
	{
		cout << "Count " << std::to_string(i) << endl;
		cv::waitKey(1000);
	}
}

void drawCluster(cv::Mat& img, const Cluster& c, cv::Scalar color)
{
	for (int i = 0; i < c.size(); i++)
	{
		cv::circle(img, cv::Point(c[i].kPt.pt.x, c[i].kPt.pt.y), 2, color, 2);
	}
}

void correlatePoints(const Cluster& controlCluster, Cluster& outputCluster)
{
	if (controlCluster.size() != outputCluster.size())
	{
		cout << "Error: Correlated Cluster's are not of equal size" << endl;
		return;
	}
	vector<int> locCorrespondance(controlCluster.size());///!!! probably not needed
	for (int i = 0; i < controlCluster.size(); i++)
	{
		int min = 0;
		int minIndex = 0;
		for (int j = 0; j < outputCluster.size(); j++)
		{
			float distance = distanceToPoint(outputCluster[j], controlCluster[i]);
			if (j == 0)
			{
				min = distance;
				minIndex = 0;
			}
			else if (distance < min)
			{
				min = distance;
				minIndex = j;
			}
		}
		//Swap The points
		PtDBSCAN tempPt = outputCluster[i];
		outputCluster[i] = outputCluster[minIndex];
		outputCluster[minIndex] = tempPt;

	}
}
float distanceToPoint(PtDBSCAN p1, PtDBSCAN p2)
{
	return sqrt(pow(p2.kPt.pt.x - p1.kPt.pt.x, 2) + pow(p2.kPt.pt.y - p1.kPt.pt.y, 2));
}
void drawNumbersOnPoints(cv::Mat& img, const Cluster& c)
{
	for (int i = 0; i < c.size(); i++)
	{
		cv::putText(img, std::to_string(i), cv::Point(c[i].kPt.pt.x, c[i].kPt.pt.y), cv::QT_FONT_NORMAL, 1, cv::Scalar(0, 0, 255));
	}
}
