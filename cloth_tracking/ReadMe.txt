========================================================================
	CLOTH TRACKING WITH MICROSOFT KINECT AND OPENCV
	Initial Author: Michael McConnell
========================================================================
Before working on the project please review some of the videos below to understand how the output works.
	https://www.youtube.com/watch?v=KVIj2ylCRNI&list=PLu5VCNX1FuYyheZNEcz6NQrOL3ggEMHZw

Project Methodology Overview
	There is a PDF stored as an issue in this repo. To access it click on the issues tab and then click the Project Paper issue. 
	A download is available in the description.

	To gain a high level understanding of how the project is supposed to function please read the paper above.

Notes on Operation
	The poject requires the folling components.
	1. Microsoft Kinect v1
	2. Cloth with easily visible circular markers on it in a rectangular pattern.
	3. Microsoft Windows
	4. OpenCV 3.0
	5. Microsoft Kinect SDK v1.8
	6. Python 2.7 (If using the experimental 3d TPS)
	7. Visual Studio 2015 (Not really required but it was written in this.)

	The program was written for Microsoft Windows, but ideally the method could be transferred to a ROS platform. This would allow it to function on a robotic system.

	To run the code you will need to link the OpenCV libraries with your IDE. Similarly the tpyPython.py file will need to be imported to your python install. This can be done by moving it into the Lib folder of the Python folder. The Kinect SDK will also need to be properly installed on the system and added to your IDE. There is  a path to a template file in the code that also needs to be changed to a picture of your template cloth image.

	Once the program compiles and connects to the kinect it will follow the following steps.
	1. Console will open along with some blank openCV windows.

	2. The program will wait for input. 

	3. When input is given an 8sec countdown will begin.
		-This is meant to give you time to hold a cloth as in the example videos.

	4. When the countdown is complete the program will begin searing for the cloth by looking for all the marked points to be detected.
	
	5. When all points are detected a mesh will be overlaid on the cloth. 
		-At this point all the points are being tracked.
	
	6. You may begin to move the cloth and see some deformation.
	
	7. If the python code is set up for use, a 3d graph of the 3dTPS expectation will appear. This will be very noisy and is not meant to represent the final mesh for analysis. 
	
	8. When tracking the cloth a 3D python plot of the 2D TPS and Raw Despth data will appear. This is running off of the same file as the 3D TPS python file. The particular function that is being called is the display2DWarpWithDepth(X_m, Y_m, Z_m) python function. 	
	-In the videos a 3d representation of the mesh is plotted with matlab. This was done by exporting the mesh data to a CSV file along with time stamps by frame. This data can then be plotted in matlab. This is functionally the same as the new python plotting. Except the python plot will update in real time. 

	
	Another note. 
		The program currently does not process the template image at run time. The user instead enters the point’s locations in order from left to right then top to bottom. 
		1 2 3
		4 5 6
		7 8 9
		To automate it some code must be written to scan the image to preserve this ordering when the template is processed at start up.

		The filtering on the image will also need to be adjusted based on the lighting conditions of the room. This can be done in the filterImage function. 

		If you would like clarification on something feel free to email me 
		Michael McConnell: msmcconnell@wpi.edu

========================================================================
    CONSOLE APPLICATION : KinectOpenCV Project Overview
========================================================================

AppWizard has created this KinectOpenCV application for you.

This file contains a summary of what you will find in each of the files that
make up your KinectOpenCV application.


KinectOpenCV.vcxproj
    This is the main project file for VC++ projects generated using an Application Wizard.
    It contains information about the version of Visual C++ that generated the file, and
    information about the platforms, configurations, and project features selected with the
    Application Wizard.

KinectOpenCV.vcxproj.filters
    This is the filters file for VC++ projects generated using an Application Wizard. 
    It contains information about the association between the files in your project 
    and the filters. This association is used in the IDE to show grouping of files with
    similar extensions under a specific node (for e.g. ".cpp" files are associated with the
    "Source Files" filter).

KinectOpenCV.cpp
    This is the main application source file.

/////////////////////////////////////////////////////////////////////////////
Other standard files:

StdAfx.h, StdAfx.cpp
    These files are used to build a precompiled header (PCH) file
    named KinectOpenCV.pch and a precompiled types file named StdAfx.obj.

/////////////////////////////////////////////////////////////////////////////
Other notes:

AppWizard uses "TODO:" comments to indicate parts of the source code you
should add to or customize.

/////////////////////////////////////////////////////////////////////////////
