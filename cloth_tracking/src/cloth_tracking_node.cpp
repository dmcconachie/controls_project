#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <smmap_msgs/messages.h>
#include "cloth_tracking/cloth_tracking_node.h"

#define MAX_MARKER_NUM 9
#define MESH_SEGMENTS 8

//Kalman filer and variables
std::vector<cv::KalmanFilter> kfilters;
const int KF_stateSize = 4; //Dimension of state: x_pos, y_pos, x_vel, y_vel, x_acc, y_acc
const int KF_observationSize = 2;//Dimension of observation: x_pos, y_pos
const int KF_controlSize = 0; //Dimension of control
const unsigned int KF_type = CV_32F; //The data type used for the Kalman filter
std::vector<int> filterLostCount (MAX_MARKER_NUM, 0);

//Mesh and variables
Mesh mesh;
cv::Ptr<cv::ThinPlateSplineShapeTransformer> myTPS = cv::createThinPlateSplineShapeTransformer(0);

filterImageParam filim_param;
cv::SimpleBlobDetector::Params blob_param;

bool bGotAllMarkers = false;

void processPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg, ros::Publisher& cloth_configuration_pub)
{
//    std::cout<<"get some data\n";
    cv::Mat rgb_image, filter_image;
    std::vector<cv::KeyPoint> blobs;

    // Get RGB image from PointCloud2 message
    getColorData(input_msg, rgb_image);
    cv::Mat test_image;
    cv::cvtColor(rgb_image, test_image, CV_BGR2HSV);
    cv::imwrite("./image.jpg", test_image);

    // Filter image using bound values of in hsv, erosion and dilation
    filterImage(rgb_image, filter_image, filim_param);

    // Blob detection
    blobs = blobDetection(filter_image, blob_param);

    // Start the tracking process only after the first time we find all 9 markers
    if (bGotAllMarkers) {
        updateKalmanFilter(kfilters, blobs);
        mesh = updateMesh(myTPS, mesh, kfilters);
        for (int i=0; i<kfilters.size(); i++) {
            cv::circle(rgb_image, cv::Point(kfilters[i].statePost.at<float>(0), kfilters[i].statePost.at<float>(1)), 4, cv::Scalar(255, 0, 0), 2);
        }
    }
    else {
        if (blobs.size()==MAX_MARKER_NUM) {
            bGotAllMarkers = true;
            initKalmanFilter(kfilters, blobs);
            mesh = initMesh(myTPS, kfilters);
        }
        for (int i=0; i<blobs.size(); i++) {
            cv::circle(rgb_image, cv::Point(blobs[i].pt.x, blobs[i].pt.y), blobs[i].size, cv::Scalar(0, 0, 255), 2);
        }
    }

    // Show the results on images
    mesh.drawMesh(rgb_image);
//    drawNumbersOnPoints(rgb_image, kfilters);
    cv::Mat resize_image(900, 1200, CV_8UC4);
    cv::resize(rgb_image, resize_image, resize_image.size());
    cv::imshow( "Display window", resize_image );
    cv::imshow( "Display window2", filter_image );
    cv::waitKey(30);

    // Output and publish
//    smmap_msgs::PointCloud output_msg;
//    output_msg.point_cloud.resize(9*9);
//    output_msg.point_cloud[0].x = 1;
//    output_msg.point_cloud[0].y = 1;
//    output_msg.point_cloud[0].z = 1;
//    cloth_configuration_pub.publish( output_msg );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cloth_tracking_node");
    ros::NodeHandle nh;

    // Set image filter params
    filim_param.lower_bound = cv::Scalar(0, 130, 130);
    filim_param.upper_bound = cv::Scalar(255, 210, 230);
    filim_param.erode_iter = 2;
    filim_param.dilate_iter = 3;
    // Set blob detection params
    blob_param.minDistBetweenBlobs = 0.0f;
    blob_param.filterByInertia = false;
    blob_param.filterByConvexity = false;
    blob_param.filterByColor = false;
    blob_param.filterByCircularity = false;
    blob_param.filterByArea = true;
    blob_param.minArea = 30.0f;
    blob_param.maxArea = 300.0f;


    ros::Publisher cloth_configuration_pub = nh.advertise<smmap_msgs::PointCloud>("cloth_configuration", 1);
    ros::Subscriber kinect_data_sub = nh.subscribe<sensor_msgs::PointCloud2>("kinect2/sd/points", 1,
                                                                             boost::bind( &processPointCloudCallback, _1, cloth_configuration_pub ) );
//    ros::spin();

    ros::Rate loop_rate(150);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
void getColorData(const sensor_msgs::PointCloud2::ConstPtr& input_msg, cv::Mat& color_image) {
    uint8_t buffer[input_msg->height*input_msg->width*4];
    int height = input_msg->height;
    int width = input_msg->width;

    for (int irow=0; irow<height; irow++) {
        for (int icol=0; icol<width; icol++) {
            buffer[4*(irow*width+icol)] = input_msg->data[input_msg->point_step*(irow*width+icol)+16];
            buffer[4*(irow*width+icol)+1] = input_msg->data[input_msg->point_step*(irow*width+icol)+17];
            buffer[4*(irow*width+icol)+2] = input_msg->data[input_msg->point_step*(irow*width+icol)+18];
            buffer[4*(irow*width+icol)+3] = input_msg->data[input_msg->point_step*(irow*width+icol)+19];
        }
    }
    cv::Mat(height, width, CV_8UC4, buffer).copyTo(color_image);
}
void filterImage(cv::Mat& src, cv::Mat& dst, struct filterImageParam& para) {
    cv::cvtColor(src, dst, CV_BGR2HSV);
    cv::inRange(dst, para.lower_bound, para.upper_bound, dst);
    cv::erode(dst, dst, cv::Mat(), cv::Point(-1,-1), para.erode_iter);
    cv::dilate(dst, dst, cv::Mat(), cv::Point(-1,-1), para.dilate_iter);
}
std::vector<cv::KeyPoint> blobDetection(cv::Mat& im, cv::SimpleBlobDetector::Params& params) {
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(im, keypoints);
    return keypoints;
}
void initKalmanFilter(std::vector<cv::KalmanFilter>& filters, const std::vector<cv::KeyPoint>& blobs) {

    for (int i = 0; i < blobs.size(); i++)
    {
        cv::KalmanFilter KF = cv::KalmanFilter(KF_stateSize, KF_observationSize, KF_controlSize);
        KF.transitionMatrix = (cv::Mat_<float>(KF_stateSize, KF_stateSize)
                               << 1, 0, 1, 0,
                                  0, 1, 0, 1,
                                  0, 0, 1, 0,
                                  0, 0, 0, 1);
        KF.statePre.at<float>(0) = blobs[i].pt.x;
        KF.statePre.at<float>(1) = blobs[i].pt.y;
        KF.statePre.at<float>(2) = 0;
        KF.statePre.at<float>(3) = 0;

        KF.statePost.at<float>(0) = blobs[i].pt.x;
        KF.statePost.at<float>(1) = blobs[i].pt.y;
        KF.statePost.at<float>(2) = 0;
        KF.statePost.at<float>(3) = 0;

        KF.measurementMatrix = cv::Mat::zeros(KF_observationSize, KF_stateSize, KF_type);
        KF.measurementMatrix.at<float>(0) = 1.0f;
        KF.measurementMatrix.at<float>(KF_stateSize+1) = 1.0f;

        /// Ex or Q
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(10));


        /// Initial covariance estimate Sigma_bar(t) or P'(k)
//        cv::setIdentity(KF.errorCovPre, cv::Scalar::all(1));

        /// Sigma(t) or P(k)
//        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));

        filters.push_back(KF);
    }
}
void updateKalmanFilter(std::vector<cv::KalmanFilter>& filters, const std::vector<cv::KeyPoint>& blobs) {
    std::vector<cv::KeyPoint> correlatedBlobs;
    std::vector<int> visited (0, MAX_MARKER_NUM);
    for (int i = 0; i < MAX_MARKER_NUM; i++) {
        //Put Place Holder Points in All parts of the vector.
        correlatedBlobs.push_back(cv::KeyPoint(-1, -1, 0));
    }

    if (blobs.size() <= MAX_MARKER_NUM) {
        for (int i = 0; i < blobs.size(); i++) {
            int index = getNearestFilterIndex(blobs[i], filters);
            correlatedBlobs[index] = blobs[i];
        }
    }
    else {
        //There are more points than the desired size
        for (int i = 0; i < filters.size(); i++) {
            int index = getNearestBlobIndex(filters[i], blobs);
            correlatedBlobs[i] = blobs[index];
        }
    }
//    for (int i = 0; i < MAX_MARKER_NUM; i++) {
//        int index = getNearestBlobIndex(filters[i], blobs, visited, threshold);
//        if (index != -1) {
//            correlatedBlobs[i] = blobs[index];
//        }
//    }

    for (int i = 0; i < filters.size(); i++) {
        cv::Mat_<float> states = filters[i].predict();//need check for detection then statePost = state
        if (correlatedBlobs[i].pt.x == -1 && correlatedBlobs[i].pt.y == -1) {
            //No Point Detected
            filterLostCount[i]++;
            if (filterLostCount[i] > 5) {
                //Set Velocity and Acceleration to 0, after 5 frames of no detection
                filters[i].statePost.at<float>(2) = 0;
                filters[i].statePost.at<float>(3) = 0;
            }
        }
        else {
            cv::Mat_<float> measurements(KF_observationSize, 1, KF_type);
            measurements.at<float>(0) = correlatedBlobs[i].pt.x;
            measurements.at<float>(1) = correlatedBlobs[i].pt.y;
//            filters[i].statePost = filters[i].correct(measurements);
            filters[i].correct(measurements);
            filterLostCount[i] = 0;
        }
    }
}
int getNearestFilterIndex(const cv::KeyPoint& blob, const std::vector<cv::KalmanFilter>& filters) {
    float distm;
    int index;
    for (int i=0; i<filters.size(); i++) {
        float dist = sqrt(pow(blob.pt.x - filters[i].statePost.at<float>(0), 2) + pow(blob.pt.y - filters[i].statePost.at<float>(1), 2));
        if (i==0) {
            distm = dist;
            index = i;
        }
        else if (dist<distm) {
            distm = dist;
            index = i;
        }
    }
    return index;
}
int getNearestBlobIndex(const cv::KalmanFilter& filter, const std::vector<cv::KeyPoint>& blobs) {//, std::vector<int>& visited, const float thrs) {
    float distm;
    int index;
    for (int i=0; i<blobs.size(); i++) {
        float dist = sqrt(pow(blobs[i].pt.x - filter.statePost.at<float>(0), 2) + pow(blobs[i].pt.y - filter.statePost.at<float>(1), 2));
        if (i==0) {
            distm = dist;
            index = i;
        }
        else if (dist<distm) {
            distm = dist;
            index = i;
        }
    }
    return index;
}
cv::Rect getMarkersBound(const std::vector<cv::KalmanFilter>& kfiters) {
    //Find Bounding Box
    float minX, minY, maxX, maxY;
    for (int i = 0; i < kfiters.size(); i++) {
        if (i == 0) {
            minX = kfiters[i].statePost.at<float>(0);
            minY = kfiters[i].statePost.at<float>(1);
            maxX = kfiters[i].statePost.at<float>(0);
            maxY = kfiters[i].statePost.at<float>(1);
        }
        if (kfiters[i].statePost.at<float>(0) < minX) {
            minX = kfiters[i].statePost.at<float>(0);
        }
        else if (kfiters[i].statePost.at<float>(0) > maxX) {
            maxX = kfiters[i].statePost.at<float>(0);
        }

        if (kfiters[i].statePost.at<float>(1) < minY) {
            minY = kfiters[i].statePost.at<float>(1);
        }
        else if (kfiters[i].statePost.at<float>(1) > maxY) {
            maxY = kfiters[i].statePost.at<float>(1);
        }
    }
    cv::Rect bounds = cv::Rect(cv::Point2f(minX, minY), cv::Point2f(maxX, maxY));
    return bounds;
}
Mesh initMesh(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, const std::vector<cv::KalmanFilter>& kfiters) {
    cv::Rect bounds = getMarkersBound(kfiters);
    Mesh mesh(bounds, MESH_SEGMENTS);

    std::vector<cv::Point2f> points1;
    for (int i = 0; i < kfiters.size(); i++) {
        cv::Point2f tempPt;
        tempPt.x = kfiters[i].statePost.at<float>(0);
        tempPt.y = kfiters[i].statePost.at<float>(1);
        cv::Vec2i loc = Mesh::getNearestPointCoordinates(tempPt, mesh.meshPoints);
        mesh.controlIndex.push_back(loc);
        points1.push_back(cv::Point(mesh.at(loc[0], loc[1])[0], mesh.at(loc[0], loc[1])[1]));
    }
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < kfiters.size(); i++) {
        cv::Point2f tempPt;
        tempPt.x = round(kfiters[i].statePost.at<float>(0));
        tempPt.y = round(kfiters[i].statePost.at<float>(1));
        points2.push_back(tempPt);
    }
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < kfiters.size(); i++) {
        matches.push_back(cv::DMatch(i, i, 0));
    }
    cv::Mat ptMat1(points1);
    cv::Mat ptMat2(points2);
    cv::transpose(ptMat1, ptMat1);
    cv::transpose(ptMat2, ptMat2);

    tpsWarp->estimateTransformation(ptMat1, ptMat2, matches);
    std::vector<cv::Point2f> meshVec1 = mesh.getVectorFromMesh();
    std::vector<cv::Point2f> meshVec2(meshVec1.size());
    tpsWarp->applyTransformation(meshVec1, meshVec2);
    Mesh mesh2(meshVec2, mesh.getRows(), mesh.getCols());
    mesh2.controlIndex = mesh.controlIndex;
    return mesh2;
}
Mesh updateMesh(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, Mesh& mesh, const std::vector<cv::KalmanFilter>& kfiters) {
    std::vector<cv::Point2f> points1;
    for (int i = 0; i < mesh.controlIndex.size(); i++) {
        //Get the control points last location
        cv::Vec2i loc = mesh.controlIndex[i];
        points1.push_back(cv::Point(mesh.at(loc[0], loc[1])[0], mesh.at(loc[0], loc[1])[1]));
    }
    std::vector<cv::Point2f> points2;
    for (int i = 0; i < kfiters.size(); i++) {
        cv::Point2f tempPt;
        tempPt.x = round(kfiters[i].statePost.at<float>(0));
        tempPt.y = round(kfiters[i].statePost.at<float>(1));
        points2.push_back(tempPt);
    }
    std::vector<cv::DMatch> matches;
    for (int i = 0; i < kfiters.size(); i++) {
        matches.push_back(cv::DMatch(i, i, 0));
    }
    cv::Mat ptMat1(points1);
    cv::Mat ptMat2(points2);
    cv::transpose(ptMat1, ptMat1);
    cv::transpose(ptMat2, ptMat2);

    tpsWarp->estimateTransformation(ptMat1, ptMat2, matches);
    std::vector<cv::Point2f> meshVec1 = mesh.getVectorFromMesh();
    std::vector<cv::Point2f> meshVec2(meshVec1.size());
    tpsWarp->applyTransformation(meshVec1, meshVec2);
    Mesh(meshVec2, mesh.getRows(), mesh.getCols()).meshPoints.copyTo(mesh.meshPoints);
    return mesh;
}
void drawNumbersOnPoints(cv::Mat& img, const std::vector<cv::KalmanFilter>& kfiters) {
    for (int i = 0; i < kfiters.size(); i++)
    {
        cv::putText(img, std::to_string(i), cv::Point2f(kfiters[i].statePost.at<float>(0), kfiters[i].statePost.at<float>(1)), cv::QT_FONT_NORMAL, 1, cv::Scalar(0, 0, 255));
    }
}
