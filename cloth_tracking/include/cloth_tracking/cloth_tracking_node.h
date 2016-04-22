#ifndef _CLOTH_TRACKING_NODE_H_
#define _CLOTH_TRACKING_NODE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <cloth_tracking/Mesh.h>

struct filterImageParam {
    int erode_iter;
    int dilate_iter;
    cv::Scalar lower_bound;
    cv::Scalar upper_bound;
};


void getColorData(const sensor_msgs::PointCloud2::ConstPtr& input_msg, cv::Mat& color_image);
void filterImage(cv::Mat& src, cv::Mat& dst, struct filterImageParam& para);
std::vector<cv::KeyPoint> blobDetection(cv::Mat& im, cv::SimpleBlobDetector::Params& params);
void initKalmanFilter(std::vector<cv::KalmanFilter>& filters, const std::vector<cv::KeyPoint>& blobs);
void updateKalmanFilter(std::vector<cv::KalmanFilter>& filters, const std::vector<cv::KeyPoint>& blobs);
int getNearestFilterIndex(const cv::KeyPoint& blob, const std::vector<cv::KalmanFilter>& filters);
int getNearestBlobIndex(const cv::KalmanFilter& filter, const std::vector<cv::KeyPoint>& blobs);
cv::Rect getMarkersBound(const std::vector<cv::KeyPoint>& blobs);
Mesh initMesh(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, const std::vector<cv::KalmanFilter>& kfiters);
Mesh updateMesh(cv::Ptr<cv::ThinPlateSplineShapeTransformer>& tpsWarp, Mesh& mesh, const std::vector<cv::KalmanFilter>& kfiters);
void drawNumbersOnPoints(cv::Mat& img, const std::vector<cv::KalmanFilter>& kfiters);

#endif // _CLOTH_TRACK_NODE_H_
