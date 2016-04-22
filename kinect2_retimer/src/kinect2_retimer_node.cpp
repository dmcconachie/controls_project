#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher kinect2_retimed_pub;

void kinectPointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& msg_in )
{
    sensor_msgs::PointCloud2 msg_out = *msg_in;
    msg_out.header.stamp = ros::Time::now();
    kinect2_retimed_pub.publish( msg_out );
}

int main( int argc, char** argv )
{
    // Read in all ROS parameters
    ros::init( argc, argv, "kinect_retimer_node" );

    ros::NodeHandle nh;

    ros::Subscriber kinect2_sub = nh.subscribe( "kinect2/sd/points", 1, kinectPointCloudCallback );
    kinect2_retimed_pub = nh.advertise< sensor_msgs::PointCloud2 >( "kinect2_retimed/sd/points", 1 );

    ros::spin();
}
