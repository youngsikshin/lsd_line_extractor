#pragma once
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace line_tracker
{

class LineTracker
{
public:
    LineTracker(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~LineTracker() { }

    void image_callback(const sensor_msgs::ImageConstPtr& img_msg);
    void reset();

private:
    image_transport::ImageTransport *it_;
    image_transport::Subscriber sub_;
};

}