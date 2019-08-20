#include <line_tracker/LineTracker.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/core/utility.hpp>

namespace line_tracker
{

LineTracker::LineTracker(ros::NodeHandle nh, ros::NodeHandle private_nh)
{
    std::string thermal_config_file;
    private_nh.getParam("thermal_cfg_file", thermal_config_file);

    ROS_INFO("Reading paramerter of thermal camera from %s", thermal_config_file.c_str());

    // thermal_config_file = "/home/irap/drone_ws/src/line_tracker/config/thermal_config.yaml";
    cv::FileStorage thermal_fs(thermal_config_file, cv::FileStorage::READ);
    if(!thermal_fs.isOpened()) {
        ROS_ERROR("Cannot open config files.");
    }

    int width = thermal_fs["Camera.width"];
    std::cout << "width : " << width << std::endl;
    ROS_INFO("%d", width);

    it_ = new image_transport::ImageTransport(nh);
    sub_ = it_->subscribe("image_raw", 10, &LineTracker::image_callback, this);
    // private_nh.param<std::string>("cfg_file", str_cfg_file_, "/filtered_pc");
}

void LineTracker::image_callback(const sensor_msgs::ImageConstPtr& img_msg)
{
    ROS_INFO("Image Callback");

    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img_msg, img_msg->encoding);

    cv::Mat gray_cv8u, histeq_img;

    double kT0 = 273.0;
    double R = 426707.0;
    double B = 1428.0;
    double F = 1.0;
    double O = -247.525;
    double min_t =  25.; // 10 33 , 12 25, -1.0 33
    double max_t = 40.;
    int raw_min = R / (std::exp(B / (min_t + kT0)) - F) + O;
    int raw_max = R / (std::exp(B / (max_t + kT0)) - F) + O;
    const double alpha = 255.0 / (raw_max - raw_min);
    const double beta = -alpha * raw_min;
    cv_ptr->image.convertTo(gray_cv8u, CV_8UC1, alpha, beta);
    cv::equalizeHist(gray_cv8u, histeq_img);

    /* create a ramdom binary mask */
    cv::Mat mask = cv::Mat::ones( cv_ptr->image.size(), CV_8UC1 );

    /* create a pointer to a BinaryDescriptor object with deafult parameters */
    /* create a pointer to an LSDDetector object */
    cv::Ptr<cv::line_descriptor::LSDDetector> lsd = cv::line_descriptor::LSDDetector::createLSDDetector();
    std::vector<cv::line_descriptor::KeyLine> keylines;

    lsd->detect( histeq_img, keylines, 2, 2 );
    // cv::Ptr<cv::BinaryDescriptor> bd = cv::BinaryDescriptor::createBinaryDescriptor();


    for(std::vector<cv::line_descriptor::KeyLine>::iterator it=keylines.begin(); it != keylines.end();) {
        ROS_INFO("%f", it->lineLength);
        if(it->lineLength < 30.0) {
            it = keylines.erase(it);
        } else {
            it++;
        }
    }

    cv::Mat out_line_img;
    cv::line_descriptor::drawKeylines(gray_cv8u, keylines, out_line_img);

    cv::namedWindow("test");
    cv::imshow("test", out_line_img);
    cv::waitKey(1);
}
void LineTracker::reset()
{

}

}