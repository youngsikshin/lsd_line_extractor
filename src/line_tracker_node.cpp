#include <iostream>
#include <ros/ros.h>
#include <line_tracker/LineTracker.h>

using namespace std;
using namespace line_tracker;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_tracker");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    cout <<"line tracker node" << endl;
    LineTracker l_tracker(nh, private_nh);

    ros::spin();
    
    return 0;
}