//
// Created by naveed on 15. 4. 30.
//

// ROS
#include <ros/ros.h>

// ROS message filters
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

// Messages
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>

// Processing
#include "../include/depth_to_pointcloud/processing.h"

namespace Biolab
{
    boost::shared_ptr<Processing> processing;
}

using namespace Biolab;

void checkTopicName(const std::string& s)
{
    if (s.compare("") == 0)
    {
        boost::format formatter("Topic %s is not defined");
        formatter % s;
        throw std::runtime_error(formatter.str());
    }
}

void depthCameraCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    processing->process(depth_msg);
}

int main(int argc, char **argv)
{
    ROS_INFO("DEPTH IMAGE TO POINT CLOUD NODE started");
    ros::init(argc, argv, "depth_to_pointcloud_node");
    ros::NodeHandle nh("~");

    // Set variables for parameters
    std::string inputDepthImage;
    std::string outputPointCloud;
    Parameters parameters;

    // Read parameters from launch file
    nh.param("inputDepthImage", inputDepthImage, std::string("output_depth_image"));
    nh.param("outputPointCloud", outputPointCloud, std::string("output_point_cloud"));


    // Check topic names for validity
    checkTopicName(inputDepthImage);
    checkTopicName(outputPointCloud);


    // Create subscriber
    ros::Subscriber sub = nh.subscribe(inputDepthImage.c_str(), parameters.queueLength, depthCameraCallback);

    // Create publisher
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>(outputPointCloud.c_str(), parameters.queueLength);



    // Create instance of the processing class
    processing = boost::shared_ptr<Processing>(new Processing(pub, parameters));
    ros::spin();
    return 0;
}