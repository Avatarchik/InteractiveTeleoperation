// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <ros/param.h>

// ROS message filters
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Messages
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>


// Processing
#include "../include/depth_image_processing/processing.h"

// CvBridge
#include "cv_bridge/cv_bridge.h"

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Boost
#include <boost/thread.hpp>


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

void depthCameraCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    processing->process(depth_msg, info_msg);
}


int main(int argc, char **argv)
{
    ROS_INFO("DEPTH IMAGE PROCESSING NODE started");
    ros::init(argc, argv, "depth_image_processing_node");
    ros::NodeHandle nh("~");

    std::string input, output, info, depth_camera, string_messages;
    nh.param("input", input, std::string("/camera/depth/image_rect_raw"));
    checkTopicName(input);
    nh.param("camera_info", info, std::string("/camera/depth/camera_info"));
    nh.param("output", output, std::string("/www"));
    nh.param("string_messages", string_messages, std::string("/string_messages"));
    int queue_size = 5;
    nh.param("queue_size", queue_size, 5);
    checkTopicName(output);

//    image_transport::ImageTransport it(nh);
//    boost::mutex connect_mutex;

    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, input.c_str(), queue_size);
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, info.c_str(), queue_size);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync;

    approximate_sync.reset(new ApproximateSync(ApproximatePolicy(queue_size), depth_sub, info_sub));
    approximate_sync->registerCallback(boost::bind(depthCameraCallback, _1, _2));

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(output.c_str(), 1);
    ros::Publisher pub_strings = nh.advertise<std_msgs::String>(string_messages.c_str(), 1);
    processing = boost::shared_ptr<Processing>(new Processing(pub, pub_strings));

    ros::spin();
    return 0;
}
