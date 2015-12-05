//
// Created by naveed on 15. 4. 30.
//

#ifndef PROCESSING_H
#define PROCESSING_H


#include "../utilities/utilities.h"


// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <ros/param.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <depth_image_proc/depth_traits.h>

// Messages
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
//#include <sensor_msgs/point_cloud2_iterator.h>

// CvBridge
#include "cv_bridge/cv_bridge.h"

// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Boost
#include "boost/foreach.hpp"
#include "boost/multi_array.hpp"
#include "boost/container/vector.hpp"

// PCL
#include "pcl_conversions/pcl_conversions.h"
#include "pcl-1.8/pcl/common/pca.h"

// Eigen
//#include "eigen3/Eigen/Core"
#include <limits>

// ZMQ
//#include <zmq.hpp>

namespace Biolab
{

    class Processing
    {
    private:
        ros::Publisher pub, pub_strings;

        Parameters p;

        cv::Mat depthToImage(const cv::Mat &depthImage);

        MatrixType cvMatToMatrix(const cv::Mat &depthImage);


        PointType depthToPointXYZ(const PointType &depthImagePoint);
        CloudTypeRGB depthImageToPointCloud(const MatrixType &depthImage);
        void setCloudColor(CloudTypeRGB &cloud, const cv::Vec3b &color);


    public:
        Processing(ros::Publisher &pub, ros::Publisher &pub_strings);
        ~Processing();

        void process(const sensor_msgs::ImageConstPtr &msg);


    };
}

#endif /* PROCESSING_H */

