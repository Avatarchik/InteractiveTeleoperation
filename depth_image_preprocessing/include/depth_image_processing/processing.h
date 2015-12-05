#ifndef PROCESSING_H
#define PROCESSING_H


#include "utilities.h"
#include "json_interop.h"

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
#include <sensor_msgs/CameraInfo.h>
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

    struct PinholeCameraModelComputed {
        PointType compute(const image_geometry::PinholeCameraModel &model);
        float center_x;
        float center_y;
        float constant_x;
        float constant_y;
        float bad_point;
    };

    static int clockwiseTraceSequenceY[] = {0, -1, -1, -1, 0, 1, 1, 1};
    static int clockwiseTraceSequenceX[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    static int counterclockwiseTraceSequenceY[] = {0, 1, 1, 1, 0, -1, -1, -1};
    static int counterclockwiseTraceSequenceX[] = {-1, -1, 0, 1, 1, 1, 0, -1};

    class Processing
    {
    private:
        boost::shared_ptr<JSONInterop> jsonInterop;

        ros::Publisher pub, pub_strings;

        Parameters p;

        cv::RNG rng;  // Random value generator

        PinholeCameraModelComputed model;

        cv::Mat depthToImage(const cv::Mat &depthImage);

        ListOfPointClouds traceBorders(const MatrixType &matrix, int distanceThreshold = 10);
        MatrixType cvMatToMatrix(const cv::Mat &depthImage);

        void traceBorderFromPoint(const MatrixType &matrix, const PairXYType &pointCurrent, const bool isClockwise, PairXYSetType &markedPixels, CloudType &border, int height, int width, int distanceThreshold = 10);

        cv::Mat drawBorders(const cv::Mat &depthImage, ListOfPointClouds &borders);

        PointType depthToPointXYZ(const PointType &depthImagePoint);
        CloudTypeRGB borderToPointCloud(const CloudType &border, const cv::Vec3b &color);
        ListOfPointCloudsRGB bordersToPointClouds(const ListOfPointClouds &borders);
        CloudTypeRGB reduceBorderClouds(const ListOfPointCloudsRGB &borderClouds);
        CloudTypeRGB findCircles(const CloudTypeRGB &border);
        CloudTypeRGB findCirclesEigen(const CloudTypeRGB &border);
        CloudTypeRGB depthImageToPointCloud(const MatrixType &depthImage);
        void setCloudColor(CloudTypeRGB &cloud, const cv::Vec3b &color);

        void doSaltNoiseRemoval(MatrixType &depthMatrix);
        MatrixType doDepthOutlierRemoval(const MatrixType &depthMatrix, int ksize = 3, int neighborsCount = 4, int depthThreshold = 7);
        MatrixType doMedianFiltering(const MatrixType &depthMatrix, int ksize = 3);

        std_msgs::String createStringMessage(const std::string &message);  //TODO: move this function to Utilities

    public:
        Processing(ros::Publisher &pub, ros::Publisher &pub_strings);
        ~Processing();

        void process(const sensor_msgs::ImageConstPtr &msg);
        void process(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg);


    };
}

#endif /* PROCESSING_H */
