//
// Created by naveed on 15. 4. 30.
//

#include "../../include/depth_to_pointcloud/processing.h"

namespace Biolab
{

    Processing::Processing(ros::Publisher &pub, ros::Publisher &pub_strings)
    {

        this->pub = pub;
        this->pub_strings = pub_strings;
        //    cv::namedWindow("original", cv::WINDOW_NORMAL);
        //    cv::namedWindow("edges", cv::WINDOW_NORMAL);
        cv::namedWindow("borders", cv::WINDOW_NORMAL);

        this->jsonInterop = boost::shared_ptr<JSONInterop>(new JSONInterop());
    }

    Processing::~Processing()
    {

    }


    void Processing::process(const sensor_msgs::ImageConstPtr &msg)
    {
        // min and max values for distance MIN = 0, MAX = 10000 (distance is in mm)
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "16SC1");
        cv::Mat cvImage = cv_ptr->image;
        MatrixType matrix = this->cvMatToMatrix(cvImage);

        CloudTypeRGB fullCloud = this->depthImageToPointCloud(matrix);

        // Publish results
        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg(fullCloud, cloud2);
        cloud2.header.frame_id = msg->header.frame_id;
        this->pub.publish(cloud2);

    }



/** Set all points to have color specified in 'color' parameter, in RGB order
 * @brief Processing::setCloudColor
 * @param cloud
 * @param color
 */
    void Processing::setCloudColor(CloudTypeRGB &cloud, const cv::Vec3b &color)
    {
        for (CloudTypeRGB::iterator it = cloud.begin(); it != cloud.end(); ++it)
        {
            it->r = color[0];
            it->g = color[1];
            it->b = color[2];
        }
    }




    PointType Processing::depthToPointXYZ(const PointType &depthImagePoint)
    {
        float xx, yy, zz;
        float depth = depthImagePoint.z;
        xx = (depthImagePoint.x - this->model.center_x) * depth * this->model.constant_x;
        yy = (depthImagePoint.y - this->model.center_y) * depth * this->model.constant_y;
        zz = depth_image_proc::DepthTraits<uint16_t>::toMeters(depth);
        PointType point(xx, yy, zz);
        return point;
    }


/** Convert depth image to point cloud with all points painted with white color
 * @brief Processing::depthImageToPointCloud
 * @param depthImage
 * @return
 */

    CloudTypeRGB Processing::depthImageToPointCloud(const MatrixType &depthImage)
    {
        CloudTypeRGB cloud;

//    cv::Size size = depthImage.size();
        int height = (int)depthImage.shape()[0];
        int width = (int)depthImage.shape()[1];


        // TODO: this procedure may be optimized using iterators
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int depthValue = depthImage[y][x];
                if (depthValue == 0)
                {
                    continue;
                }
                PointType pointDepth(x, y, depthValue);
                PointType point3d = this->depthToPointXYZ(pointDepth);
                PointTypeRGB point3dColor(255, 255, 255);
                point3dColor.x = point3d.x;
                point3dColor.y = point3d.y;
                point3dColor.z = point3d.z;
                cloud.push_back(point3dColor);
            }
        }

        return cloud;
    }

}

