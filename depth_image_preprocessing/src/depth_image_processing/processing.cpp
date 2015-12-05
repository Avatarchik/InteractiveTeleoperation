#include "../../include/depth_image_processing/processing.h"

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
//    // min and max values for distance MIN = 0, MAX = 10000 (distance is in mm)
//    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "16SC1");
//    cv::Mat cvImage = cv_ptr->image;

//    //    cv::Mat edges(cvImage.size(), CV_16SC1);
//    //    cv::Sobel(cvImage, edges, cvImage.depth(), 1, 1, 7);

//    // Good
//    //    cv::Mat filtered;
//    //    cv::medianBlur(cvImage, filtered, 5);

//    //    cv::Mat edges(cvImage.size(), CV_8UC1);
//    //    cv::Canny(depthToImage(cvImage), edges, 100, 500, 5);

//    //    cv::Mat displayImage = depthToImage(cvImage);
//    //    cv::imshow("original", displayImage);

//    //    cv::imwrite("/home/vprooks/Documents/depth.png", displayImage);  //TODO: deleteme
//    //    displayImage = depthToImage(edges);
//    //    cv::imshow("edges", displayImage);

//    // Get borders

//    ListOfPointClouds borders = traceBorders(cvImage);
//    cv::Mat bordersImage = drawBorders(cvImage, borders);
//    //        cv::imwrite("/home/vprooks/Documents/bordersvis.png", bordersImage);  //TODO: deleteme
//    cv::imshow("borders", bordersImage);
//    cv::waitKey(1);
    }


    void Processing::process(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
    {
        // min and max values for distance MIN = 0, MAX = 10000 (distance is in mm)
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "16SC1");
        cv::Mat cvImage = cv_ptr->image;
        MatrixType matrix = this->cvMatToMatrix(cvImage);

        image_geometry::PinholeCameraModel model;
        model.fromCameraInfo(info_msg);
        this->model.compute(model);

        //    // Do preprocessing, if necessary
        //    if (this->p.doMedianFiltering)
        //    {
        ////        cv::Mat filtered;
        //        cv::medianBlur(cvImage, cvImage, 7);
        //    }

        // Filter
        /////////

        // Remove salt noise
        if (this->p.saltRemovalEnabled)
        {
            this->doSaltNoiseRemoval(matrix);
        }

        // Remove depth outliers
        if (this->p.radiusOutlierRemovalEnabled)
        {
            matrix = this->doDepthOutlierRemoval(matrix);
        }

        // DO median filtering
        if (this->p.medianFilteringEnabled)
        {
            matrix = this->doMedianFiltering(matrix);
        }


        // Get borders

        ListOfPointClouds borders = traceBorders(matrix);
        cv::Mat bordersImage = drawBorders(cvImage, borders);
        cv::imshow("borders", bordersImage);
        cv::waitKey(1);

        // Convert each border into a point cloud
        ListOfPointCloudsRGB borderClouds = this->bordersToPointClouds(borders);

        // Select only circular borders
        ListOfPointCloudsRGB borderCloudsSelected;
        int counter = 0;
        cv::Vec3b redColor((uchar) 255, (uchar) 0, (uchar) 0);
        BOOST_FOREACH(CloudTypeRGB border, borderClouds)
                    {
                        // TODO: only add if there is non-empty result
                        //        CloudTypeRGB circularBorder = this->findCircles(border);
                        CloudTypeRGB circularBorder = this->findCirclesEigen(border);
                        if (circularBorder.size() > 0) {
                            this->setCloudColor(circularBorder, redColor);
                            borderCloudsSelected.push_back(circularBorder);
                            counter++;
                        }
                    }

        // Put all point clouds containing borders into a single point cloud
        //    CloudTypeRGB result = this->reduceBorderClouds(borderClouds);  // All borders
        CloudTypeRGB selectedBorderCloud = this->reduceBorderClouds(borderCloudsSelected);  // Only circluar borders
        CloudTypeRGB fullCloud = this->depthImageToPointCloud(matrix);
        CloudTypeRGB result;
        result += fullCloud;
        result += selectedBorderCloud;

        // Publish results
        sensor_msgs::PointCloud2 cloud2;
        pcl::toROSMsg(result, cloud2);
        cloud2.header.frame_id = msg->header.frame_id;
        this->pub.publish(cloud2);

        // Publish strings here
        std::string message;
        message = "Hello!";
        message = this->jsonInterop->PointCloudToJson(result);

        // TODO: pack point cloud to string here
        this->pub_strings.publish(this->createStringMessage(message));
    }

    std_msgs::String Processing::createStringMessage(const std::string &message)
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << message;
        msg.data = ss.str();
        return msg;
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


    PointType PinholeCameraModelComputed::compute(const image_geometry::PinholeCameraModel &model)
    {
        this->center_x = model.cx();
        this->center_y = model.cy();
        double unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
        this->constant_x = unit_scaling / model.fx();
        this->constant_y = unit_scaling / model.fy();
        this->bad_point = std::numeric_limits<float>::quiet_NaN();
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


/**
 * Get a depth image and create a grayscale image for display purposes.
 * Depth image is an image of CV_16SC1 type (or CV_16UC1) with min and max values from 0 to 10000.
 * This method maps the depth image to [0..256) half-segment (the output image is a grayscale
 * image of type CV_8UC1).
 */
    cv::Mat Processing::depthToImage(const cv::Mat &depthImage)
    {
        // Create output matrix containing zeros
        cv::Mat result(depthImage.size(), CV_8UC1);
        DepthValueType minValue = std::numeric_limits<DepthValueType>::max();
        DepthValueType maxValue = std::numeric_limits<DepthValueType>::min();
        cv::MatConstIterator_<DepthValueType> it, end;
        for (it = depthImage.begin<DepthValueType>(), end = depthImage.end<DepthValueType>(); it != end; ++it)
        {
            DepthValueType value = *it;
            if (value < minValue)
            {
                minValue = value;
            }
            if (value > maxValue)
            {
                maxValue = value;
            }
        }

        cv::MatIterator_<uchar> it2;  // iterator for the result image
        it2 = result.begin<uchar>();
        for (it = depthImage.begin<DepthValueType>(), end = depthImage.end<DepthValueType>(); it != end; ++it)
        {
            DepthValueType value = *it;
            double nominator = value - minValue;
            double denominator = maxValue - minValue;
            uchar normalizedValue = (uchar) (nominator / denominator * 255);
            *it2 = normalizedValue;
            *it2++;  // Go to next pixel in the result image
        }

        return result;
    }

    void Processing::doSaltNoiseRemoval(MatrixType &depthMatrix)
    {
        int height = depthMatrix.shape()[0];
        int width = depthMatrix.shape()[1];
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                DepthValueType *pointer = &(depthMatrix[y][x]);
                if (*pointer >= this->p.saltValue)
                {
                    *pointer = 0;
                }
            }
        }
    }


/**
 * @brief Processing::doDepthOutlierRemoval Count neighbors in window of ksize that are closer in terms of depth than depthThreshold value. If there are not enough neighbors (less than or equal to neighborsCount), remove the point. Please note that all the pixels lying on the edges of the image (i.e. pixels with x = 0, y = 0, x = width - 1, y = width - 1) are considered outliers and will be removed.
 * @param depthMatrix
 * @param ksize
 * @param neighborsCount
 * @param radius
 */
    MatrixType Processing::doDepthOutlierRemoval(const MatrixType &depthMatrix, int ksize, int neighborsCount, int depthThreshold)
    {
        if (ksize < 3 || ksize % 2 == 0)
        {
            throw std::invalid_argument("ksize should be an odd number that is greater or equal to 3");
        }
        int height = depthMatrix.shape()[0];
        int width = depthMatrix.shape()[1];
        MatrixType result(boost::extents[height][width]);

        int minBorder = - (ksize / 2);
        int maxBorder = ksize / 2;

        int outliersCount = 0;
        int distCount = 0;
        int dist = 0;

        for (int y = maxBorder; y < height - maxBorder; y++)
        {
            for (int x = maxBorder; x < width - maxBorder; x++)
            {
                DepthValueType depth = depthMatrix[y][x];
                int currentNeighbors = 0;

                for (int i = minBorder; i <= maxBorder; i++)
                {
                    for (int j = minBorder; j <= maxBorder; j++)
                    {
                        if (i == 0 && j == 0)
                        {
                            continue;
                        }

                        int distance = abs(depthMatrix[y+i][x+j] - depth);
                        if (distance <= depthThreshold)
                        {
                            currentNeighbors++;
                            dist += distance;
                            distCount++;
                        }
                    }
                }
                if (currentNeighbors >= neighborsCount)
                {
                    result[y][x] = depth;
                }
                else
                {
                    outliersCount++;
                }
            }
        }
        return result;
    }


    MatrixType Processing::doMedianFiltering(const MatrixType &depthMatrix, int ksize)
    {
        if (ksize < 3 || ksize % 2 == 0)
        {
            throw std::invalid_argument("ksize should be an odd number that is greater or equal to 3");
        }
        int height = depthMatrix.shape()[0];
        int width = depthMatrix.shape()[1];
        MatrixType result(boost::extents[height][width]);

        int minBorder = - (ksize / 2);
        int maxBorder = ksize / 2;

        int outliersCount = 0;
        int distCount = 0;
        int dist = 0;

        for (int y = maxBorder; y < height - maxBorder; y++)
        {
            for (int x = maxBorder; x < width - maxBorder; x++)
            {
                // TODO: initialize
                boost::container::vector<DepthValueType> neighborhoodDepths;
                neighborhoodDepths.push_back(depthMatrix[y][x]);

                for (int i = minBorder; i <= maxBorder; i++)
                {
                    for (int j = minBorder; j <= maxBorder; j++)
                    {
                        if (i == 0 && j == 0)
                        {
                            continue;
                        }

                        // Insert depth values in a way that we get a sorted array/list (like in insertion sort)
                        DepthValueType depth = depthMatrix[y+j][x+i];
                        bool isInserted = false;  // This variable is used when 'depth' variable has the biggest value
                        for (boost::container::vector<DepthValueType>::iterator it = neighborhoodDepths.begin(); it != neighborhoodDepths.end(); ++it)
                        {
                            if (depth >= *it)
                            {
                                neighborhoodDepths.insert(it, depth);
                                isInserted = true;
                                break;
                            }
                        }

                        if (!isInserted)
                        {
                            neighborhoodDepths.push_back(depth);
                        }
                    }
                }

                // TODO: compute median
                DepthValueType median = neighborhoodDepths.at(ksize * ksize / 2);
                // TODO: assign median value to (x,y) pixel
                result[y][x] = median;
            }
        }
        return result;
    }


    ListOfPointClouds Processing::traceBorders(const MatrixType &matrix, int distanceThreshold)
    {
        int height = matrix.shape()[0];
        int width = matrix.shape()[1];

        ListOfPointClouds allBorders;

        PairXYSetType markedPixels;

        for (int y = 0; y < height; y++)
        {
            for (int x = 1; x < width; x++)
            {
                // Continue we already have current pixel marked as a border
                PairXYType pointCurrent(x,y);
                if (markedPixels.find(pointCurrent) != markedPixels.end())
                {
                    continue;
                }
                // Continue if we do not find border candidate here
                if (abs(matrix[y][x-1] - matrix[y][x]) < distanceThreshold)
                {
                    continue;
                }

                CloudType border;

                // Border candidate is found at point (y, x), start border tracing
                this->traceBorderFromPoint(matrix, pointCurrent, true, markedPixels, border, height, width, distanceThreshold);

                this->traceBorderFromPoint(matrix, pointCurrent, false, markedPixels, border, height, width, distanceThreshold);

                if (border.empty() || border.size() < this->p.minBorderLength)
                {
                    continue;
                }
                allBorders.push_back(border);
            }
        }

        return allBorders;
    }


    void Processing::traceBorderFromPoint(const MatrixType &matrix, const PairXYType &pointCurrent, const bool isClockwise, PairXYSetType &markedPixels, CloudType &border, int height, int width, int distanceThreshold)
    {
        int *traceSequenceXArray, *traceSequenceYArray;
        if (isClockwise)
        {
            traceSequenceXArray = clockwiseTraceSequenceX;
            traceSequenceYArray = clockwiseTraceSequenceY;
        }
        else
        {
            traceSequenceXArray = counterclockwiseTraceSequenceX;
            traceSequenceYArray = counterclockwiseTraceSequenceY;
        }
        int x = pointCurrent.get<0>();
        int y = pointCurrent.get<1>();
        DepthValueType depthCurrent = matrix[y][x];

        int directionIndex = 0;
        while (true)
        {
            bool initialPointFound = false;
            bool imageLimitFound = false;
            bool newBorderPointFound = false;
            int xx, yy;

            int backtrackDirection = directionIndex;
            int maxTurns = 8; //TODO: I don't know whether this is correct (try to use 7)
            DepthValueType depthNeighbor;
            for (int i = directionIndex; i < directionIndex + maxTurns; i++)
            {
                int ii = i % 8;
                //            int yShift = traceSequence[ii][0];
                //            int xShift = traceSequence[ii][1];
                int yShift = traceSequenceYArray[ii];
                int xShift = traceSequenceXArray[ii];
                xx = x + xShift;
                yy = y + yShift;
                // Test if border touches limits of the image. Break in this case
                if (yy < 0 || yy >= height || xx < 0 || xx >= width)
                {
                    imageLimitFound = true;
                    break;
                }

                depthNeighbor = matrix[yy][xx];
                if (abs(depthNeighbor - depthCurrent) >= distanceThreshold)
                {
                    continue;
                }

                // New border point found. Break the loop
                newBorderPointFound = true;
                backtrackDirection = (ii + 6) % 8; // Store previous tracing direction
                break;
            }

            // Loop break conditions:
            //   a) found initial point;  (see below)
            //   b) going off the limits of the image;
            //   c) next border point is not found. (restrict algorithm to turn back 180 degrees, count number of consequent steps, it must be always less than 8 or 7!)
            if (imageLimitFound || !newBorderPointFound)
            {
                break;
            }

            // Backtrack
            directionIndex = backtrackDirection;

            PairXYType borderXY(xx, yy);

            if (pointCurrent == borderXY)
            {
                initialPointFound = true;
                break;  // new border point is the initial point
            }

            // Check if border point was already encountered
            if (markedPixels.find(borderXY) != markedPixels.end())
            {
                break;
            }

            PointType borderPointXYZ(xx, yy, depthNeighbor);
            border.push_back(borderPointXYZ);
            markedPixels.insert(borderXY);
            // Update coordinates of current point
            x = xx;
            y = yy;
            // Keep track of current depth's value
            depthCurrent = matrix[yy][xx];
        }
    }


    MatrixType Processing::cvMatToMatrix(const cv::Mat &depthImage)
    {
        // TODO: return pointer to the array instead of array
        cv::Size size = depthImage.size();
        int height = size.height;
        int width = size.width;

        MatrixType result(boost::extents[size.height][size.width]);

        cv::MatConstIterator_<DepthValueType> it, end;
        int x = 0;
        int y = 0;
        for (it = depthImage.begin<DepthValueType>(), end = depthImage.end<DepthValueType>(); it!= end; ++it)
        {
            // Get the pixel value
            DepthValueType value = *it;

            // Store pixel value in the matrix
            result[y][x] = value;

            // Modify coordinates
            x++;
            if (x >= width)
            {
                x = 0;
                y++;
            }
        }

        return result;
    }


    cv::Mat Processing::drawBorders(const cv::Mat &depthImage, ListOfPointClouds &borders)
    {
        cv::Size size = depthImage.size();
        cv::Mat result = cv::Mat::zeros(size.height, size.width, CV_8UC3);
        BOOST_FOREACH(CloudType border, borders)
                    {
                        cv::Vec3b color((uchar)this->rng(128) + 128,
                                        (uchar)this->rng(128) + 128,
                                        (uchar)this->rng(128) + 128);
                        BOOST_FOREACH(PointType point, border)
                                    {
                                        int y = point.y;
                                        int x = point.x;
                                        result.at<cv::Vec3b>(cv::Point(x, y)) = color;
                                    }
                    }

        return result;
    }


    CloudTypeRGB Processing::borderToPointCloud(const CloudType &border, const cv::Vec3b &color)
    {
        CloudTypeRGB cloud;
        BOOST_FOREACH(PointType point, border)
                    {
                        PointType point3d = this->depthToPointXYZ(point);
                        PointTypeRGB point3dcolor(color[0], color[1], color[2]);
                        point3dcolor.x = point3d.x;
                        point3dcolor.y = point3d.y;
                        point3dcolor.z = point3d.z;
                        cloud.push_back(point3dcolor);
                    }

        return cloud;
    }


    ListOfPointCloudsRGB Processing::bordersToPointClouds(const ListOfPointClouds &borders)
    {
        ListOfPointCloudsRGB pointClouds;
        BOOST_FOREACH(CloudType border, borders)
                    {
                        //TODO: create random color
                        // TODO: convert border to point cloud
                        cv::Vec3b color((uchar)this->rng(128) + 128,
                                        (uchar)this->rng(128) + 128,
                                        (uchar)this->rng(128) + 128);
                        CloudTypeRGB pointCloud = this->borderToPointCloud(border, color);
                        pointClouds.push_back(pointCloud);
                    }

        return pointClouds;
    }


    CloudTypeRGB Processing::reduceBorderClouds(const ListOfPointCloudsRGB &borderClouds)
    {
        CloudTypeRGB cloud;
        BOOST_FOREACH(CloudTypeRGB border, borderClouds)
                    {
                        cloud += border;
                    }

        return cloud;
    }


    CloudTypeRGB Processing::findCircles(const CloudTypeRGB &border)
    {
        // TODO: verify that border points belong to a circle

        // TODO: compute center of the circle and build a normal (it should have direction to the camera)

        // TODO: ??? Do I need to check if points belong to the same plane??

        unsigned long point_count = border.size();
        if (point_count <= 1)
        {
            throw std::invalid_argument("border cannot contain 0 or 1 points");
        }

        double mju_x = 0, mju_y = 0, mju_z = 0;
        BOOST_FOREACH(PointTypeRGB point, border)
                    {
                        mju_x += point.x;
                        mju_y += point.y;
                        mju_z += point.z;
                    }

        double k = 1. / point_count;
        mju_x *= k;
        mju_y *= k;
        mju_z *= k;

        std::vector<double> r_computed(point_count);
        int index = 0;
        BOOST_FOREACH(PointTypeRGB point, border)
                    {
                        double xx = point.x - mju_x;
                        xx *= xx;
                        double yy = point.y - mju_y;
                        yy *= yy;
                        double zz = point.z - mju_z;
                        zz *= zz;
                        double r = sqrt(xx + yy + zz);
                        r_computed[index] = r;
                        index++;
                    }

        // Compute mean and variance for r_computed
        double mju_r = 0;
        BOOST_FOREACH(double r, r_computed)
                    {
                        mju_r += r;
                    }

        mju_r *= k;

        // TODO: check if radius is not too small!!!!!!! (should be at least 2cm)

        double var_r = 0;
        BOOST_FOREACH(double r, r_computed)
                    {
                        double a = r - mju_r;
                        a *= a;
                        var_r += a;
                    }

        var_r = sqrt(var_r) / (point_count - 1);

        double threshold = 1e-3;
        // Border is not circular, thus return empty cloud
        if (var_r > threshold || mju_r < 1e-6)
        {
            // TODO: I need to return some empty object
            CloudTypeRGB emptyCloud;
            return emptyCloud;
        }

        ROS_INFO("mju_point = (%e, %e, %e); mju_r = %e; var_r = %e; point_count = %d", mju_x, mju_y, mju_z, mju_r, var_r, point_count);

        double min_radius = 1e-2;
        if (fabs(mju_r) < min_radius)
        {
            ROS_INFO("radius is too small: %e, %e < %e", mju_r, min_radius);
            CloudTypeRGB emptyCloud;
            return emptyCloud;
        }

        ROS_INFO("added");

        // TODO: add center point and normal to the point cloud
        return border;
    }


    CloudTypeRGB Processing::findCirclesEigen(const CloudTypeRGB &border)
    {
        // TODO: verify that border points belong to a circle

        // TODO: compute center of the circle and build a normal (it should have direction to the camera)

        // TODO: ??? Do I need to check if points belong to the same plane??

        unsigned long point_count = border.size();
        if (point_count <= 1)
        {
            throw std::invalid_argument("border cannot contain 0 or 1 points");
        }

        double mju_x = 0, mju_y = 0, mju_z = 0;
        BOOST_FOREACH(PointTypeRGB point, border)
                    {
                        mju_x += point.x;
                        mju_y += point.y;
                        mju_z += point.z;
                    }

        double k = 1. / point_count;
        mju_x *= k;
        mju_y *= k;
        mju_z *= k;

        boost::shared_ptr<CloudTypeRGB> pointsShifted = boost::shared_ptr<CloudTypeRGB>(new CloudTypeRGB);
        BOOST_FOREACH(PointTypeRGB point, border)
                    {
                        PointTypeRGB pointShifted(point.r, point.g, point.b);
                        pointShifted.x = point.x - mju_x;
                        pointShifted.y = point.y - mju_y;
                        pointShifted.z = point.z - mju_z;
                        pointsShifted->push_back(pointShifted);
                    }

        //    Eigen::Matrix3f& pcl::PCA<PointTypeRGB> eigenVectors;
        //    eigenVectors = pcl::PCA<PointTypeRGB>::getEigenVectors();
        pcl::PCA<PointTypeRGB> pca;
        pca.setInputCloud(pointsShifted);
        //    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();  // Probably I don't need this for the test
        Eigen::Vector3f eigenValues = pca.getEigenValues();
        int minEigenValueIndex;
        float minEigenValue = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; i++)
        {
            if (eigenValues(i) < minEigenValue)
            {
                minEigenValue = eigenValues(i);
                minEigenValueIndex = i;
            }
        }
        int indexA, indexB;
        switch (minEigenValueIndex) {
            case 0:
                indexA = 1;
                indexB = 2;
                break;
            case 1:
                indexA = 0;
                indexB = 2;
                break;
            case 2:
                indexA = 0;
                indexB = 1;
        }

        float minEigenValueThreshold = 1e-2;
        float maxEigenValueThreshold = 1e-5;
        CloudTypeRGB emptyCloud;
        if (eigenValues(minEigenValueIndex) > minEigenValueThreshold)
        {
            return emptyCloud;
        }

        if ((eigenValues(indexA) < maxEigenValueThreshold) || (eigenValues(indexB) < maxEigenValueThreshold))
        {
            return emptyCloud;
        }

        float ratio = eigenValues(indexA) / eigenValues(indexB);
        if (ratio < 0.75 || ratio > 1.25)
        {
            return emptyCloud;
        }

        ROS_INFO("min = %f A = %f B = %f, ratio = %f", eigenValues(minEigenValueIndex), eigenValues(indexA), eigenValues(indexB), ratio);

        // TODO: one eigenvalue has to be small, two others - close values (ratio almost equal to 1)


        //    // TODO: add center point and normal to the point cloud
        return border;
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

















