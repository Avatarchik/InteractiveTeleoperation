//
// Created by naveed on 15. 4. 30.
//

#ifndef UTILITIES_H
#define UTILITIES_H

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>

// Boost
#include "boost/multi_array.hpp"
#include "boost/tuple/tuple.hpp"
#include "boost/tuple/tuple_comparison.hpp"
#include "boost/container/set.hpp"
#include "boost/foreach.hpp"
#include "boost/container/list.hpp"

namespace Biolab
{
// Define shorter names for PCL data types
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
    typedef pcl::PointXYZRGB PointTypeRGB;
    typedef pcl::PointCloud<pcl::PointXYZRGB> CloudTypeRGB;

// Define shorter names for algorithm-specific data types
    typedef short int DepthValueType;
    typedef boost::tuple<int, int> PairXYType;
    typedef boost::container::set<PairXYType> PairXYSetType;
    typedef boost::container::list<CloudType> ListOfPointClouds;
    typedef boost::container::list<CloudTypeRGB> ListOfPointCloudsRGB;
    typedef boost::multi_array<DepthValueType, 2> MatrixType;


    struct Parameters
    {
        Parameters();

        bool buildFullCloud;  // This flag determines whether to pull all the points in the cloud with selected borders highlighted, or not


    };
}

#endif /* UTILITIES_H */