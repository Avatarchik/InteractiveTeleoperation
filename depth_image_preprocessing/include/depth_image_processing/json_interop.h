#ifndef JSON_INTEROP_H
#define JSON_INTEROP_H

#include "utilities.h"

// RapidJSON
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

// BOOST
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>

// STD
#include <string>
#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>


// TEST
// base64 encoded = 6hNOP2haPj/qXR8/EKLiPXTgnz4=
// array =  [0.8049913644790649, 0.7435669898986816, 0.6225267648696899, 0.11066067218780518, 0.312259316444397]



namespace Biolab
{

    class JSONInterop
    {
        // TODO: this class must be a singleton!! (or static or something else)
    private:
//    std::string stringCompressEncode(const std::string &data);

//    std::string base64_encode(const char *bytes_to_encode, size_t len);
//    std::string base64_decode(std::string const& s);

//    std::string base64Encode(const char *buffer, size_t len);
        std::string ToBase64(std::vector<unsigned char> buffer);
        std::vector<unsigned char> PointCloudToString(const CloudTypeRGB &cloud);


    public:
        JSONInterop();
        ~JSONInterop();

        std::string PointCloudToJson(const CloudTypeRGB &cloud);

    };

}

#endif /* JSON_INTEROP_H */
