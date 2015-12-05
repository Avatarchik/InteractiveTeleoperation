#include "../../include/depth_image_processing/json_interop.h"

namespace Biolab {

    JSONInterop::JSONInterop() {
    }

    JSONInterop::~JSONInterop() {
    }

    std::string JSONInterop::PointCloudToJson(const CloudTypeRGB &cloud) {
// JSON data format example
//    {
//        coordinates: {
//            prefix: 'THREE.Vector3',
//            data: [-10, 10, 0, -10, -10, 0, 10, -10, 0]
//        },
//        colors: {
//            prefix: 'THREE.Color',
//            data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
//        }
//    }

// compress and base64 encode coordinates
        std::vector<unsigned char> buffer;
        buffer = PointCloudToString(cloud);
        std::string b64 = ToBase64(buffer);
        ROS_INFO("b64 length = %d", (int) b64.length());
        ROS_INFO("aaaaaaa short length %d", (int) sizeof(short));

// TODO: move constant JSON data to constructor
        using namespace rapidjson;
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        writer.StartObject();
        writer.Key("coordinates");
        writer.StartObject();
        writer.Key("prefix");
        writer.String("THREE.Vector3");
        writer.Key("stride");
        writer.Int(3);
        writer.Key("array_item_size");
        writer.Int((int) sizeof(short));  // TODO: here can be `float`
        writer.Key("data");
        writer.String(b64.c_str());  // Put coordinates here
        writer.EndObject();
        writer.EndObject();

        return s.GetString();
    }

    std::vector<unsigned char> JSONInterop::PointCloudToString(const CloudTypeRGB &cloud) {
//        size_t pointsCount = cloud.size();
//        float plainCoordinates[3 * pointsCount];
//        size_t counter = 0;
//        BOOST_FOREACH(PointTypeRGB point, cloud)
//                    {
//                        plainCoordinates[counter++] = point.x;
//                        plainCoordinates[counter++] = point.y;
//                        plainCoordinates[counter++] = point.z;
//                    }
//
//        std::vector<unsigned char> buffer;
//        buffer.resize(sizeof(plainCoordinates));
//        memcpy(&buffer[0], plainCoordinates, sizeof(plainCoordinates));
//        return buffer;
        size_t pointsCount = cloud.size();
        short plainCoordinates[3 * pointsCount];  // TODO: or float here
        size_t counter = 0;
        BOOST_FOREACH(PointTypeRGB point, cloud) {
                        plainCoordinates[counter++] = (short) (1000 * point.x);
                        plainCoordinates[counter++] = (short) (1000 * point.y);
                        plainCoordinates[counter++] = (short) (1000 * point.z);
                    }

        // Create buffer
        std::vector<unsigned char> buffer;
        // Allocate memory for buffer
        buffer.resize(sizeof(plainCoordinates));
        memcpy(&buffer[0], plainCoordinates, sizeof(plainCoordinates));
        return buffer;
    }


//std::string JSONInterop::stringCompressEncode(const std::string &data)
//{
//    // ZLib and GZip compression show similar results (compress up to 37%). BZip2 is not supported by Pako (javascript library that implements decompression)
//    // Compression mode is set to default_compression, because compression rate is 38% vs 37% for best_compression, though a little bit faster

//    // http://stackoverflow.com/questions/27529570/simple-zlib-c-string-compression-and-decompression
//    std::stringstream compressed;
//    std::stringstream original;
//    original << data;
//    boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
////    out.push(boost::iostreams::zlib_compressor());
//    boost::iostreams::basic_zlib_compressor<std::allocator<char> > compressor(boost::iostreams::zlib::default_compression);

//    out.push(compressor);
//    out.push(original);
//    boost::iostreams::copy(out, compressed);

//    std::string compressedStr = compressed.str();
//    std::string compressed_encoded = base64_encode(reinterpret_cast<const unsigned char*>(compressedStr.c_str()), compressedStr.length());

//    return compressed_encoded;
//}




/**
*  This function is from http://www.codeproject.com/Articles/359160/Base-Encoder-and-Boost
*/
    std::string JSONInterop::ToBase64(std::vector<unsigned char> buffer) {
        using namespace boost::archive::iterators;

        // Pad with 0 until a multiple of 3
        unsigned int paddedCharacters = 0;

        while (buffer.size() % 3 != 0) {
            paddedCharacters++;
            buffer.push_back(0x00);
        }

        // Crazy typedef black magic
        typedef base64_from_binary<    // convert binary values to base64 characters
                transform_width<   // retrieve 6 bit integers from a sequence of 8 bit bytes
                        const unsigned char *, 6, 8> > base64Iterator; // compose all the above operations in to a new iterator

        // Encode the buffer and create a string
        std::string encodedString(
                base64Iterator(buffer.begin().base()),
                base64Iterator(buffer.begin().base() + (buffer.size() - paddedCharacters)));
        // Add '=' for each padded character used
        for (unsigned int i = 0; i < paddedCharacters; i++) {
            encodedString.push_back('=');
        }

        return encodedString;
    }

}
