// Allrights reserved.
// Author: github.com/izhengfan (ZHENG, Fan)
/// \brief Image publishing demo
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
    vector<string> image_names;
    int num_camera_images = 0;
    std::string path(argv[1]);
    string folder = path;
    for (auto it = boost::filesystem::directory_iterator(folder);
         it != boost::filesystem::directory_iterator(); it++)
    {
        if (!boost::filesystem::is_directory(it->path())) //we eliminate directories
        {
            std::string filename = it->path().filename().string();
            if(boost::filesystem::extension(filename) == ".png")
            {
                num_camera_images++;
                image_names.push_back(filename);
            }
        } else
            continue;
    }
    std::sort(image_names.begin(), image_names.end());
    ros::Rate loop(20);
    for(unsigned i=0; i<image_names.size(); i++)
    {
        string image_name = image_names[i];
        cv::Mat image = cv::imread(folder + image_name, cv::IMREAD_GRAYSCALE);
        // pub image
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
        pub.publish(msg);
        if(!ros::ok()) break;
        ros::spinOnce();
        loop.sleep();

    }
    return 0;
}

