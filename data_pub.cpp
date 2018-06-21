// Allrights reserved.
// Author: github.com/izhengfan (ZHENG, Fan)
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

std::vector<std::string> split(const std::string &s, const char d)
{
    std::vector<std::string> v;
    char *str = new char[s.size()+1];
    strcpy(str, s.c_str());
    while (char *t = strsep(&str, &d))
        if(t[0] != '\0') v.push_back(t);
    delete[] str;
    return v;
}

sensor_msgs::Imu parseImuMsg(const vector<double> & data)
{
    sensor_msgs::Imu msg;
    msg.header.stamp.fromSec(data[0]);
    msg.linear_acceleration.x = data[1];
    msg.linear_acceleration.y = data[2];
    msg.linear_acceleration.z = data[3];
    msg.angular_velocity.x = data[4];
    msg.angular_velocity.y = data[5];
    msg.angular_velocity.z = data[6];
    return msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/cam0/image_raw", 1);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu0", 1);

    std::string path(argv[1]);
    std::string cam_time_line;
    std::ifstream cam_time_file(path + "/timestamps_cameras.txt");
    std::vector<double> cam_times(0);
    while(std::getline(cam_time_file, cam_time_line))
    {
        cam_times.push_back(atof(cam_time_line.c_str()));
    }
    vector<string> image_names;
    int num_camera_images = 0;
    string folder = path +"/left_cam_frames/";
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
    if(num_camera_images == 0)
    {
        cout << "No images at " << folder << endl;
        exit(-1);
    }

    cout << "Cam " << "images: " << num_camera_images << endl;
    std::sort(image_names.begin(), image_names.end());

    // open the IMU file
    string line_imu;
    std::ifstream imu_file(path + "/imu.txt");
    vector<vector<double>> imu_data;
    while(std::getline(imu_file, line_imu))
    {
        vector<double> data_line(7);
        std::vector<std::string> elem = split(line_imu, ' ');
        for(int i = 0; i < 7; i++)
        {
            data_line[i] = atof(elem[i].c_str());
        }
        imu_data.push_back(data_line);
    }
    imu_file.close();
    cout << "Imu count: " << imu_data.size() << endl;

    int count = 0;
    auto imu_itr = imu_data.begin();

    ros::Rate loop(20);

    for(unsigned i = 0; i < image_names.size(); i++)
    {
        string image_name = image_names[i];
        cv::Mat image = cv::imread(folder + image_name, cv::IMREAD_GRAYSCALE);
        double cam_time = cam_times[i];

        vector<double> line = *imu_itr;
        ros::Rate imu_loop(200);
        while(imu_itr != imu_data.end() && line[0] <= cam_time)
        {
            // pub imu
            sensor_msgs::Imu imu = parseImuMsg(line);
            imu_pub.publish(imu);
            imu_itr++;
            line = *imu_itr;
            imu_loop.sleep();
        }

        // pub image
        std_msgs::Header header;
        header.stamp.fromSec(cam_time);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
        pub.publish(msg);

        if(imu_itr == imu_data.end())
            break;
        else
            imu_itr++;

        if(!ros::ok()) break;

        ros::spinOnce();
        loop.sleep();

    }
    while(imu_itr != imu_data.end())
    {
        // pub imu
        imu_pub.publish(parseImuMsg(*imu_itr));
        imu_itr++;
    }

    return 0;

}
