// Allrights reserved.
// Author: github.com/izhengfan (ZHENG, Fan)
/// \brief Subscribe pose topic published by ROVIO, and save to a log file.

#include <iostream>
#include <atomic>
#include <string>
#include <fstream>
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Geometry>
using namespace std;

class PoseViewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    constexpr static const double imageSize = 500.0;
    ofstream logfile;
    PoseViewer()
    {
        cv::namedWindow("ROVIO Top View");
        _image.create(imageSize, imageSize, CV_8UC3);
        drawing_ = false;
        showing_ = false;
    }
    ~PoseViewer()
    {
        if(logfile.is_open()) logfile.close();
    }

    // this we can register as a callback
    void publishFullStateAsCallback( double time_, const Eigen::Matrix4d & T_WS)
    {

        // store the time stamps
        double exact_time = time_;
        _times.push_back(exact_time);

        Eigen::Vector3d r = T_WS.block<3,1>(0,3);
        Eigen::Matrix3d C = T_WS.block<3,3>(0,0);

        // write to the log file
        logfile << std::fixed << std::setprecision(8);
        logfile << std::setw(12) << exact_time << " ";
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                logfile << std::setw(12) << C(i,j) << " ";
            }
            logfile << std::setw(12) << r(i) << " ";
        }
        logfile << std::endl;

        _path.push_back(cv::Point2d(r[0], r[1]));
        _heights.push_back(r[2]);
        // maintain scaling
        if (r[0] - _frameScale < _min_x)
            _min_x = r[0] - _frameScale;
        if (r[1] - _frameScale < _min_y)
            _min_y = r[1] - _frameScale;
        if (r[2] < _min_z)
            _min_z = r[2];
        if (r[0] + _frameScale > _max_x)
            _max_x = r[0] + _frameScale;
        if (r[1] + _frameScale > _max_y)
            _max_y = r[1] + _frameScale;
        if (r[2] > _max_z)
            _max_z = r[2];
        _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));

        // draw it
        while (showing_) {
        }
        drawing_ = true;
        // erase
        _image.setTo(cv::Scalar(250, 250, 250));
        drawPath();
        // draw axes
        Eigen::Vector3d e_x = C.col(0);
        Eigen::Vector3d e_y = C.col(1);
        Eigen::Vector3d e_z = C.col(2);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_x[0], e_x[1]) * _frameScale),
                cv::Scalar(0, 0, 255), 1, CV_AA);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_y[0], e_y[1]) * _frameScale),
                cv::Scalar(0, 255, 0), 1, CV_AA);
        cv::line(
                    _image,
                    convertToImageCoordinates(_path.back()),
                    convertToImageCoordinates(
                        _path.back() + cv::Point2d(e_z[0], e_z[1]) * _frameScale),
                cv::Scalar(255, 0, 0), 1, CV_AA);

        // some text:
        std::stringstream postext;
        postext << "position = [" << r[0] << ", " << r[1] << ", " << r[2] << "]";
        cv::putText(_image, postext.str(), cv::Point(15,15),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(5,5,5), 1);

        drawing_ = false; // notify
    }
    void display()
    {
        while (drawing_) {
        }
        showing_ = true;
        cv::imshow("ROVIO Top View", _image);
        showing_ = false;
        cv::waitKey(1);
    }
private:
    cv::Point2d convertToImageCoordinates(const cv::Point2d & pointInMeters) const
    {
        cv::Point2d pt = (pointInMeters - cv::Point2d(_min_x, _min_y)) * _scale;
        return cv::Point2d(pt.x, imageSize - pt.y); // reverse y for more intuitive top-down plot
    }
    void drawPath()
    {
        for (size_t i = 0; i + 1 < _path.size(); ) {
            cv::Point2d p0 = convertToImageCoordinates(_path[i]);
            cv::Point2d p1 = convertToImageCoordinates(_path[i + 1]);
            cv::Point2d diff = p1-p0;
            if(diff.dot(diff)<2.0){
                _path.erase(_path.begin() + i + 1);  // clean short segment
                _heights.erase(_heights.begin() + i + 1);
                continue;
            }
            double rel_height = (_heights[i] - _min_z + _heights[i + 1] - _min_z)
                    * 0.5 / (_max_z - _min_z);
            cv::line(
                        _image,
                        p0,
                        p1,
                        rel_height * cv::Scalar(255, 0, 0)
                        + (1.0 - rel_height) * cv::Scalar(0, 0, 255),
                        1, CV_AA);
            i++;
        }
    }
    cv::Mat _image;
    std::vector<cv::Point2d> _path;
    std::vector<double> _heights;
    std::vector<double> _times;
    double _scale = 1.0;
    double _min_x = -0.5;
    double _min_y = -0.5;
    double _min_z = -0.5;
    double _max_x = 0.5;
    double _max_y = 0.5;
    double _max_z = 0.5;
    const double _frameScale = 0.2;  // [m]
    std::atomic_bool drawing_;
    std::atomic_bool showing_;
};

PoseViewer g_viewer;

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    double time = msg->header.stamp.toSec();
    Eigen::Matrix4d pose;
    Eigen::Quaterniond eig_quat;
    eig_quat.x() = msg->pose.pose.orientation.x;
    eig_quat.y() = msg->pose.pose.orientation.y;
    eig_quat.z() = msg->pose.pose.orientation.z;
    eig_quat.w() = msg->pose.pose.orientation.w;
    pose.block<3,3>(0,0) = eig_quat.toRotationMatrix();
    pose(0,3) = msg->pose.pose.position.x;
    pose(1,3) = msg->pose.pose.position.y;
    pose(2,3) = msg->pose.pose.position.z;
    g_viewer.publishFullStateAsCallback(time, pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rovio_pose_sub");
    ros::NodeHandle nh;

    string logfile_name = argv[1];
    g_viewer.logfile.open(logfile_name);

    ros::Subscriber pose_sub = nh.subscribe("rovio/pose_with_covariance_stamped", 10, poseCallback);
    while(ros::ok())
    {
        g_viewer.display();
        ros::spinOnce();
        char key = cv::waitKey(5);
        if(key == 27) break; // Esc to exit
    }
    return 0;
}
