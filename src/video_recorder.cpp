/****************************************************************************
* Software License Agreement (Apache License)
*
*     Copyright (C) 2012-2013 Open Source Robotics Foundation
*
*     Licensed under the Apache License, Version 2.0 (the "License");
*     you may not use this file except in compliance with the License.
*     You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
*     Unless required by applicable law or agreed to in writing, software
*     distributed under the License is distributed on an "AS IS" BASIS,
*     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*     See the License for the specific language governing permissions and
*     limitations under the License.
*
*****************************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Bool.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <rosbag/bag.h>
#include "draco_point_cloud_transport/CompressedPointCloud2.h"

// #include <image_transport/image_transport.h>
// #include <camera_calibration_parsers/parse.h>
// #if CV_MAJOR_VERSION == 3
// #include <opencv2/videoio.hpp>
// #endif

cv::VideoWriter outputVideo;
std::ofstream outFile;

int g_count = 0;
ros::Time g_last_wrote_time = ros::Time(0);
std::string encoding;
std::string codec;
double fps;
std::string filename, data_root_dir_path, sub_dir_path;;
std::string txt_path, bag_path, imu_bag_path;
double min_depth_range;
double max_depth_range;
bool use_dynamic_range;
int colormap;
int total_msgs_number, current_msgs_number;
bool lidar_bag_opened = false;
bool imu_bag_opened = false;
rosbag::Bag point_cloud_comp_bag, imu_bag;

bool mkdir_if_not_exist(const std::string& dir_path){
    struct stat info;
    if(stat(dir_path.c_str(), &info)!=0){
        if(mkdir(dir_path.c_str(), 0777)==-1){
            std::cout<<"failed mkdir dir: "<<dir_path<<std::endl;
            return false;
        }
    }
    return true;
}

void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
    if (!outputVideo.isOpened()) {

        std::stringstream ss;
        ss << ros::Time::now().toNSec();
        sub_dir_path = data_root_dir_path + "/" + ss.str();
        mkdir_if_not_exist(sub_dir_path);

        filename = sub_dir_path + "/" + ss.str() + ".avi";
        txt_path = filename + ".txt";
        bag_path = sub_dir_path + "/" + ss.str() + "_lidar_draco.bag";
        imu_bag_path = sub_dir_path + "/" + ss.str() + "_imu.bag";
        ROS_INFO("Video recording to %s", filename.c_str());

        {
            try {
                imu_bag.open(imu_bag_path, rosbag::bagmode::Write);
            }
            catch (rosbag::BagException e) {
                ROS_ERROR("Error writing: %s", e.what());
                ros::shutdown();
            }
            ROS_INFO("Recording to %s.", imu_bag_path.c_str());
            imu_bag_opened = true;
        }

        {
            try {
                point_cloud_comp_bag.open(bag_path, rosbag::bagmode::Write);
            }
            catch (rosbag::BagException e) {
                ROS_ERROR("Error writing: %s", e.what());
                ros::shutdown();
            }
            ROS_INFO("Recording to %s.", bag_path.c_str());
            lidar_bag_opened = true;
        }


        cv::Size size(image_msg->width, image_msg->height);
//        total_msgs_number = fps * 60 * 5; // 5 minute msgs
        total_msgs_number = fps * 60  * 5; // 5 minute msgs
        current_msgs_number = 1;

        outputVideo.open(filename,
                cv::VideoWriter::fourcc(codec.c_str()[0],
                          codec.c_str()[1],
                          codec.c_str()[2],
                          codec.c_str()[3]), 
                fps,
                size,
                true);

        if (!outputVideo.isOpened())
        {
            ROS_ERROR("Could not create the output video! Check filename and/or support for codec.");
            exit(-1);
        }
        outFile.open(txt_path);
        outFile << std::fixed;
        ROS_INFO_STREAM("Starting to record " << codec << " video at " << size << "@" << fps << "fps. Press Ctrl+C to stop recording." );

    }
    try {
        cv_bridge::CvtColorForDisplayOptions options;
        options.do_dynamic_scaling = false;
        options.min_image_value = 0;
        options.max_image_value = 0;
        options.colormap = colormap;
        const cv::Mat image = cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(image_msg), encoding,
                                                            options)->image;
        if (!image.empty()) {
            outputVideo << image;
            double timestamp = image_msg->header.stamp.toSec();
            outFile << timestamp << "\n";
            if(current_msgs_number>=total_msgs_number){
                outFile.close();
                outputVideo.release();

                point_cloud_comp_bag.close();
                lidar_bag_opened = false;

                imu_bag.close();
                imu_bag_opened = false;

            }
            current_msgs_number++;
        } else {
            ROS_WARN("Frame skipped, no data!");
        }
    } catch (cv_bridge::Exception) {
        ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
        return;
    }
}

void poinc_cloud_callback(const draco_point_cloud_transport::CompressedPointCloud2Ptr& point_cloud_comp_msg){
    if(lidar_bag_opened){
        point_cloud_comp_bag.write("/point_cloud_compress/draco", point_cloud_comp_msg->header.stamp, point_cloud_comp_msg);
    }
    return;

}

void imu_callback(const sensor_msgs::ImuPtr& imu_msg){
    if(imu_bag_opened){
        imu_bag.write("/imu/data", imu_msg->header.stamp, imu_msg);
    }
    return;

}

int main(int argc, char** argv)
{
    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    ros::init(argc, argv, "video_recorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    // local_nh.param("filename", filename, std::string("output.avi"));
    // bool stamped_filename;
    // local_nh.param("stamped_filename", stamped_filename, false);
    local_nh.param("fps", fps, 15.0);
    std::string topic;
    local_nh.param("topic", topic, std::string("/rgb/image_raw"));
    local_nh.param("path", data_root_dir_path, std::string("./output"));
    codec = "MJPG";
    // local_nh.param("codec", codec, std::string("MJPG"));
    // local_nh.param("encoding", encoding, std::string("bgr8"));
    encoding = "bgr8";
    // cv_bridge::CvtColorForDisplayOptions
    // local_nh.param("min_depth_range", min_depth_range, 0.0);
    // local_nh.param("max_depth_range", max_depth_range, 0.0);
    // local_nh.param("use_dynamic_depth_range", use_dynamic_range, false);
    // local_nh.param("colormap", colormap, -1);

    mkdir_if_not_exist(data_root_dir_path);



//    std::string sub_dir_path;
//    std::stringstream ss;
//    ss << ros::Time::now().toNSec();
//    sub_dir_path = data_root_dir_path + "/" + ss.str();
//    mkdir_if_not_exist(sub_dir_path);
//
//    filename = sub_dir_path + "/" + ss.str() + ".avi";
//    txt_path = filename + ".txt";
//    ROS_INFO("Video recording to %s", filename.c_str());

    if (codec.size() != 4) {
        ROS_ERROR("The video codec must be a FOURCC identifier (4 chars)");
        exit(-1);
    }

    ros::Subscriber sub_image = nh.subscribe(topic, 1000, callback);
    ros::Subscriber sub_point_cloud_comp = nh.subscribe("/point_cloud_compress/draco", 1000, poinc_cloud_callback);
    ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1000, imu_callback);

    ROS_INFO_STREAM("Waiting for topic " << topic << "...");
    ros::spin();
    std::cout << "\nVideo saved as " << filename << std::endl;
    outFile.close();
}
