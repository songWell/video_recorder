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
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
// #include <camera_calibration_parsers/parse.h>
// #if CV_MAJOR_VERSION == 3
// #include <opencv2/videoio.hpp>
// #endif


int main(int argc, char** argv)
{
    cv::VideoCapture cap;
    std::string path, frame_id, txt_path, topic;
    

    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    ros::init(argc, argv, "video_decorder", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    nh.param("path", path, std::string("output.avi"));
    nh.param("txt_path", txt_path, std::string("output.avi.txt"));
    nh.param("frame_id", frame_id, std::string("rgb_camera_link"));
    nh.param("topic", topic, std::string("/rgb/image_raw"));

//    std::string topic = "/rgb/image_raw";
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_image = it.advertise(topic, 100000);

    cap.open(path);
    if(!cap.isOpened()){
        std::cout<<"read video failed!!! "<<std::endl;

    }
    std::ifstream in_file(txt_path, std::ios::in);
    if(!in_file){
        std::cout<<"read time stamp txt failed!!!"<<std::endl;
    }

    ros::Rate loop_rate(15);
    int count = 1;
//    ros::Duration(1).sleep();

    while(ros::ok()){
        auto num_subscriber = pub_image.getNumSubscribers();
        if(num_subscriber>0)
            break;
    }
    while(ros::ok()){
        cv::Mat frame;
        if(!cap.read(frame)){
            cap.release();
            in_file.close();
            std::cout<<"no video frame!!! "<<std::endl;
            break;
        }
        // cap.get(cv::CAP_PROP_POS_MSEC, float(timestamp * 1000));
        std::string line_str;
        std::getline(in_file, line_str);
        double time_stamp = std::atof(line_str.c_str());

        sensor_msgs::ImagePtr out_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        out_img->header.frame_id = frame_id;
        out_img->header.stamp = ros::Time().fromSec(time_stamp);

        pub_image.publish(out_img);
        loop_rate.sleep();
        ++count;

    }

    ros::spin();
}
