// (MIT License)
//
// Copyright 2019 David B. Curtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// Subscribes to an image stream of side-by-side stereo where each image
// message consists of a left and right image concatenated to form a single
// double-wide image.  This node splits the incoming image down the middle
// and republishes each half as stereo/left and stereo/right images.
//
// This is a modified version of public domain code posted by PeteBlackerThe3rd
// in response to my question on ROS Answers:
// https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/
//
// -- dbc

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


// If non-zero, outputWidth and outputHeight set the size of the output images.
// If zero, the outputWidth is set to 1/2 the width of the input image, and
// outputHeight is the same as the height of the input image.
int OUTPUT_WIDTH, OUTPUT_HEIGHT;
std::string LEFT_CAMERA_INFO_URL, RIGHT_CAMERA_INFO_URL;
std::string LEFT_FRAME, RIGHT_FRAME;

// Input image subscriber.
ros::Subscriber IMAGE_SUB;

// Left and right image publishers.
image_transport::Publisher LEFT_IMAGE_PUBLISHER, RIGHT_IMAGE_PUBLISHER;

// Left and right camera info publishers and messages.
ros::Publisher LEFT_CAMERA_INFO_PUBLISHER, RIGHT_CAMERA_INFO_PUBLISHER;
sensor_msgs::CameraInfo LEFT_CAMERA_INFO_MSG, RIGHT_CAMERA_INFO_MSG;

// Image capture callback.
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Get double camera image.
    cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat image = cv_img->image;

    // If there are any subscribers to either output topic then publish images
    // on them.
    if (LEFT_IMAGE_PUBLISHER.getNumSubscribers() > 0 || RIGHT_IMAGE_PUBLISHER.getNumSubscribers() > 0)
    {
        // Define the relevant rectangles to crop.
        cv::Rect left_roi, right_roi;
        left_roi.y = right_roi.y = 0;
        left_roi.width = right_roi.width = image.cols / 2;
        left_roi.height = right_roi.height = image.rows;
        left_roi.x = 0;
        right_roi.x = image.cols / 2;

        // Crop images.
        cv::Mat left_image = cv::Mat(image, left_roi);
        cv::Mat right_image = cv::Mat(image, right_roi);

        // Apply scaling, if specified.
        bool use_scaled;
        cv::Mat left_scaled, right_scaled;
        if (use_scaled = (OUTPUT_WIDTH > 0 && OUTPUT_HEIGHT > 0))
        {
          cv::Size sz = cv::Size(OUTPUT_WIDTH, OUTPUT_HEIGHT);
          cv::resize(left_image, left_scaled, sz);
          cv::resize(right_image, right_scaled, sz);
        }

        // Publish.
        cv_bridge::CvImage cv_image;
        sensor_msgs::ImagePtr img;
        cv_image.encoding = msg->encoding;
        cv_image.header.stamp = msg->header.stamp;
        if (LEFT_IMAGE_PUBLISHER.getNumSubscribers() > 0 || LEFT_CAMERA_INFO_PUBLISHER.getNumSubscribers() > 0)
        {
          if (LEFT_FRAME.empty())
            LEFT_FRAME = msg->header.frame_id;
          cv_image.image = use_scaled ? left_scaled : left_image;
          cv_image.header.frame_id = LEFT_FRAME;
          img = cv_image.toImageMsg();
          LEFT_IMAGE_PUBLISHER.publish(img);
          LEFT_CAMERA_INFO_MSG.header.stamp = img->header.stamp;
          LEFT_CAMERA_INFO_MSG.header.frame_id = LEFT_FRAME;
          LEFT_CAMERA_INFO_PUBLISHER.publish(LEFT_CAMERA_INFO_MSG);
        }
        if (RIGHT_IMAGE_PUBLISHER.getNumSubscribers() > 0 || RIGHT_CAMERA_INFO_PUBLISHER.getNumSubscribers() > 0)
        {
          if (RIGHT_FRAME.empty())
            RIGHT_FRAME = msg->header.frame_id;
          cv_image.image = use_scaled ? right_scaled : right_image;
          cv_image.header.frame_id = RIGHT_FRAME;
          img = cv_image.toImageMsg();
          RIGHT_IMAGE_PUBLISHER.publish(img);
          RIGHT_CAMERA_INFO_MSG.header.stamp = img->header.stamp;
          RIGHT_CAMERA_INFO_MSG.header.frame_id = RIGHT_FRAME;
          RIGHT_CAMERA_INFO_PUBLISHER.publish(RIGHT_CAMERA_INFO_MSG);
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sxs_stereo");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // load the camera info
    nh.param("left_camera_info_url", LEFT_CAMERA_INFO_URL, std::string(""));
    ROS_INFO("left_camera_info_url=%s\n", LEFT_CAMERA_INFO_URL.c_str());
    nh.param("left_frame", LEFT_FRAME, std::string(""));
    nh.param("right_camera_info_url", RIGHT_CAMERA_INFO_URL, std::string(""));
    ROS_INFO("right_camera_info_url=%s\n", RIGHT_CAMERA_INFO_URL.c_str());
    nh.param("right_frame", RIGHT_FRAME, std::string(""));

    // Load node settings.
    std::string input_image_topic, left_output_image_topic, right_output_image_topic, left_camera_info_topic,
        right_camera_info_topic, left_camera_info_manager, right_camera_info_manager;
    nh.param("input_image_topic", input_image_topic, std::string("input_image_topic_not_set"));
    ROS_INFO("input topic to stereo splitter=%s\n", input_image_topic.c_str());
    nh.param("left_output_image_topic", left_output_image_topic, std::string("left/image_raw"));
    nh.param("right_output_image_topic", right_output_image_topic, std::string("right/image_raw"));
    nh.param("left_camera_info_topic", left_camera_info_topic, std::string("left/camera_info"));
    nh.param("right_camera_info_topic", right_camera_info_topic, std::string("right/camera_info"));
    nh.param("left_camera_info_manager_ns", left_camera_info_manager, std::string("~/left"));
    nh.param("right_camera_info_manager_ns", right_camera_info_manager, std::string("~/right"));
    nh.param("output_width", OUTPUT_WIDTH, 0);    // 0 -> use 1/2 input width.
    nh.param("output_height", OUTPUT_HEIGHT, 0);  // 0 -> use input height.

    // Register publishers and subscriber.
    IMAGE_SUB = nh.subscribe(input_image_topic, 2, &imageCallback);
    LEFT_IMAGE_PUBLISHER = it.advertise(left_output_image_topic, 5);
    RIGHT_IMAGE_PUBLISHER = it.advertise(right_output_image_topic, 5);
    LEFT_CAMERA_INFO_PUBLISHER = nh.advertise<sensor_msgs::CameraInfo>(left_camera_info_topic, 5);
    RIGHT_CAMERA_INFO_PUBLISHER = nh.advertise<sensor_msgs::CameraInfo>(right_camera_info_topic, 5);

    // Camera info managers.
    ros::NodeHandle nh_left(left_camera_info_manager);
    ros::NodeHandle nh_right(right_camera_info_manager);
    // Allocate and initialize camera info managers.
    camera_info_manager::CameraInfoManager left_cinfo(nh_left, "camera", LEFT_CAMERA_INFO_URL);
    camera_info_manager::CameraInfoManager right_cinfo(nh_right, "camera", RIGHT_CAMERA_INFO_URL);
    left_cinfo.loadCameraInfo(LEFT_CAMERA_INFO_URL);
    right_cinfo.loadCameraInfo(RIGHT_CAMERA_INFO_URL);

    // Pre-fill camera_info messages.
    LEFT_CAMERA_INFO_MSG = left_cinfo.getCameraInfo();
    RIGHT_CAMERA_INFO_MSG = right_cinfo.getCameraInfo();

    // Run node until cancelled.
    ros::spin();

    // De-allocate CameraInfoManagers.
    // left_cinfo_.~CameraInfoManager();
    // right_cinfo_.~CameraInfoManager();
}
