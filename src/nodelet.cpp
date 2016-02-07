/*
 Copyright (C) 2015 Stefan Kohlbrecher, TU Darmstadt

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "libcam.h"
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

namespace ps4_stereo_camera{


struct PS4CameraMode
{
  int img_width;
  int img_height;
  int raw_width;
  int raw_height;
  int raw_offset_x;
};

class StereoCameraNodelet : public nodelet::Nodelet{
public:
  StereoCameraNodelet()
    : nodelet::Nodelet()
    , img_counter_(0)
  {
  }

  void onInit()
  {
    ros::NodeHandle& pn = this->getPrivateNodeHandle();
    
    //int img_width = 640;
    //int img_height = 480;

    pn.param("camera_name", p_camera_name_, std::string("ps_eye"));
//     pn.param("camera_topic", p_camera_topic_, p_camera_name_ + "/image_raw");
    pn.param("frame_name", p_frame_name_, std::string("ps_eye_frame"));
    pn.param("dev", p_device_name_, std::string("/dev/video0"));
    pn.param("left_camera_info_url", p_left_camera_info_url_, std::string(""));
    pn.param("right_camera_info_url", p_right_camera_info_url_, std::string(""));
    pn.param("use_every_n_th_image", p_use_every_n_th_image_, 1);
    pn.param("fps", p_fps_, 30);
    //pn.param("width", img_width, 640);
    //pn.param("height", img_height, 480);

    left_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle("~/left"), p_camera_name_+"/left", p_left_camera_info_url_));
    right_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle("~/right"), p_camera_name_+"/right", p_right_camera_info_url_));

    image_transport_ = new image_transport::ImageTransport(getPrivateNodeHandle());

    raw_debug_publisher_ = image_transport_->advertise("image_raw_debug", 5);

    left_mono_publisher_ = image_transport_->advertise("left/image_mono", 5);
    right_mono_publisher_ = image_transport_->advertise("right/image_mono", 5);
    left_color_publisher_ = image_transport_->advertise("left/image_raw", 5);
    right_color_publisher_ = image_transport_->advertise("right/image_raw", 5);

    left_camera_info_publisher_ = getPrivateNodeHandle().advertise<sensor_msgs::CameraInfo>("left/camera_info",5, false);
    right_camera_info_publisher_ = getPrivateNodeHandle().advertise<sensor_msgs::CameraInfo>("right/camera_info",5, false);

    this->setupModes();

    current_mode_ = 2;

    this->setupResolution(camera_modes_[current_mode_].img_width,
                          camera_modes_[current_mode_].img_height,
                          camera_modes_[current_mode_].raw_width,
                          camera_modes_[current_mode_].raw_height
                          );

    camera_.reset(new Camera(p_device_name_.c_str(),
                             camera_modes_[current_mode_].raw_width,
                             camera_modes_[current_mode_].raw_height,
                             p_fps_));

    stream_thread_.reset(new boost::thread(boost::bind(&StereoCameraNodelet::run, this)));

    //update_timer_ = pn.createTimer(ros::Duration(1.0/(static_cast<double>(p_fps_))), &StereoCameraNodelet::timerPublishImageCallback, this, false );

    //cv::Mat* img = &cvImg.image;
  }

  void setupModes()
  {
    PS4CameraMode high;

    high.img_height = 800;
    high.img_width = 1280;
    high.raw_height = 808;
    high.raw_width = 3448;
    high.raw_offset_x = 48;
    camera_modes_.push_back(high);

    PS4CameraMode medium;
    medium.img_height = 400;
    medium.img_width = 640;
    medium.raw_height = 408;
    medium.raw_width = 1748;
    medium.raw_offset_x = 24;
    camera_modes_.push_back(medium);

    PS4CameraMode low;
    low.img_height = 192;
    low.img_width = 319;
    low.raw_height = 200;
    low.raw_width = 898;
    low.raw_offset_x = 48;
    camera_modes_.push_back(low);
  }

  void setupResolution(int width, int height, int raw_width, int raw_height)
  {
    cv_img_.header.frame_id = p_frame_name_;
    cv_img_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_img_.image = cv::Mat(raw_height,raw_width,CV_8UC1);

    left_cv_img_.header.frame_id = p_frame_name_;
    left_cv_img_.encoding = sensor_msgs::image_encodings::MONO8;
    left_cv_img_.image = cv::Mat(height,width,CV_8UC1);

    right_cv_img_.header.frame_id = p_frame_name_;
    right_cv_img_.encoding = sensor_msgs::image_encodings::MONO8;
    right_cv_img_.image = cv::Mat(height,width,CV_8UC1);

    left_cv_color_img_.header.frame_id = p_frame_name_;
    left_cv_color_img_.encoding = sensor_msgs::image_encodings::YUV422;
    left_cv_color_img_.image = cv::Mat(height,width,CV_8UC2);

    right_cv_color_img_.header.frame_id = p_frame_name_;
    right_cv_color_img_.encoding = sensor_msgs::image_encodings::YUV422;
    right_cv_color_img_.image = cv::Mat(height,width,CV_8UC2);
  }

  /*
  void timerPublishImageCallback(const ros::TimerEvent& e)
  {
    while (ros::ok()){
      this->retrieveAndPublishImage();
    }
  }
  */

  void run()
  {
    while (ros::ok()){
      img_counter_++;

      bool retrieve_image = false;

      if ((img_counter_ % p_use_every_n_th_image_) == 0){
        retrieve_image = true;
      }

      if (retrieve_image){
        camera_->Update(true);

        ros::Time capture_time = ros::Time::now();

        int img_width  = camera_modes_[current_mode_].img_width;
        int img_height = camera_modes_[current_mode_].img_height;
        int img_offset = camera_modes_[current_mode_].raw_offset_x;

        cv_img_.header.stamp = capture_time;


        if (raw_debug_publisher_.getNumSubscribers() > 0){
          camera_->toMonoMat(&cv_img_.image);
          raw_debug_publisher_.publish(cv_img_.toImageMsg());
        }

        bool left_camera_info_requested = left_camera_info_publisher_.getNumSubscribers() > 0;
        bool left_mono_requested = left_mono_publisher_.getNumSubscribers() > 0;
        bool left_color_requested = left_color_publisher_.getNumSubscribers() > 0;

        bool right_camera_info_requested = right_camera_info_publisher_.getNumSubscribers() > 0;
        bool right_mono_requested = right_mono_publisher_.getNumSubscribers() > 0;
        bool right_color_requested = right_color_publisher_.getNumSubscribers() > 0;

        if (left_camera_info_requested ||
            left_mono_requested ||
            left_color_requested)
        {
          sensor_msgs::CameraInfoPtr left_camera_info = boost::make_shared<sensor_msgs::CameraInfo>(left_camera_info_manager_->getCameraInfo());
          left_camera_info->header = cv_img_.header;

          left_camera_info_publisher_.publish(left_camera_info);
        }

        if (right_camera_info_requested ||
            right_mono_requested ||
            right_color_requested)
        {
          sensor_msgs::CameraInfoPtr right_camera_info = boost::make_shared<sensor_msgs::CameraInfo>(right_camera_info_manager_->getCameraInfo());
          right_camera_info->header = cv_img_.header;

          right_camera_info_publisher_.publish(right_camera_info);
        }



        if (left_mono_requested){
          camera_->toMonoMat(&left_cv_img_.image, img_offset + img_width, img_width, img_height);

          left_cv_img_.header = cv_img_.header;

          left_mono_publisher_.publish(left_cv_img_.toImageMsg());
        }

        if (right_mono_requested){
          camera_->toMonoMat(&right_cv_img_.image, img_offset, img_width, img_height);

          right_cv_img_.header = cv_img_.header;

          right_mono_publisher_.publish(right_cv_img_.toImageMsg());
        }


        if (left_color_requested){
          camera_->toColorMat(&left_cv_color_img_.image, img_offset + img_width, img_width, img_height);

          left_cv_color_img_.header = cv_img_.header;

          left_color_publisher_.publish(left_cv_color_img_.toImageMsg());
        }

        if (right_color_requested){
          camera_->toColorMat(&right_cv_color_img_.image, img_offset, img_width, img_height);

          right_cv_color_img_.header = cv_img_.header;

          right_color_publisher_.publish(right_cv_color_img_.toImageMsg());
        }

      }else{
        camera_->Update(false);
      }
      if (!camera_->freeBuffer()){
        NODELET_ERROR("Error while freeing buffer!");
      }
    }
  }


protected:
  boost::shared_ptr<Camera> camera_;
  boost::scoped_ptr<boost::thread> stream_thread_;

  std::vector<PS4CameraMode> camera_modes_;

  image_transport::ImageTransport* image_transport_;
  image_transport::Publisher raw_debug_publisher_;

  image_transport::Publisher left_mono_publisher_;
  image_transport::Publisher right_mono_publisher_;
  image_transport::Publisher left_color_publisher_;
  image_transport::Publisher right_color_publisher_;
  ros::Publisher left_camera_info_publisher_;
  ros::Publisher right_camera_info_publisher_;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> left_camera_info_manager_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> right_camera_info_manager_;

  cv_bridge::CvImage cv_img_;
  cv_bridge::CvImage left_cv_img_;
  cv_bridge::CvImage right_cv_img_;
  cv_bridge::CvImage left_cv_color_img_;
  cv_bridge::CvImage right_cv_color_img_;

  ros::Timer update_timer_;


  int current_mode_;

  std::string p_camera_name_;
  std::string p_camera_topic_;
  std::string p_frame_name_;
  std::string p_device_name_;
  std::string p_left_camera_info_url_;
  std::string p_right_camera_info_url_;
  int p_use_every_n_th_image_;
  int p_fps_;

  int img_counter_;
};


}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ps4_stereo_camera::StereoCameraNodelet, nodelet::Nodelet)
