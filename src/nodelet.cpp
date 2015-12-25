#include "libcam.h"
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace hector_pseye_camera{

class HectorPsEyeCameraNodelet : public nodelet::Nodelet{
public:
  HectorPsEyeCameraNodelet()
    : nodelet::Nodelet()
    , img_counter_(0)
  {
    //this->getNodeHandle();
  }

  void onInit()
  {
    ros::NodeHandle& pn = this->getPrivateNodeHandle();
    
    int img_width = 640;
    int img_height = 480;

    pn.param("camera_name", p_camera_name_, std::string("ps_eye"));
//     pn.param("camera_topic", p_camera_topic_, p_camera_name_ + "/image_raw");
    pn.param("frame_name", p_frame_name_, std::string("ps_eye_frame"));
    pn.param("dev", p_device_name_, std::string("/dev/video0"));
    pn.param("camera_info_url", p_camera_info_url_, std::string(""));
    pn.param("use_every_n_th_image", p_use_every_n_th_image_, 1);
    pn.param("fps", p_fps_, 30);
    pn.param("width", img_width, 640);
    pn.param("height", img_height, 480);

    camera_ = new Camera(p_device_name_.c_str(), img_width, img_height, p_fps_);
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(pn, p_camera_name_, p_camera_info_url_);

    image_transport_ = new image_transport::ImageTransport(getPrivateNodeHandle());
    camera_publisher_ = image_transport_->advertiseCamera("image_raw", 1);

    update_timer_ = pn.createTimer(ros::Duration(1.0/(static_cast<double>(p_fps_))), &HectorPsEyeCameraNodelet::timerPublishImageCallback, this, false );

    cv_img_.header.frame_id = p_frame_name_;
    cv_img_.encoding = sensor_msgs::image_encodings::MONO8;
    cv_img_.image = cv::Mat(img_height,img_width,CV_8UC1);

    //cv::Mat* img = &cvImg.image;
  }

  void timerPublishImageCallback(const ros::TimerEvent& e)
  {

    img_counter_++;

    bool retrieve_image = false;

    if ((img_counter_ % p_use_every_n_th_image_) == 0){
      retrieve_image = true;
    }

    if (retrieve_image){
      camera_->Update(true);
      ros::Time capture_time = ros::Time::now();
      camera_->toMonoMat(&cv_img_.image);
      
      cv_img_.header.stamp = capture_time;

      sensor_msgs::CameraInfoPtr camera_info = boost::make_shared<sensor_msgs::CameraInfo>(camera_info_manager_->getCameraInfo());
      camera_info->header = cv_img_.header;
      camera_publisher_.publish(cv_img_.toImageMsg(), camera_info);
    }else{
      camera_->Update(false);
    }



  }


protected:
  Camera* camera_;
  camera_info_manager::CameraInfoManager* camera_info_manager_;

  image_transport::ImageTransport* image_transport_;
  image_transport::CameraPublisher camera_publisher_;

  cv_bridge::CvImage cv_img_;

  ros::Timer update_timer_;

  std::string p_camera_name_;
  std::string p_camera_topic_;
  std::string p_frame_name_;
  std::string p_device_name_;
  std::string p_camera_info_url_;
  int p_use_every_n_th_image_;
  int p_fps_;

  int img_counter_;
};


}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_pseye_camera::HectorPsEyeCameraNodelet, nodelet::Nodelet)
