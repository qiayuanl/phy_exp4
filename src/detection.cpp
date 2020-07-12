//
// Created by qiayuan on 7/2/20.
//

#include "detection.h"

#include <memory>

namespace phy_exp4 {

Detection::Detection() : nh_("~") {
  detector_ = std::make_shared<Detector>(nh_);
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);

  sub_ = it_->subscribe(
      getParam(nh_, "camera_name", std::string("/galaxy_camera"))
          + "/image_rect_color",
      1, &Detection::imageCB, this);
  image_pub_ = it_->advertise("debug_image", 1);

  reconfig_srv_ = new dynamic_reconfigure::Server<DetectionConfig>(
      ros::NodeHandle(nh_, "detection"));
  dynamic_reconfigure::Server<DetectionConfig>::CallbackType
      cb = boost::bind(&Detection::reconfigCB, this, _1, _2);
  reconfig_srv_->setCallback(cb);
}

void Detection::imageCB(const sensor_msgs::ImageConstPtr &image) {
  try {
    cv_image_ = cv_bridge::toCvShare(image, image->encoding);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //detector_->detect(cv_image_, info);
  detector_->detect(cv_image_, image->header.stamp);
  sensor_msgs::ImagePtr msg;
  if (debug_image_ == DebugImage::DISABLE) {
    detector_->drawMatch(detector_->getRoi());
    msg = cv_bridge::CvImage(
        std_msgs::Header(), "bgr8",
        detector_->getResultImg()).toImageMsg();
  } else if (debug_image_ == DebugImage::ROI)
    msg = cv_bridge::CvImage(
        std_msgs::Header(), "bgr8", detector_->getRoi()).toImageMsg();
  else if (debug_image_ == DebugImage::MASK)
    msg = cv_bridge::CvImage(
        std_msgs::Header(), "mono8", detector_->getMask()).toImageMsg();
  else if (debug_image_ == DebugImage::MORPHOLOGY)
    msg = cv_bridge::CvImage(
        std_msgs::Header(), "mono8",
        detector_->getMorFinal()).toImageMsg();
  else if (debug_image_ == DebugImage::CIRCLES) {
    detector_->drawCircle(detector_->getRoi());
    msg = cv_bridge::CvImage(
        std_msgs::Header(), "bgr8",
        detector_->getResultImg()).toImageMsg();
  }

  image_pub_.publish(msg);
}

void Detection::reconfigCB(DetectionConfig &config, uint32_t level) {
  debug_image_ = DebugImage(config.debug_image);
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "phy_exp4");
  ros::NodeHandle nh("~");
  phy_exp4::Detection detection;
  ros::spin();
}