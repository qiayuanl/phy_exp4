//
// Created by qiayuan on 7/2/20.
//

#ifndef SRC_RM_DETECTION_INCLUDE_DETECTION_H_
#define SRC_RM_DETECTION_INCLUDE_DETECTION_H_
#include "detector.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <phy_exp4/DetectionConfig.h>

namespace phy_exp4 {

class Detection {
 public:
  Detection();
 private:
  void imageCB(const sensor_msgs::ImageConstPtr &image);

  void reconfigCB(DetectionConfig &config, uint32_t level);

  ros::NodeHandle nh_;
  std::shared_ptr<Detector> detector_;
  cv_bridge::CvImageConstPtr cv_image_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detection_pub_;

  DebugImage debug_image_ = DebugImage::DISABLE;
  dynamic_reconfigure::Server<DetectionConfig> *reconfig_srv_{};
};
}

#endif //SRC_RM_DETECTION_INCLUDE_DETECTION_H_
