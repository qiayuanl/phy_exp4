//
// Created by qiayuan on 6/21/20.
//

#ifndef RM_DETECTION_INCLUDE_DETECTOR_H_
#define RM_DETECTION_INCLUDE_DETECTOR_H_

#include "common.h"

#include <opencv/cv.hpp>
#include <utility>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <phy_exp4/ProcessConfig.h>
#include <phy_exp4/ArmorConfig.h>
#include <sensor_msgs/CameraInfo.h>
namespace phy_exp4 {
class Detector {
 public:
  explicit Detector(ros::NodeHandle &p_nh);
//  void detect(cv_bridge::CvImageConstPtr &cv_img,
//              const sensor_msgs::CameraInfoConstPtr &info);
  void detect(cv_bridge::CvImageConstPtr &cv_img, ros::Time stamp);
  void drawCircle(const cv::Mat &img);
  void drawMatch(const cv::Mat &img);

  cv::Mat &getRoi() { return roi_; }
  cv::Mat &getMask() { return mask_; }
  cv::Mat &getMorFinal() { return mor_final_; }
  cv::Mat &getResultImg() { return result_img_; }
 protected:
  void templateMethod(ros::Time stamp);
  void hsvMethod(ros::Time stamp);

  void resize(const cv::Mat &img);
  void processReconfigCB(ProcessConfig &config, uint32_t level);
  cv::Mat roi_;
  int roi_lu_x_{}, roi_lu_y_{}, roi_rd_x_{}, roi_rd_y_{};

  cv::Mat hsv_;
  int hsv_bound_[6]{};
  cv::Mat mask_;

  bool mor_order_ = true;
  cv::Mat mor_final_;
  cv::Mat element_close_, element_open_;

  ros::NodeHandle p_nh_;

  std::vector<cv::Vec3f> circles_;
  double circles_param1_{}, circles_param2_{};
  int circles_min_r_{}, circles_max_r_{};

  cv::Mat template_;
  cv::Mat result_;
  cv::TemplateMatchModes match_method_ = cv::TM_SQDIFF;
  cv::Point match_loc_;

  cv::Mat result_img_;

  bool new_ = true;
  int last_y_ = -1;
  std::vector<double> speed_;
  ros::Time last_stamp_;
  MovingAverageFilter<double> filter;

  ros::Publisher pub_;
  dynamic_reconfigure::Server<ProcessConfig> *process_srv_{};
  dynamic_reconfigure::Server<ProcessConfig>::CallbackType cb_;

};

}

#endif //RM_DETECTION_INCLUDE_DETECTOR_H_
