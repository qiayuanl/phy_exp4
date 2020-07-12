//
// Created by qiayuan on 7/11/20.
//

#include "detector.h"
#include <phy_exp4/PosSpeedStamp.h>
#include <numeric>
namespace phy_exp4 {

Detector::Detector(ros::NodeHandle &p_nh) :
    p_nh_(ros::NodeHandle(p_nh, "process")),
    process_srv_(new dynamic_reconfigure::Server<ProcessConfig>(p_nh_)),
    filter(1) {
  cb_ = boost::bind(&Detector::processReconfigCB, this, _1, _2);
  process_srv_->setCallback(cb_);
  pub_ = p_nh_.advertise<PosSpeedStamp>("pos_speed", 100);
  std::string
      a = getParam(p_nh,
                   "template_path",
                   std::string("/home/qiayuan/Pictures/exp4/template.png"));
  template_ = imread(a, cv::IMREAD_COLOR);
}

void Detector::processReconfigCB(ProcessConfig &config, uint32_t level) {
  (void) level;
  roi_lu_x_ = config.roi_lu_x;
  roi_lu_y_ = config.roi_lu_y;
  roi_rd_x_ = config.roi_rd_x;
  roi_rd_y_ = config.roi_rd_y;

  hsv_bound_[0] = config.h_max;
  hsv_bound_[1] = config.h_min;
  hsv_bound_[2] = config.s_max;
  hsv_bound_[3] = config.s_min;
  hsv_bound_[4] = config.v_max;
  hsv_bound_[5] = config.v_min;

  mor_order_ = config.order;
  element_open_ = getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(2 * config.open_size + 1, 2 * config.open_size + 1),
      cv::Point(config.open_size, config.open_size));
  element_close_ = getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(2 * config.close_size + 1, 2 * config.close_size + 1),
      cv::Point(config.close_size, config.close_size));

  circles_param1_ = config.circles_param1;
  circles_param2_ = config.circles_param2;
  circles_min_r_ = config.circles_min_r;
  circles_max_r_ = config.circles_max_r;
}

void Detector::detect(cv_bridge::CvImageConstPtr &cv_img, ros::Time stamp) {
  resize(cv_img->image);
  templateMethod(stamp);
  //hsvMethod(stamp);
}

void Detector::templateMethod(ros::Time stamp) {
  matchTemplate(roi_, template_, result_, match_method_);
  normalize(result_, result_, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
  double min_val, max_val;
  cv::Point min_loc, max_loc;
  minMaxLoc(result_, &min_val, &max_val, &min_loc, &max_loc, cv::Mat());
  if (match_method_ == cv::TM_SQDIFF
      || match_method_ == cv::TM_SQDIFF_NORMED)
    match_loc_ = min_loc;
  else match_loc_ = max_loc;

  int y = match_loc_.y;
  auto dy = (double) (y - last_y_);

  double dt = (stamp - last_stamp_).toSec();
  if (new_) {
    new_ = false;
    last_y_ = y;
    last_stamp_ = stamp;

    filter.clear();
  }
  if (dt < 0.) {
    new_ = true;

    double sum = std::accumulate(std::begin(speed_), std::end(speed_), 0.0);
    double mean = sum / speed_.size();
    double accum = 0.0;
    std::for_each(std::begin(speed_),
                  std::end(speed_),
                  [&](const double d) {
                    accum += (d - mean) * (d - mean);
                  });
    double std = sqrt(accum / (speed_.size() - 1));

    ROS_INFO("mean: %f, std %f, sample %d", mean, std, speed_.size());
    speed_.clear();

  } else if (dy < 250 && dy > 0) {
    filter.input(dy);
    double speed = filter.output() / dt / 1293. * 0.250;

    speed_.push_back(speed);

    last_y_ = y;
    last_stamp_ = stamp;

    PosSpeedStamp pos_speed;
    pos_speed.header.stamp = stamp;
    pos_speed.pos = (double) y / 1293. * 0.250;
    pos_speed.speed = speed;
    pub_.publish(pos_speed);
  }
}

void Detector::hsvMethod(ros::Time stamp) {

  cvtColor(roi_, hsv_, CV_BGR2HSV);

  cv::inRange(hsv_,
              cv::Scalar(hsv_bound_[1], hsv_bound_[3], hsv_bound_[5]),
              cv::Scalar(hsv_bound_[0], hsv_bound_[2], hsv_bound_[4]),
              mask_);

  //morphology
  if (mor_order_) { // open before close
    morphologyEx(mask_, mor_final_, cv::MORPH_OPEN, element_open_,
                 cv::Point(-1, -1));
    morphologyEx(mor_final_, mor_final_, cv::MORPH_CLOSE, element_close_,
                 cv::Point(-1, -1));
  } else {
    morphologyEx(mask_, mor_final_, cv::MORPH_CLOSE, element_close_,
                 cv::Point(-1, -1));
    morphologyEx(mor_final_, mor_final_, cv::MORPH_OPEN, element_open_,
                 cv::Point(-1, -1));
  }
  //circles
  cv::HoughCircles(mor_final_, circles_, CV_HOUGH_GRADIENT, 1,
                   500, circles_param1_, circles_param2_,
                   circles_min_r_, circles_max_r_);

  for (auto &i : circles_) {
    cv::Point center(cvRound(i[0]), cvRound(i[1]));
    int y = center.y;

    auto dy = (double) (y - last_y_);
    double dt = (stamp - last_stamp_).toSec();

    if (new_) {
      new_ = false;
      last_y_ = y;
      last_stamp_ = stamp;

      filter.clear();
    }
    if (dt < 0.) {
      new_ = true;

      double sum = std::accumulate(std::begin(speed_), std::end(speed_), 0.0);
      double mean = sum / speed_.size();
      double accum = 0.0;
      std::for_each(std::begin(speed_),
                    std::end(speed_),
                    [&](const double d) {
                      accum += (d - mean) * (d - mean);
                    });
      double std = sqrt(accum / (double) (speed_.size() - 1));

      ROS_INFO("mean: %f, std %f, sample %d", mean, std, (int) speed_.size());
      speed_.clear();

    } else if (dy < 250 && dy > 0) {
      filter.input(dy);
      double speed = filter.output() / dt / 1293. * 0.250;

      speed_.push_back(speed);

      last_y_ = y;
      last_stamp_ = stamp;

      PosSpeedStamp pos_speed;
      pos_speed.header.stamp = stamp;
      pos_speed.pos = (double) y / 1293. * 0.250;
      pos_speed.speed = speed;
      pub_.publish(pos_speed);
    }

  }
}

void Detector::resize(const cv::Mat &img) {
  cv::Point lu = cv::Point(roi_lu_x_, roi_lu_y_);  /* point left up */
  cv::Point rd = cv::Point(roi_rd_x_, roi_rd_y_);  /* point right down */

  cv::Rect rect_roi_ = cv::Rect(lu, rd);
  img(rect_roi_).copyTo(roi_);
}

void Detector::drawCircle(const cv::Mat &img) {
  result_img_ = img.clone();
  for (auto &i : circles_) {
    cv::Point center(cvRound(i[0]), cvRound(i[1]));
    int radius = cvRound(i[2]);
    // circle center
    circle(result_img_, center, 2, cv::Scalar(0, 255, 0), -1, 8, 0);
    // circle outline
    circle(result_img_, center, radius, cv::Scalar(255, 255, 0), 3, 8, 0);
  }
}
void Detector::drawMatch(const cv::Mat &img) {
  result_img_ = img.clone();
  rectangle(result_img_,
            match_loc_,
            cv::Point(match_loc_.x + template_.cols,
                      match_loc_.y + template_.rows),
            cv::Scalar(255, 255, 0), 2, 8, 0);
}

}
