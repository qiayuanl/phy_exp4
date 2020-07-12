//
// Created by qiayuan on 7/11/20.
//

#ifndef PHY_EXP4_INCLUDE_COMMON_H_
#define PHY_EXP4_INCLUDE_COMMON_H_
#include <ros/ros.h>

namespace phy_exp4 {

enum class DebugImage {
  DISABLE = 0,
  ROI = 1,
  MASK = 2,
  MORPHOLOGY = 3,
  CIRCLES = 4
};

template<typename T>
T getParam(ros::NodeHandle &pnh,
           const std::string &param_name, const T &default_val) {
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}

template<typename T>
class MovingAverageFilter {
 public:
  explicit MovingAverageFilter(int num_data)
      : num_data_(num_data), idx_(0), sum_(0.0) {
    buffer_ = new T[num_data_];
    memset((void *) buffer_, 0.0, sizeof(T) * num_data_);
  }

  ~MovingAverageFilter() { delete[] buffer_; }
  void input(T input_value) {
    sum_ -= buffer_[idx_];
    sum_ += input_value;
    buffer_[idx_] = input_value;
    ++idx_;
    idx_ %= num_data_;

  }
  T output() { return sum_ / num_data_; };
  void clear() {
    sum_ = 0.0;
    memset((void *) buffer_, 0.0, sizeof(T) * num_data_);
  }

 private:
  T *buffer_;
  int num_data_;
  int idx_;
  T sum_;
};

}
#endif //RM_DETECTION_INCLUDE_COMMON_H_
