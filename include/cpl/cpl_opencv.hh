#include <vector>

#include <opencv2/core/core.hpp>

#include "cpl/core.hh"

namespace cpl {
template <typename T>
cv::Point_<T> toCV(const cpl::Vector<T>& vec) {
  cv::Point_<T> ret;
  ret.x = vec.x();
  ret.y = vec.y();
  return ret;
}

template <typename T>
cpl::Vector<T> fromCV(const cv::Point_<T>& pt) {
  cpl::Vector<T> ret;
  ret.x() = pt.x;
  ret.y() = pt.y;
  return ret;
}

template <typename T>
std::vector<cv::Point_<T>> toCV(const std::vector<cpl::Vector<T>>& vec) {
  std::vector<cv::Point_<T>> ret(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    ret[i] = toCV(vec[i]);
  }
  return ret;
}

template <typename T>
std::vector<cpl::Vector<T>> fromCV(const std::vector<cv::Point_<T>>& vec) {
  std::vector<cpl::Vector<T>> ret(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    ret[i] = fromCV(vec[i]);
  }
  return ret;
}

}  // namespace cpl
