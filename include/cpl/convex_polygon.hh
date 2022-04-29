#ifndef CPL_CONVEX_POLYGON_H__
#define CPL_CONVEX_POLYGON_H__

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace cpl {
template <typename T>
class Vector {
 public:
  Vector() {
    x_ = 0;
    y_ = 0;
  }

  void setZero() {
    x_ = 0;
    y_ = 0;
  }

  Vector(T x, T y) : x_(x), y_(y) {}
  T& x() { return x_; }
  T& y() { return y_; }

  Vector<T> operator+(const Vector<T>& p) const {
    Vector ret;
    ret.x() = x_ + p.x_;
    ret.y() = y_ + p.y_;
    return ret;
  }

  void operator+=(const Vector<T>& p) {
    this->x_ += p.x_;
    this->y_ += p.y_;
  }

  Vector<T> operator-(const Vector<T>& p) const {
    Vector ret;
    ret.x() = x_ - p.x_;
    ret.y() = y_ - p.y_;
    return ret;
  }

  Vector<T> operator*(const T val) const {
    Vector ret;
    ret.x() = x_ * val;
    ret.y() = y_ * val;
    return ret;
  }

  friend Vector<T> operator*(const T val, const Vector<T>& vec) {
    return vec * val;
  }

  T operator*(const Vector<T>& p) { return x_ * p.x_ + y_ * p.y_; }

  Vector<T> operator/(const T val) {
    Vector ret;
    ret.x() = x_ / val;
    ret.y() = y_ / val;
    return ret;
  }

  void operator/=(const T val) {
    this->x_ /= val;
    this->y_ /= val;
  }

  friend std::ostream& operator<<(std::ostream& s, Vector<T> t) {
    s << "(" << t.x() << ", " << t.y() << ")";
    return s;
  }

  T cross(const Vector<T> t) const { return x_ * t.y_ - y_ * t.x_; }

  T norm() const { return std::sqrt(x_ * x_ + y_ * y_); }

  Vector<T> normalized() const {
    Vector<T> ret;
    T _norm = norm();
    ret.x_ = x_ / _norm;
    ret.y_ = y_ / _norm;
    return ret;
  }

  void normalize() {
    T _norm = norm();
    x_ /= _norm;
    y_ /= _norm;
  }

 private:
  T x_;
  T y_;
};

template <typename T>
class LineSegment {
 public:
  using PointT = Vector<T>;

  T length() const { return (pts_[1] - pts_[0]).norm(); }

  PointT start() const { return pts_[0]; }
  PointT& start() { return pts_[0]; }
  PointT end() const { return pts_[1]; }
  PointT& end() { return pts_[1]; }

  bool intersect(const LineSegment<T>& q, PointT& inter_pt) const {
    /* https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
     */
    PointT r = end() - start(), s = q.end() - q.start();
    T r_cross_s = r.cross(s);
    if (std::abs(r_cross_s) < 1e-5) {
      return false;
    }
    double t = (q.start() - this->start()).cross(s) / r_cross_s;
    double u = (this->start() - q.start()).cross(r) / -r_cross_s;
    if (t <= 0 || t >= 1 || u <= 0 || u >= 1) {
      return false;
    }

    inter_pt = this->start() + t * r;
    return true;
  }

  friend std::ostream& operator<<(std::ostream&s, const LineSegment& l) {
    s << "[" << l.start() << ", " << l.end();
    return s;
  }

  LineSegment& operator<<(PointT pt) {
    pts_[0] = pt;
    return *this;
  }

  LineSegment& operator,(PointT pt) {
    pts_[1] = pt;
    return *this;
  }

 private:
  PointT pts_[2];
};

enum ClipOperation {
  kInterseciton = 0,
  kUnion,
};

template <typename T>
class ConvexPolygon {
 public:
  using PointT = Vector<T>;
  using LineSegmentT = LineSegment<T>;

  void update() const {
    if (updated_) {
      return;
    }
    PointT center_pt;
    center_pt.setZero();
    for (auto it = pts_.begin(); it != pts_.end(); ++it) {
      center_pt += *it;
    }
    center_pt /= pts_.size();

    for (auto it = pts_.begin(); it != pts_.end(); ++it) {
      PointT dir = *it - center_pt;
      dir.normalize();
      T score = std::atan2(dir.y(), dir.x());
    }

    pts_order_.resize(pts_.size());
    for (size_t pi = 0; pi < pts_.size(); ++pi) {
      pts_order_[pi] = pi;
    }

    std::sort(pts_order_.begin(), pts_order_.end(),
              [&](uint16_t& lhs, uint16_t& rhs) {
                PointT dir_lhs = this->pts_[lhs] - center_pt;
                dir_lhs.normalize();
                T score_lhs = std::atan2(dir_lhs.y(), dir_lhs.x());
                PointT dir_rhs = this->pts_[rhs] - center_pt;
                dir_rhs.normalize();
                T score_rhs = std::atan2(dir_rhs.y(), dir_rhs.x());
                return score_lhs < score_rhs;
              });
  }

  bool containPoint(const PointT pt) const {
    if (!updated_) {
      update();
    }
    bool ret = true;
    size_t pts_size = points().size();
    std::vector<PointT> pts = this->points();
    for (size_t pi = 0; pi < pts_size; ++pi) {
      PointT a = pts[pi];
      PointT b = pts[(pi + 1) % pts_size];
      PointT ab = b - a;
      PointT am = pt - a;

      if (ab.cross(am) < 1e-5) {
        ret = false;
        break;
      }
    }
    return ret;
  }

  void addPoint(const PointT& pt) {
    pts_.push_back(pt);
    updated_ = false;
  }

  friend ConvexPolygon& operator<<(ConvexPolygon& cp, const PointT& pt) {
    cp.addPoint(pt);
    return cp;
  }

  friend ConvexPolygon& operator,(ConvexPolygon& cp, const PointT& pt) {
    cp.addPoint(pt);
    return cp;
  }

  friend std::ostream& operator<<(std::ostream& s, const ConvexPolygon& cp) {
    std::vector<PointT> ls = cp.points();
    s << "{";
    for (size_t pi = 0; pi < ls.size(); ++pi) {
      s << ls[pi];
    }
    s << "}";
    return s;
  }

  T area() const {
    /*https://en.wikipedia.org/wiki/Polygon#Area*/
    T ret = 0;
    if (!updated_) {
      update();
    }
    std::vector<PointT> pts = this->points();
    for (int pi = 0; pi < pts.size(); ++pi) {
      PointT a = pts[pi];
      PointT b = pts[(pi + 1) % pts.size()];
      ret += T(0.5) * (a.cross(b));
    }
    return std::abs(ret);
  }

  ConvexPolygon clip(
      const ConvexPolygon& cp,
      const ClipOperation op = ClipOperation::kInterseciton) const {
    ConvexPolygon ret;
    if (op == ClipOperation::kInterseciton) {
      if (!cp.valid() || !this->valid()) {
        std::cerr << "[ConvexPolygon]: either this polygon or the other "
                     "polygon is invalid."
                  << std::endl;
        std::cerr << "[ConvexPolygon]: lhs pts: " << this->pts_.size()
                  << ", rhs pts: " << cp.pts_.size() << std::endl;
        return ret;
      }
      /* step 1: select all points that are contained in cp. */
      for (auto it = pts_.begin(); it != pts_.end(); ++it) {
        if (cp.containPoint(*it)) {
          ret.addPoint(*it);
        }
      }
      /* step 2: select all points that are contained in this polygon. */
      for (auto it = cp.pts_.begin(); it != cp.pts_.end(); ++it) {
        if (this->containPoint(*it)) {
          ret.addPoint(*it);
        }
      }
      /* step 3: select all points that are intersections of two polygon. */
      std::vector<LineSegmentT> line_segs = this->lineSegments();
      std::vector<LineSegmentT> rhs_line_segs = cp.lineSegments();
      PointT inter_pt;
      for (size_t li = 0; li < line_segs.size(); ++li) {
        for (size_t ri = 0; ri < rhs_line_segs.size(); ++ri) {
          if (line_segs[li].intersect(rhs_line_segs[ri], inter_pt)) {
            ret.addPoint(inter_pt);
          }
        }
      }
    } else if (op == ClipOperation::kUnion) {
      std::cerr << "kUnion has not been implemented." << std::endl;
      return ret;
    }
  }

  bool valid() const {
    if (pts_.size() <= 2) {
      return false;
    }

    return true;
  }

  void reset() { pts_.clear(); }

  std::vector<PointT> points() const {
    static std::vector<PointT> ordered_pts;
    if (!updated_) {
      update();
      ordered_pts.resize(pts_.size());
      for (size_t pi = 0; pi < pts_.size(); ++pi) {
        ordered_pts[pi] = pts_[pts_order_[pi]];
      }
    }
    return ordered_pts;
  }

  std::vector<LineSegmentT> lineSegments() const {
    static std::vector<LineSegmentT> ret;
    if (!updated_) {
      update();
      ret.resize(pts_.size());
      LineSegmentT line;
      for (size_t pi = 0; pi < pts_.size(); ++pi) {
        PointT start = pts_[pts_order_[pi]];
        PointT end = pts_[pts_order_[(pi + 1) % pts_.size()]];
        line << start, end;
        ret[pi] = line;
      }
    }
    return ret;
  }

 private:
  std::vector<PointT> pts_;
  mutable std::vector<uint16_t> pts_order_;
  bool updated_ = false;
};

typedef Vector<float> PointF;
typedef Vector<double> PointD;

typedef ConvexPolygon<float> ConvexPolygonF;
typedef ConvexPolygon<double> ConvexPolygonD;

typedef LineSegment<float> LineSegmentF;
typedef LineSegment<double> LineSegmentD;

}  // namespace cpl
#endif
