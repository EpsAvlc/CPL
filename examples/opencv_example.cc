#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cpl/core.hh"
#include "cpl/cpl_opencv.hh"

cv::Size paint_size(800, 800);

// void polyLinesF()

int main(int argc, char const *argv[]) {
  cv::Mat paint_ori(paint_size, CV_8UC3, cv::Scalar(40, 40, 40));

  std::vector<cv::Point2d> cv_poly_l, cv_poly_r;
  cv_poly_l.emplace_back(200, 300);
  cv_poly_l.emplace_back(500, 300);
  cv_poly_l.emplace_back(500, 600);
  cv_poly_l.emplace_back(200, 600);

  // cv_poly_r.emplace_back(200, 200);
  // cv_poly_r.emplace_back(500, 200);
  // cv_poly_r.emplace_back(500, 500);
  // cv_poly_r.emplace_back(200, 500);

  cv_poly_r.emplace_back(600, 500);
  cv_poly_r.emplace_back(300, 500);
  cv_poly_r.emplace_back(300, 200);
  cv_poly_r.emplace_back(600, 200);

  std::vector<cv::Point2i> cv_poly_l_i, cv_poly_r_i;
  while (cv::waitKey(20) != 27) {
    cv::Mat paint = paint_ori.clone();
    cv::Mat(cv_poly_l).copyTo(cv_poly_l_i);
    cv::Mat(cv_poly_r).copyTo(cv_poly_r_i);

    std::vector<cpl::PointD> cpl_poly_l_pts, cpl_poly_r_pts;
    cpl_poly_l_pts = cpl::fromCV(cv_poly_l);
    cpl_poly_r_pts = cpl::fromCV(cv_poly_r);
    cpl::ConvexPolygonD cpl_poly_l(cpl_poly_l_pts), cpl_poly_r(cpl_poly_r_pts);
    cpl::ConvexPolygonD cpl_inter_poly = cpl_poly_l.clip(cpl_poly_r_pts);
    double area = cpl_inter_poly.area();
    cv::putText(paint, "Area: " + std::to_string(area), cv::Point2d(300, 100),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255), 3);

    std::vector<cv::Point2d> cv_inter_poly = cpl::toCV(cpl_inter_poly.points());
    std::vector<cv::Point2i> cv_inter_poly_i;
    cv::Mat(cv_inter_poly).copyTo(cv_inter_poly_i);

    cv::polylines(paint, cv_poly_l_i, true, cv::Scalar(255, 0, 0));
    cv::polylines(paint, cv_poly_r_i, true, cv::Scalar(0, 255, 0));
    cv::polylines(paint, cv_inter_poly_i, true, cv::Scalar(0, 0, 255), 3);
    cv::imshow("paint", paint);

    static int l_dx = 1;
    static int l_dy = 1;
    static int r_dx = -1;
    static int r_dy = -1;
    for (int pi = 0; pi < 4; ++pi) {
      cv_poly_l[pi].x += l_dx;
      cv_poly_l[pi].y += l_dy;
      cv_poly_r[pi].x += r_dx;
      cv_poly_r[pi].y += r_dy;

      if (cv_poly_l[pi].x <= 0 || cv_poly_l[pi].x >= paint_size.width - 1) {
        l_dx = -l_dx;
      }
      if (cv_poly_l[pi].y <= 0 || cv_poly_l[pi].y >= paint_size.height - 1) {
        l_dy = -l_dy;
      }
      if (cv_poly_r[pi].x <= 0 || cv_poly_r[pi].x >= paint_size.width - 1) {
        r_dx = -r_dx;
      }
      if (cv_poly_r[pi].y <= 0 || cv_poly_r[pi].y >= paint_size.height - 1) {
        r_dy = -r_dy;
      }
    }
  }
  return 0;
}
