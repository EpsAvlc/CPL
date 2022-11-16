#include "cpl/core.hh"

#include <gtest/gtest.h>

TEST(ConvexPolygonTest, Update) {
  cpl::ConvexPolygonF cp1;
  cp1 << cpl::PointF(100, 0) << cpl::PointF(0, 100) << cpl::PointF(0, 0)
      << cpl::PointF(100, 100);
  cp1.update();
  EXPECT_NEAR(cp1.points()[0].x(), 0, 1e-4);
  EXPECT_NEAR(cp1.points()[0].y(), 0, 1e-4);

  EXPECT_NEAR(cp1.points()[1].x(), 100, 1e-4);
  EXPECT_NEAR(cp1.points()[1].y(), 0, 1e-4);

  EXPECT_NEAR(cp1.points()[2].x(), 100, 1e-4);
  EXPECT_NEAR(cp1.points()[2].y(), 100, 1e-4);

  EXPECT_NEAR(cp1.points()[3].x(), 0, 1e-4);
  EXPECT_NEAR(cp1.points()[3].y(), 100, 1e-4);
}

TEST(ConvexPolygonTest, ContainPoint) {
  cpl::ConvexPolygonF cp;
  cp << cpl::PointF(100, 0) << cpl::PointF(0, 100) << cpl::PointF(0, 0)
     << cpl::PointF(100, 100);
  EXPECT_EQ(cp.containPoint(cpl::PointF(50, 50)), true);
  EXPECT_EQ(cp.containPoint(cpl::PointF(150, 50)), false);
  EXPECT_EQ(cp.containPoint(cpl::PointF(100, 100)), false);
}

TEST(ConvexPolygonTest, Area) {
  cpl::ConvexPolygonF cp;
  cp << cpl::PointF(100, 0), cpl::PointF(0, 100), cpl::PointF(0, 0),
      cpl::PointF(100, 100);
  EXPECT_NEAR(cp.area(), 10000.f, 1e-5);

  cp.reset();
  cp << cpl::PointF(100, 0), cpl::PointF(100, 100), cpl::PointF(0, 0);
  EXPECT_NEAR(cp.area(), 5000.f, 1e-5);

  cp.reset();
  cp << cpl::PointF(100, 0), cpl::PointF(100, 0), cpl::PointF(0, 0);
  EXPECT_NEAR(cp.area(), 0.f, 1e-5);
}

TEST(ConvexPolygonTest, LineIntersect) {
  cpl::LineSegmentF line1, line2;
  line1 << cpl::PointF(0, 0), cpl::PointF(100, 100);
  line2 << cpl::PointF(100, 0), cpl::PointF(0, 100);
  cpl::PointF inter_pt;
  bool intersected = line1.intersectWith(line2, inter_pt);
  EXPECT_TRUE(intersected);
  EXPECT_NEAR(inter_pt.x(), 50.f, 1e-5);
  EXPECT_NEAR(inter_pt.y(), 50.f, 1e-5);

  line1 << cpl::PointF(0, 0), cpl::PointF(100, 0);
  line2 << cpl::PointF(-100, -100), cpl::PointF(-100, 0);
  std::cout << line1.start() << ", " << line1.end() << std::endl;
  std::cout << line2.start() << ", " << line2.end() << std::endl;
  intersected = line1.intersectWith(line2, inter_pt);
  std::cout << inter_pt << std::endl;
  EXPECT_FALSE(intersected);
}

TEST(ConvexPolygonTest, CPIntersect) {
  cpl::ConvexPolygonF cp1;
  cp1 << cpl::PointF(100, 0), cpl::PointF(0, 100), cpl::PointF(0, 0),
      cpl::PointF(100, 100);

  cpl::ConvexPolygonF cp2;
  cp2 << cpl::PointF(50, 50), cpl::PointF(150, 50), cpl::PointF(150, 150),
      cpl::PointF(50, 150);

  cpl::ConvexPolygonF inter_cp = cp1.clip(cp2);
  std::cout << inter_cp << std::endl;
  std::cout << inter_cp << std::endl;
  EXPECT_NEAR(inter_cp.area(), 2500, 1e-4);
  inter_cp = cp2.clip(cp1);
  EXPECT_NEAR(inter_cp.area(), 2500, 1e-4);
}
