#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _RENDER_UTIL_H_
#define _RENDER_UTIL_H_

#include "PointVectors.h"
#include <string>

struct Color {
  double r {0.0};
  double g {0.0};
  double b {0.0};
  Color() { }
  Color(double r, double g, double b) : r(r), g(g), b(b) { }
};

const Color RED        (1.0, 0.0, 0.0);
const Color GREEN      (0.0, 1.0, 0.0);
const Color BLUE       (0.0, 0.0, 1.0);
const Color VIOLET     (1.0, 0.0, 1.0);
const Color YELLOW     (1.0, 1.0, 0.0);
const Color TEAL       (0.0, 1.0, 1.0);
const Color WHITE      (1.0, 1.0, 1.0);
const Color BLACK      (0.0, 0.0, 0.0);
const Color LIGHT_RED  (1.0, 0.5, 0.5);
const Color LIGHT_BLUE (0.5, 0.5, 1.0);

class RenderUtil {
  std::vector<std::string> objects;

public:

  RenderUtil() { }
  void clear();
  std::string get_json();

  void draw_sphere(
      const Vec3D &pos,
      const double &radius,
      const Color &color,
      const double &alpha);
  void draw_line(
      const Vec3D &p1,
      const Vec3D &p2,
      const double &width,
      const Color &color,
      const double &alpha);

  void draw_border(const double &border_z);
  void draw_ball_path(
      const Path &path,
      const double &radius,
      const Color &color,
      const double &alpha,
      const double &z_limit = 42);
};


#endif
