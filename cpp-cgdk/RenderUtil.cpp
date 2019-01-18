#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _RENDER_UTIL_CPP_
#define _RENDER_UTIL_CPP_

#include "RenderUtil.h"

void RenderUtil::clear() {
  objects.clear();
}

std::string RenderUtil::get_json() {
  std::string json = "[";
  for (int i = 0; i < int(objects.size()); ++i) {
    if (i)  json += ",";
    json += objects[i];
  }
  json += "]";
  return json;
}


void RenderUtil::draw_sphere(
    const Vec3D &pos,
    const double &radius,
    const Color &color,
    const double &alpha) {
  std::string pos_str = "\"x\":" + std::to_string(pos.x) + "," +
                        "\"y\":" + std::to_string(pos.y) + "," +
                        "\"z\":" + std::to_string(pos.z);
  std::string radius_str = "\"radius\":" + std::to_string(radius);
  std::string color_str = "\"r\":" + std::to_string(color.r) + "," +
                          "\"g\":" + std::to_string(color.g) + "," +
                          "\"b\":" + std::to_string(color.b) + "," +
                          "\"a\":" + std::to_string(alpha);

  objects.push_back("{\"Sphere\":{" + pos_str + "," +
                                      radius_str + "," +
                                      color_str + "}}");
}

void RenderUtil::draw_line(
    const Vec3D &p1,
    const Vec3D &p2,
    const double &width,
    const Color &color,
    const double &alpha) {
  std::string p1_str = "\"x1\":" + std::to_string(p1.x) + "," +
                       "\"y1\":" + std::to_string(p1.y) + "," +
                       "\"z1\":" + std::to_string(p1.z);
  std::string p2_str = "\"x2\":" + std::to_string(p2.x) + "," +
                       "\"y2\":" + std::to_string(p2.y) + "," +
                       "\"z2\":" + std::to_string(p2.z);
  std::string width_str = "\"width\":" + std::to_string(width);
  std::string color_str = "\"r\":" + std::to_string(color.r) + "," +
                          "\"g\":" + std::to_string(color.g) + "," +
                          "\"b\":" + std::to_string(color.b) + "," +
                          "\"a\":" + std::to_string(alpha);

  objects.push_back("{\"Line\": {" + p1_str + "," +
                                     p2_str + "," +
                                     width_str + "," +
                                     color_str + "}}");
}


void RenderUtil::draw_border(const double &border_z) {
  /*  b -- a
   *  |    |
   *  c -- d
   */
  Vec3D corner_A = Vec3D( 22, border_z, 17);
  Vec3D corner_B = Vec3D(-22, border_z, 17);
  Vec3D corner_C = Vec3D(-22, border_z,  1);
  Vec3D corner_D = Vec3D( 22, border_z,  1);
  // border (cross)
  draw_line(corner_A, corner_C, 5, GREEN, 0.5);
  draw_line(corner_B, corner_D, 5, GREEN, 0.5);
  // border (box)
  draw_line(corner_A, corner_B, 5, GREEN, 0.5);
  draw_line(corner_B, corner_C, 5, GREEN, 0.5);
  draw_line(corner_C, corner_D, 5, GREEN, 0.5);
  draw_line(corner_D, corner_A, 5, GREEN, 0.5);
}

void RenderUtil::draw_path(
    const Path &path,
    const double &radius,
    const Color &color,
    const double &alpha,
    const bool &has_z_limit,
    const double &z_limit) {
  for (const PosVelTime &pvt : path) {
    if (has_z_limit and std::fabs(pvt.position.z) >= z_limit) {
      draw_sphere(pvt.position, radius + 0.5, color, 1.0);
      break;
    }
    draw_sphere(pvt.position, radius, color, alpha);
  }
}

#endif
