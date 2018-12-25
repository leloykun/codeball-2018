#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _POINT_VECTORS_H_
#define _POINT_VECTORS_H_

#include <cmath>

struct Vec2D {
  double x {0.0};
  double z {0.0};
  Vec2D() {}
  Vec2D(double x, double z) : x(x), z(z) {}
  void set(double _x, double _z) { x=_x; z=_z; }
  double len() { return std::sqrt(x*x + z*z); }
  Vec2D operator-(const Vec2D &other) { return {x-other.x, z-other.z}; }
  void operator-=(const Vec2D &other) { x -= other.x; z -= other.z; }
  Vec2D operator+(const Vec2D &other) { return {x+other.x, z+other.z}; }
  void operator+=(const Vec2D &other) { x += other.x; z += other.z; }
  Vec2D operator*(const double num) { return {x*num, z*num}; }
  void operator*=(const double num) { x *= num; z *= num; }
  Vec2D normalize() { double l=len(); return {x/l, z/l}; }
};

struct Vec3D {
  double x {0.0};
  double z {0.0};
  double y {0.0};
  Vec3D() {}
  Vec3D(double x, double z, double y) : x(x), z(z), y(y) {}
  void set(double _x, double _z, double _y) { x=_x; z=_z; y=_y; }
  double len() { return std::sqrt(x*x + z*z + y*y); }
  Vec3D operator-(const Vec3D &other) { return {x-other.x, z-other.z, y-other.y}; }
  void operator-=(const Vec3D &other) { x -= other.x; z -= other.z; y -= other.y; }
  Vec3D operator+(const Vec3D &other) { return {x+other.x, z+other.z, y+other.y}; }
  void operator+=(const Vec3D &other) { x += other.x; z += other.z; y += other.y; }
  Vec3D operator*(double num) { return {x*num, z*num, y*num}; }
  void operator*=(double num) { x *= num; z *= num; y *= num; }
  Vec3D normalize() { double l=len(); return {x/l, z/l, y/l}; }
  double dot(const Vec3D &other) { return (x*other.x) + (z*other.z) + (y*other.y); }
  void clamp(double val) { x=std::min(x, val); z=std::min(z, val); y=std::min(y, val); }
};

#endif
