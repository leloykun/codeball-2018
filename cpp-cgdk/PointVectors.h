#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _POINT_VECTORS_H_
#define _POINT_VECTORS_H_

#include <cmath>
#include <string>
#include <vector>

const double EPS = 1e-5;

struct Vec2D {
  double x {0.0};
  double z {0.0};
  Vec2D() {}
  Vec2D(double x, double z) : x(x), z(z) {}
  void set(double _x, double _z) { x=_x; z=_z; }
  double len() const { return std::sqrt(x*x + z*z); }
  Vec2D normalize() const { double l=len(); return {x/l, z/l}; }
  std::string str() const {
    return "("+std::to_string(x)+","+
               std::to_string(z)+")"; }
  Vec2D& operator-=(const Vec2D &other) {
    this->x -= other.x;  this->z -= other.z;
    return *this;  }
  Vec2D& operator+=(const Vec2D &other) {
    this->x += other.x;  this->z += other.z;
    return *this;  }
  Vec2D& operator*=(const double num) {
    this->x *= num;  this->z *= num;
    return *this;  }
};
inline Vec2D operator-(Vec2D lhs, const Vec2D &rhs) {
  lhs -= rhs;  return lhs;  }
inline Vec2D operator+(Vec2D lhs, const Vec2D &rhs) {
  lhs += rhs;  return lhs;  }
inline Vec2D operator*(Vec2D lhs, const double num) {
  lhs *= num;  return lhs;  }


struct Vec3D {
  double x {0.0};
  double z {0.0};
  double y {0.0};
  double t {0.0};
  Vec3D() {}
  Vec3D(double x, double z, double y) : x(x), z(z), y(y) { }
  Vec3D(const Vec2D &plane, double y) : x(plane.x), z(plane.z), y(y) { }
  void set(double _x, double _z, double _y) { x=_x; z=_z; y=_y; }
  void clamp(double val) {
    double l = len();
    if (l < val)  return;
    x *= val/l;
    z *= val/l;
    y *= val/l;  }
  double len() const { return std::sqrt(x*x + z*z + y*y); }
  Vec3D normalize() const {
    double l = len();
    if (l < EPS)  return {x, z, y};
    return {x/l, z/l, y/l};  }
  double dot(const Vec3D &other) const {
    return (x*other.x) + (z*other.z) + (y*other.y);  }
  std::string str() const {
    return "("+std::to_string(x)+","+
               std::to_string(z)+","+
               std::to_string(y)+")"; }
  Vec3D& operator-=(const Vec3D &other) {
    this->x -= other.x;  this->z -= other.z;  this->y -= other.y;
    return *this;  }
  Vec3D& operator+=(const Vec3D &other) {
    this->x += other.x;  this->z += other.z;  this->y += other.y;
    return *this;  }
  Vec3D& operator*=(const double num) {
    this->x *= num;  this->z *= num;  this->y *= num;
    return *this;  }
};
inline Vec3D operator-(Vec3D lhs, const Vec3D &rhs) {
  lhs -= rhs;  return lhs;  }
inline Vec3D operator+(Vec3D lhs, const Vec3D &rhs) {
  lhs += rhs;  return lhs;  }
inline Vec3D operator*(Vec3D lhs, const double num) {
  lhs *= num;  return lhs;  }

inline Vec3D clamp(const Vec3D &v, double val) {
  if (v.len() < val)  return v;
  return v.normalize() * val;  }

inline double clamp(double a, double min_val, double max_val) {
  return std::min(std::max(a, min_val), max_val);  }

typedef std::vector<Vec3D> Path;

struct JumpBallIntercept {
  bool exists;
  bool can_score;
  Path robot_path;
  Path ball_path;
  Vec3D robot_pos;
  Vec3D ball_pos;
};

#endif
