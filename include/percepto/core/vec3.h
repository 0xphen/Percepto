#pragma once

#include <cmath>
#include <iostream>

namespace percepto::core
{
/**
 * @file Vec3.h
 * @brief Defines the `Vec3` class — a simple 3D vector type.
 *
 * This header defines the `Vec3` class, a minimal 3D vector utility designed for use
 * in simulation, geometry, and ray tracing contexts. All member functions are defined
 * inline for performance and inlining opportunities.
 *
 * The class supports:
 * - Unary negation
 * - Element-wise addition, subtraction, multiplication, and division
 * - Scalar multiplication (from both left and right)
 * - Dot product and cross product
 * - Vector length, normalization, and indexed component access
 *
 * All methods are defined inline in the header to support maximum compiler optimization
 * and reduce function call overhead.
 *
 * @code
 * Vec3 a(1.0, 2.0, 3.0);
 * Vec3 b = 2.0 * a.normalized();
 * double d = a.dot(b);
 * Vec3 c = a.cross(b);
 * @endcode
 */

class Vec3
{
 public:
  double x, y, z;

  Vec3() : x(0), y(0), z(0) {}
  Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

  // Unary minus
  Vec3 operator-() const { return Vec3(-x, -y, -z); }

  // Element access
  double operator[](int index) const
  {
    if (index < 0 || index > 2) throw std::out_of_range("Index out of bounds");
    return (index == 0) ? x : (index == 1) ? y : z;
  }

  double& operator[](int index)
  {
    if (index < 0 || index > 2) throw std::out_of_range("Index out of bounds");
    return (index == 0) ? x : (index == 1) ? y : z;
  }

  // Arithmetic operators
  Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
  Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
  Vec3 operator*(double t) const { return Vec3(t * x, t * y, t * z); }
  Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
  bool operator==(const Vec3& v) const { return x == v.x && y == v.y && z == v.z; }

  // Compound assignment
  Vec3& operator+=(const Vec3& v)
  {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }

  Vec3& operator-=(const Vec3& v)
  {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  Vec3& operator*=(double t)
  {
    x *= t;
    y *= t;
    z *= t;
    return *this;
  }

  Vec3& operator/=(double t)
  {
    x /= t;
    y /= t;
    z /= t;
    return *this;
  }

  // Length and normalization
  double length_squared() const { return x * x + y * y + z * z; }
  double length() const { return std::sqrt(length_squared()); }

  void normalize()
  {
    double len = length();
    if (len > 0)
    {
      x /= len;
      y /= len;
      z /= len;
    }
  }

  Vec3 normalized() const
  {
    double len = length();
    return (len > 0) ? Vec3(x / len, y / len, z / len) : Vec3(0, 0, 0);
  }

  // Dot and cross product
  double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

  Vec3 cross(const Vec3& v) const
  {
    return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  friend std::ostream& operator<<(std::ostream& os, const Vec3& v)
  {
    return os << v.x << ", " << v.y << ", " << v.z;
  }
};

// Scalar multiplication from the left: scalar * Vec3
inline Vec3 operator*(double t, const Vec3& v)
{
  return v * t;
}
}  // namespace percepto::core