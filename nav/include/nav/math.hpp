#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace nav {
constexpr double pi = 3.14159265358979323846;
constexpr double deg = pi / 180.0;
template <std::size_t N> struct Vec {
  std::array<double, N> v{};
  double &operator[](std::size_t i) { return v[i]; }
  double operator[](std::size_t i) const { return v[i]; }
};
template <std::size_t R, std::size_t C> struct Mat {
  std::array<double, R * C> v{};
  double &operator()(std::size_t r, std::size_t c) { return v[r * C + c]; }
  double operator()(std::size_t r, std::size_t c) const { return v[r * C + c]; }
};
using Vec3 = Vec<3>;
template <std::size_t N> Vec<N> operator+(Vec<N> a, const Vec<N> &b) {
  for (std::size_t i = 0; i < N; ++i)
    a[i] += b[i];
  return a;
}
template <std::size_t N> Vec<N> operator-(Vec<N> a, const Vec<N> &b) {
  for (std::size_t i = 0; i < N; ++i)
    a[i] -= b[i];
  return a;
}
template <std::size_t N> Vec<N> operator-(Vec<N> a) {
  for (auto &x : a.v)
    x = -x;
  return a;
}
template <std::size_t N> Vec<N> operator*(Vec<N> a, double s) {
  for (auto &x : a.v)
    x *= s;
  return a;
}
template <std::size_t N> Vec<N> operator*(double s, Vec<N> a) { return a * s; }
template <std::size_t N> Vec<N> operator/(Vec<N> a, double s) {
  return a * (1.0 / s);
}
template <std::size_t N> double dot(const Vec<N> &a, const Vec<N> &b) {
  double s = 0;
  for (std::size_t i = 0; i < N; ++i)
    s += a[i] * b[i];
  return s;
}
template <std::size_t N> double norm(const Vec<N> &a) {
  return std::sqrt(dot(a, a));
}
template <std::size_t N> bool finite(const Vec<N> &a) {
  for (double x : a.v)
    if (!std::isfinite(x))
      return false;
  return true;
}
template <std::size_t R, std::size_t C> bool finite(const Mat<R, C> &a) {
  for (double x : a.v)
    if (!std::isfinite(x))
      return false;
  return true;
}
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
           a[0] * b[1] - a[1] * b[0]}};
}
template <std::size_t N> Mat<N, N> identity() {
  Mat<N, N> a;
  for (std::size_t i = 0; i < N; ++i)
    a(i, i) = 1;
  return a;
}
template <std::size_t R, std::size_t C>
Mat<C, R> transpose(const Mat<R, C> &a) {
  Mat<C, R> b;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t j = 0; j < C; ++j)
      b(j, i) = a(i, j);
  return b;
}
template <std::size_t R, std::size_t K, std::size_t C>
Mat<R, C> operator*(const Mat<R, K> &a, const Mat<K, C> &b) {
  Mat<R, C> o;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t k = 0; k < K; ++k) {
      const double s = a(i, k);
      if (s != 0)
        for (std::size_t j = 0; j < C; ++j)
          o(i, j) += s * b(k, j);
    }
  return o;
}
template <std::size_t R, std::size_t C>
Vec<R> operator*(const Mat<R, C> &a, const Vec<C> &b) {
  Vec<R> o;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t j = 0; j < C; ++j)
      o[i] += a(i, j) * b[j];
  return o;
}
template <std::size_t R, std::size_t C>
Mat<R, C> operator+(Mat<R, C> a, const Mat<R, C> &b) {
  for (std::size_t i = 0; i < R * C; ++i)
    a.v[i] += b.v[i];
  return a;
}
template <std::size_t R, std::size_t C>
Mat<R, C> operator-(Mat<R, C> a, const Mat<R, C> &b) {
  for (std::size_t i = 0; i < R * C; ++i)
    a.v[i] -= b.v[i];
  return a;
}
template <std::size_t R, std::size_t C>
Mat<R, C> operator*(Mat<R, C> a, double s) {
  for (auto &x : a.v)
    x *= s;
  return a;
}
inline Mat<3, 3> skew(const Vec3 &a) {
  Mat<3, 3> s;
  s(0, 1) = -a[2];
  s(0, 2) = a[1];
  s(1, 0) = a[2];
  s(1, 2) = -a[0];
  s(2, 0) = -a[1];
  s(2, 1) = a[0];
  return s;
}
// 小尺寸正定矩阵 Cholesky；失败时不修改调用者的导航状态。
template <std::size_t N> bool cholesky(const Mat<N, N> &a, Mat<N, N> &l) {
  l = {};
  if (!finite(a))
    return false;
  for (std::size_t i = 0; i < N; ++i)
    for (std::size_t j = 0; j <= i; ++j) {
      if (std::abs(a(i, j) - a(j, i)) >
          1e-9 * std::max({1.0, std::abs(a(i, j)), std::abs(a(j, i))}))
        return false;
      double s = a(i, j);
      for (std::size_t k = 0; k < j; ++k)
        s -= l(i, k) * l(j, k);
      if (i == j) {
        if (!(s > 1e-15))
          return false;
        l(i, j) = std::sqrt(s);
      } else
        l(i, j) = s / l(j, j);
    }
  return true;
}
template <std::size_t N> bool inverse(const Mat<N, N> &a, Mat<N, N> &out) {
  if (!finite(a))
    return false;
  Mat<N, N> w = a;
  out = identity<N>();
  for (std::size_t c = 0; c < N; ++c) {
    std::size_t pivot = c;
    for (std::size_t r = c + 1; r < N; ++r)
      if (std::abs(w(r, c)) > std::abs(w(pivot, c)))
        pivot = r;
    if (std::abs(w(pivot, c)) < 1e-14)
      return false;
    for (std::size_t j = 0; j < N; ++j) {
      std::swap(w(c, j), w(pivot, j));
      std::swap(out(c, j), out(pivot, j));
    }
    const double s = w(c, c);
    for (std::size_t j = 0; j < N; ++j) {
      w(c, j) /= s;
      out(c, j) /= s;
    }
    for (std::size_t r = 0; r < N; ++r)
      if (r != c) {
        const double k = w(r, c);
        for (std::size_t j = 0; j < N; ++j) {
          w(r, j) -= k * w(c, j);
          out(r, j) -= k * out(c, j);
        }
      }
  }
  return finite(out);
}
inline double wrapAngle(double x) { return std::remainder(x, 2 * pi); }
struct Quaternion {
  double w{1}, x{}, y{}, z{};
  Quaternion normalized() const {
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (!(n > 1e-15) || !std::isfinite(n))
      return {};
    return {w / n, x / n, y / n, z / n};
  }
  Quaternion conjugate() const { return {w, -x, -y, -z}; }
  Quaternion operator*(const Quaternion &b) const {
    return {w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w};
  }
  static Quaternion exp(const Vec3 &a) {
    const double n = norm(a);
    const double s = n < 1e-9 ? 0.5 - n * n / 48 : std::sin(n / 2) / n;
    return Quaternion{std::cos(n / 2), a[0] * s, a[1] * s, a[2] * s}
        .normalized();
  }
  Vec3 log() const {
    auto q = normalized();
    if (q.w < 0) {
      q.w = -q.w;
      q.x = -q.x;
      q.y = -q.y;
      q.z = -q.z;
    }
    const double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    const double k = s < 1e-9 ? 2 : 2 * std::atan2(s, q.w) / s;
    return {{q.x * k, q.y * k, q.z * k}};
  }
  static Quaternion fromEuler(double roll, double pitch, double yaw) {
    return exp({{0, 0, yaw}}) * exp({{0, pitch, 0}}) * exp({{roll, 0, 0}});
  }
  Mat<3, 3> matrix() const {
    const auto q = normalized();
    Mat<3, 3> c;
    c(0, 0) = 1 - 2 * (q.y * q.y + q.z * q.z);
    c(0, 1) = 2 * (q.x * q.y - q.w * q.z);
    c(0, 2) = 2 * (q.x * q.z + q.w * q.y);
    c(1, 0) = 2 * (q.x * q.y + q.w * q.z);
    c(1, 1) = 1 - 2 * (q.x * q.x + q.z * q.z);
    c(1, 2) = 2 * (q.y * q.z - q.w * q.x);
    c(2, 0) = 2 * (q.x * q.z - q.w * q.y);
    c(2, 1) = 2 * (q.y * q.z + q.w * q.x);
    c(2, 2) = 1 - 2 * (q.x * q.x + q.y * q.y);
    return c;
  }
  Vec3 toEuler() const {
    const auto c = matrix();
    return {{std::atan2(c(2, 1), c(2, 2)),
             std::asin(std::clamp(-c(2, 0), -1.0, 1.0)),
             std::atan2(c(1, 0), c(0, 0))}};
  }
};
inline bool finite(const Quaternion &q) {
  const double n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::isfinite(n) && n > 1e-12;
}
inline Quaternion interpolate(const Quaternion &a, Quaternion b, double t) {
  if (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0)
    b = {-b.w, -b.x, -b.y, -b.z};
  return Quaternion{a.w * (1 - t) + b.w * t, a.x * (1 - t) + b.x * t,
                    a.y * (1 - t) + b.y * t, a.z * (1 - t) + b.z * t}
      .normalized();
}
} // namespace nav
