#pragma once
//==============================================================================
// pe/math/Vec4.h
//
// 四维向量。物理计算本身**不用**它 —— 它只有两个职责：
//   1. 作为 Mat4 的行，用于和渲染层交换变换矩阵
//   2. 齐次坐标下的平面表示 (nx, ny, nz, d)，即 n·p + d = 0
//      （凸包的面、视锥裁剪都用这个形式）
//
// 不要拿 Vec4 做物理运算 —— 位置、速度、力一律用 Vec3。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Vec3.h"

namespace pe {

struct Vec4 {
    real x, y, z, w;

    Vec4() = default;
    constexpr Vec4(real x_, real y_, real z_, real w_) noexcept
        : x(x_), y(y_), z(z_), w(w_) {}

    /// 由 Vec3 加一个 w 分量构造。点用 w=1，方向用 w=0 —— 这样乘以仿射矩阵时
    /// 点会被平移而方向不会，这正是齐次坐标的意义。
    constexpr Vec4(const Vec3& v, real w_) noexcept : x(v.x), y(v.y), z(v.z), w(w_) {}

    static constexpr Vec4 Zero() noexcept { return Vec4(0, 0, 0, 0); }

    constexpr real& operator[](int i) noexcept { return (&x)[i]; }
    constexpr const real& operator[](int i) const noexcept { return (&x)[i]; }

    /// 丢掉 w，取前三个分量。
    constexpr Vec3 XYZ() const noexcept { return Vec3(x, y, z); }

    constexpr Vec4 operator-() const noexcept { return Vec4(-x, -y, -z, -w); }
    constexpr Vec4& operator+=(const Vec4& v) noexcept {
        x += v.x; y += v.y; z += v.z; w += v.w; return *this;
    }
    constexpr Vec4& operator*=(real s) noexcept {
        x *= s; y *= s; z *= s; w *= s; return *this;
    }
};

inline constexpr Vec4 operator+(const Vec4& a, const Vec4& b) noexcept {
    return Vec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}
inline constexpr Vec4 operator-(const Vec4& a, const Vec4& b) noexcept {
    return Vec4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}
inline constexpr Vec4 operator*(const Vec4& v, real s) noexcept {
    return Vec4(v.x * s, v.y * s, v.z * s, v.w * s);
}
inline constexpr Vec4 operator*(real s, const Vec4& v) noexcept { return v * s; }

inline constexpr real Dot(const Vec4& a, const Vec4& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline constexpr bool operator==(const Vec4& a, const Vec4& b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
inline constexpr bool operator!=(const Vec4& a, const Vec4& b) noexcept { return !(a == b); }

inline constexpr bool NearlyEqual(const Vec4& a, const Vec4& b,
                                  real eps = kEpsilon) noexcept {
    return NearlyEqual(a.x, b.x, eps) && NearlyEqual(a.y, b.y, eps) &&
           NearlyEqual(a.z, b.z, eps) && NearlyEqual(a.w, b.w, eps);
}

}  // namespace pe
