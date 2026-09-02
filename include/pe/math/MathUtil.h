#pragma once
//==============================================================================
// pe/math/MathUtil.h
//
// 标量数学工具。所有向量/矩阵/四元数运算最终都落到这里。
//
// 为什么要包一层而不直接用 <cmath>：
//   1. 统一走 `real` 类型，避免 float/double 之间的隐式转换（换类型时不会漏改）。
//   2. 少数函数需要防御性处理（Acos 的定义域、除零、归一化零向量），
//      集中在这里做一次，比在每个调用点各写一遍可靠得多。
//==============================================================================

#include <cmath>

#include "pe/core/Types.h"

namespace pe {

//------------------------------------------------------------------------------
// 基础比较与钳制（constexpr，编译期可用）
//------------------------------------------------------------------------------

inline constexpr real Abs(real x) noexcept { return x < real(0) ? -x : x; }
inline constexpr real Min(real a, real b) noexcept { return a < b ? a : b; }
inline constexpr real Max(real a, real b) noexcept { return a > b ? a : b; }

inline constexpr real Clamp(real x, real lo, real hi) noexcept {
    return x < lo ? lo : (x > hi ? hi : x);
}

/// 返回 -1 / 0 / +1。注意 x 恰为 0 时返回 0，不返回 +1。
inline constexpr real Sign(real x) noexcept {
    return x > real(0) ? real(1) : (x < real(0) ? real(-1) : real(0));
}

inline constexpr real Square(real x) noexcept { return x * x; }

/// 线性插值。t 不做钳制 —— 外推有时是有意的（例如速度预测）。
inline constexpr real Lerp(real a, real b, real t) noexcept { return a + (b - a) * t; }

/// 绝对容差比较。适合比较接近 0 的量。
/// 注意：比较两个很大的数时绝对容差会过于严格，那种场合应该用相对容差，
/// 但物理引擎里的量级大多在 [1e-3, 1e3]，绝对容差够用且行为可预测。
inline constexpr bool NearlyEqual(real a, real b, real eps = kEpsilon) noexcept {
    return Abs(a - b) <= eps;
}

inline constexpr bool NearlyZero(real a, real eps = kEpsilon) noexcept {
    return Abs(a) <= eps;
}

//------------------------------------------------------------------------------
// 角度
//------------------------------------------------------------------------------

inline constexpr real DegToRad(real deg) noexcept { return deg * (kPi / real(180)); }
inline constexpr real RadToDeg(real rad) noexcept { return rad * (real(180) / kPi); }

//------------------------------------------------------------------------------
// 超越函数（薄封装，保证类型一致）
//------------------------------------------------------------------------------

inline real Sqrt(real x) noexcept { return std::sqrt(x); }
inline real Sin(real x) noexcept { return std::sin(x); }
inline real Cos(real x) noexcept { return std::cos(x); }
inline real Tan(real x) noexcept { return std::tan(x); }
inline real Atan2(real y, real x) noexcept { return std::atan2(y, x); }

/// 安全的 acos：先把参数钳制到 [-1, 1]。
/// 为什么必须钳制：两个单位向量的点积在数学上必然落在 [-1,1]，但浮点误差
/// 会让它变成 1.0000001，直接喂给 std::acos 会返回 NaN，而 NaN 一旦进入
/// 物理状态就会污染整个模拟（位置变 NaN → AABB 变 NaN → 宽相位崩溃）。
/// 这个坑在 Slerp 和"求两向量夹角"里出现频率极高。
inline real SafeAcos(real x) noexcept { return std::acos(Clamp(x, real(-1), real(1))); }
inline real SafeAsin(real x) noexcept { return std::asin(Clamp(x, real(-1), real(1))); }

//------------------------------------------------------------------------------
// 数值健壮性
//------------------------------------------------------------------------------

inline bool IsFinite(real x) noexcept { return std::isfinite(x); }

/// 安全除法：分母接近 0 时返回 fallback，而不是产生 inf/NaN。
inline real SafeDivide(real numerator, real denominator,
                       real fallback = real(0)) noexcept {
    return Abs(denominator) > kEpsilon ? numerator / denominator : fallback;
}

}  // namespace pe
