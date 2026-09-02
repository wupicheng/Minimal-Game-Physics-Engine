#pragma once
//==============================================================================
// pe/math/Transform.h
//
// 刚体变换：平移 + 旋转。引擎内部所有"位姿"都用它，而不是 Mat4。
//
//------------------------------------------------------------------------------
// 为什么故意不包含缩放
//------------------------------------------------------------------------------
// 这是一个刻意的、会影响整个引擎的设计决策：
//
//   1. 非均匀缩放会破坏 GJK 的支撑函数。支撑函数依赖"沿方向 d 最远的点"，
//      而缩放会改变方向与距离的关系（d 方向上最远的点，在缩放后的空间里
//      不再是最远的），必须把方向也做逆转置变换才对 —— 复杂且容易出错。
//   2. 缩放会破坏惯量张量。I 是二阶张量，非均匀缩放下的变换规则远比 R*I*R^T
//      复杂，而且缩放后的形状质量分布已经变了，惯量必须重算。
//   3. 缩放让"接触法线"不再垂直于表面（法线要用逆转置矩阵变换），
//      碰撞响应会得到系统性错误的方向。
//
// 结论：需要不同尺寸的物体，就在创建 Shape 时给不同的参数（半径、半长等），
// 而不是缩放变换。渲染层想怎么缩放是渲染层的事，物理这边不认。
//
//------------------------------------------------------------------------------
// 约定
//------------------------------------------------------------------------------
//   变换一个点：  p_world = rotation.Rotate(p_local) + position
//   即"先转再平移"。复合 A * B 表示"先做 B 再做 A"，和矩阵、四元数一致。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Quat.h"
#include "pe/math/Vec3.h"

namespace pe {

struct Transform {
    Vec3 position;
    Quat rotation;

    /// 不初始化（保持平凡类型）。要单位变换请用 Transform::Identity()。
    Transform() = default;

    constexpr Transform(const Vec3& p, const Quat& r) noexcept : position(p), rotation(r) {}

    /// 只有平移。
    explicit constexpr Transform(const Vec3& p) noexcept
        : position(p), rotation(Quat::Identity()) {}

    static constexpr Transform Identity() noexcept {
        return Transform(Vec3::Zero(), Quat::Identity());
    }

    //--------------------------------------------------------------------------
    // 点与方向的变换
    //
    // 点和方向必须分开处理：点受平移影响，方向不受。把方向当点来变换是
    // 引擎里最常见的一类 bug（表现为法线随物体位置漂移）。
    //--------------------------------------------------------------------------

    /// 局部空间的点 -> 世界空间。
    Vec3 TransformPoint(const Vec3& localPoint) const noexcept {
        return rotation.Rotate(localPoint) + position;
    }

    /// 局部空间的方向（法线、轴）-> 世界空间。只旋转，不平移。
    Vec3 TransformDirection(const Vec3& localDir) const noexcept {
        return rotation.Rotate(localDir);
    }

    /// 世界空间的点 -> 局部空间。
    /// 先减平移再用逆旋转，顺序与正变换相反。
    Vec3 InverseTransformPoint(const Vec3& worldPoint) const noexcept {
        return rotation.RotateInverse(worldPoint - position);
    }

    /// 世界空间的方向 -> 局部空间。
    Vec3 InverseTransformDirection(const Vec3& worldDir) const noexcept {
        return rotation.RotateInverse(worldDir);
    }

    //--------------------------------------------------------------------------
    // 复合与求逆
    //--------------------------------------------------------------------------

    /// 逆变换，满足 t.Inverse().TransformPoint(t.TransformPoint(p)) == p。
    ///
    /// 推导：正变换是 p' = R*p + T，反解得 p = R^-1 * (p' - T) = R^-1*p' + (-R^-1*T)，
    /// 所以逆变换的旋转是 R^-1（单位四元数即共轭），平移是 -R^-1 * T。
    Transform Inverse() const noexcept {
        const Quat invRot = rotation.Conjugate();
        return Transform(-invRot.Rotate(position), invRot);
    }

    /// 复合：this * child，表示"先施加 child，再施加 this"。
    ///
    /// 典型用法是把碰撞体的局部偏移变换到世界空间：
    ///     worldShapeTransform = bodyTransform * colliderLocalTransform
    Transform operator*(const Transform& child) const noexcept {
        return Transform(TransformPoint(child.position), rotation * child.rotation);
    }

    //--------------------------------------------------------------------------
    // 插值
    //--------------------------------------------------------------------------

    /// 位置线性插值 + 姿态球面插值。
    ///
    /// 这是"固定步长物理 + 可变帧率渲染"的关键：物理以 1/60 秒的固定步长推进，
    /// 而渲染帧可能落在两个物理步之间。直接拿最新的物理状态去渲染，会因为
    /// 帧率与物理步长不同步而产生规律性的位置跳动（表现为轻微但恼人的抖动）。
    /// 正确做法是保留上一步的状态，用 alpha = 累加器余量 / 固定步长 做插值。
    static Transform Interpolate(const Transform& prev, const Transform& curr,
                                 real alpha) noexcept {
        return Transform(Lerp(prev.position, curr.position, alpha),
                         Quat::Slerp(prev.rotation, curr.rotation, alpha));
    }
};

inline bool NearlyEqual(const Transform& a, const Transform& b,
                        real eps = kEpsilon) noexcept {
    return NearlyEqual(a.position, b.position, eps) &&
           NearlyEqual(a.rotation, b.rotation, eps);
}

}  // namespace pe
