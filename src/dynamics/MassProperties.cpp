//==============================================================================
// src/dynamics/MassProperties.cpp
//
// 各形状的转动惯量。每个公式都给出推导，因为这些系数（2/5、1/12、3/8…）
// 单看数字完全无法验证对错，而错了之后的表现是"物体翻滚起来感觉怪怪的"——
// 一种没人能准确描述、也没人知道该怎么查的 bug。
//
// 测试那边用**数值积分**（在形状里撒几十万个点求和）独立验证这些解析公式，
// 那是唯一真正靠得住的检查手段。
//==============================================================================

#include "pe/dynamics/MassProperties.h"

namespace pe {

Mat3 MassProperties::InverseInertia() const noexcept { return inertia.Inverse(); }

MassProperties MassProperties::WithMass(real newMass) const noexcept {
    MassProperties result = *this;
    if (mass > real(0) && newMass > real(0)) {
        // I 与质量成正比（I = ∫ r^2 dm），所以同比例缩放
        const real scale = newMass / mass;
        result.inertia = inertia * scale;
        result.mass = newMass;
    } else {
        result.mass = Max(real(0), newMass);
        result.inertia = Mat3::Diagonal(Vec3::Zero());
    }
    return result;
}

//==============================================================================
// 按形状计算
//==============================================================================

MassProperties ComputeMassProperties(const Shape& shape, real density) noexcept {
    MassProperties out;
    out.centerOfMass = Vec3::Zero();  // 形状的局部原点就是几何中心（见 Shape.h）
    out.mass = real(0);
    out.inertia = Mat3::Diagonal(Vec3::Zero());

    if (density <= real(0) || !shape.IsValid()) return out;

    switch (shape.type) {
        case ShapeType::Sphere: {
            //------------------------------------------------------------------
            // 实心球：I = (2/5) * m * r^2，三根轴相同（各向同性）。
            //
            // 推导：把球切成垂直于 z 轴的薄圆盘，每片半径 sqrt(r^2 - z^2)，
            // 圆盘绕自身轴的惯量是 (1/2) dm R^2，沿 z 积分即得。
            //------------------------------------------------------------------
            const real r = shape.sphere.radius;
            out.mass = density * shape.Volume();
            const real i = (real(2) / real(5)) * out.mass * r * r;
            out.inertia = Mat3::Diagonal(Vec3(i, i, i));
            break;
        }

        case ShapeType::Box: {
            //------------------------------------------------------------------
            // 长方体，边长 (2hx, 2hy, 2hz)：
            //     I_xx = (1/12) * m * ((2hy)^2 + (2hz)^2) = (1/3) * m * (hy^2 + hz^2)
            //
            // 注意每根轴的惯量由**另外两根**轴的尺寸决定 —— 绕 X 转时，
            // 质量离 X 轴的距离由 y、z 决定，和 x 方向多长毫无关系。
            // 把下标写错是这个公式最经典的错误，症状是细长盒子的翻滚方向不对。
            //------------------------------------------------------------------
            const Vec3& h = shape.box.halfExtents;
            out.mass = density * shape.Volume();
            const real k = out.mass / real(3);
            out.inertia = Mat3::Diagonal(Vec3(k * (h.y * h.y + h.z * h.z),
                                              k * (h.x * h.x + h.z * h.z),
                                              k * (h.x * h.x + h.y * h.y)));
            break;
        }

        case ShapeType::Capsule: {
            //------------------------------------------------------------------
            // 胶囊 = 圆柱段 + 两个半球端盖，轴沿局部 +Y。
            // 分别算再相加，端盖要用平行轴定理搬到胶囊中心。
            //
            // 记 r = 半径，hh = 圆柱段半长，hc = 2*hh = 圆柱段全高。
            //
            // 圆柱（质量 mc）：
            //     绕轴向 Y ：I = (1/2) * mc * r^2
            //     绕横向 X/Z：I = mc * (hc^2/12 + r^2/4)
            //
            // 每个半球（质量 mh）：
            //     绕轴向 Y ：I = (2/5) * mh * r^2
            //         整球绕直径是 (2/5) M r^2，沿赤道面切成两半，
            //         每半的质量减半、惯量也减半，比值不变。
            //         而且 Y 轴穿过半球质心，不需要平行轴修正。
            //     绕横向 X/Z：先写出"绕平面圆心"的惯量 (2/5) mh r^2，
            //         半球质心距平面圆心 3r/8，先退回质心再搬到胶囊中心：
            //             I = (2/5)mh r^2 - mh*(3r/8)^2 + mh*(hh + 3r/8)^2
            //               = (2/5)mh r^2 + mh*hh^2 + mh*(3*hh*r/4)
            //         那两个 (3r/8)^2 正负抵消掉了 —— 这正是必须"先退回质心"
            //         的原因：直接用 I_平面圆心 + m*d^2 会多算一项，是常见错误。
            //------------------------------------------------------------------
            const real r = shape.capsule.radius;
            const real hh = shape.capsule.halfHeight;

            const real cylinderVolume = kPi * r * r * (real(2) * hh);
            const real hemisphereVolume = (real(2) / real(3)) * kPi * r * r * r;

            const real mc = density * cylinderVolume;
            const real mh = density * hemisphereVolume;  // 单个半球

            out.mass = mc + real(2) * mh;

            // 轴向（Y）
            const real axial =
                real(0.5) * mc * r * r + real(2) * (real(2) / real(5)) * mh * r * r;

            // 横向（X、Z）
            const real hc = real(2) * hh;
            const real cylTransverse = mc * (hc * hc / real(12) + r * r / real(4));
            const real capTransverse =
                (real(2) / real(5)) * mh * r * r + mh * hh * hh +
                mh * (real(3) * hh * r / real(4));
            const real transverse = cylTransverse + real(2) * capTransverse;

            out.inertia = Mat3::Diagonal(Vec3(transverse, axial, transverse));
            break;
        }
    }

    return out;
}

MassProperties ComputeMassPropertiesFromMass(const Shape& shape, real mass) noexcept {
    // 先用单位密度算一遍，再按比例缩到目标质量。
    // 这样每种形状的公式只需要写一份。
    const MassProperties unit = ComputeMassProperties(shape, real(1));
    if (unit.mass <= real(0) || mass <= real(0)) {
        MassProperties out;
        out.mass = real(0);
        out.centerOfMass = Vec3::Zero();
        out.inertia = Mat3::Diagonal(Vec3::Zero());
        return out;
    }
    return unit.WithMass(mass);
}

//==============================================================================
// 坐标变换
//==============================================================================

Mat3 TranslateInertia(const Mat3& inertiaAboutCom, real mass,
                      const Vec3& offset) noexcept {
    // I_new = I_com + m * (|d|^2 * E - d ⊗ d)
    //
    // 括号里那一项在 d 方向上恒为零：把 d 代进去，
    // (|d|^2 E - d⊗d) * d = |d|^2 d - d (d·d) = 0。
    // 也就是说"绕平移方向自身那根轴"的转动惯量不变 —— 物理上显然，
    // 但只有写成张量形式才能自动满足。
    const real d2 = offset.LengthSq();

    Mat3 result = inertiaAboutCom;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const real kronecker = (i == j) ? real(1) : real(0);
            result.rows[i][j] += mass * (d2 * kronecker - offset[i] * offset[j]);
        }
    }
    return result;
}

Mat3 RotateInertia(const Mat3& inertia, const Mat3& rotation) noexcept {
    // I' = R * I * R^T
    //
    // 两边都要乘：惯量张量把角速度映射到角动量（L = I*w），两边都是向量，
    // 所以换基时输入要先转回旧基（R^T）、输出再转到新基（R）。
    // 只乘一个 R 得到的矩阵不再对称，也不再有物理意义。
    return rotation * inertia * rotation.Transposed();
}

}  // namespace pe
