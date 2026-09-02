#pragma once
//==============================================================================
// pe/math/Mat4.h
//
// 4x4 仿射矩阵。**物理内部不用它** —— 它只存在于引擎与渲染层的边界上。
//
// 物理内部一律用 Transform（位置 + 四元数）：更小、插值稳定、求逆便宜。
// 但渲染 API（OpenGL / D3D / 你现有的渲染层）要的是 4x4 矩阵，所以在
// 每帧末尾用 Mat4::FromTransform() 转换一次。
//
// 存储约定与 Mat3 一致：行优先，列向量右乘（v' = M * v）。
// 平移量在**第 4 列**（rows[0].w, rows[1].w, rows[2].w）。
//
//     | R00 R01 R02 Tx |
//     | R10 R11 R12 Ty |
//     | R20 R21 R22 Tz |
//     |  0   0   0   1 |
//
// 如果你的渲染层用的是 OpenGL/glm 那套（列优先存储），
// 请用 ToColumnMajorArray() 输出，它已经做好了转置。
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/Mat3.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec3.h"
#include "pe/math/Vec4.h"

namespace pe {

struct Mat4 {
    Vec4 rows[4];

    Mat4() = default;

    constexpr Mat4(const Vec4& r0, const Vec4& r1, const Vec4& r2, const Vec4& r3) noexcept
        : rows{r0, r1, r2, r3} {}

    static constexpr Mat4 Identity() noexcept {
        return Mat4(Vec4(1, 0, 0, 0), Vec4(0, 1, 0, 0), Vec4(0, 0, 1, 0), Vec4(0, 0, 0, 1));
    }

    static constexpr Mat4 Zero() noexcept {
        return Mat4(Vec4::Zero(), Vec4::Zero(), Vec4::Zero(), Vec4::Zero());
    }

    /// 由 3x3 旋转 + 平移组装。
    static constexpr Mat4 FromRotationTranslation(const Mat3& r, const Vec3& t) noexcept {
        return Mat4(Vec4(r.rows[0], t.x),
                    Vec4(r.rows[1], t.y),
                    Vec4(r.rows[2], t.z),
                    Vec4(0, 0, 0, 1));
    }

    /// 从物理的 Transform 生成渲染矩阵。每帧对每个可见刚体调用一次。
    static Mat4 FromTransform(const Transform& t) noexcept {
        return FromRotationTranslation(t.rotation.ToMat3(), t.position);
    }

    static Mat4 FromTranslation(const Vec3& t) noexcept {
        return FromRotationTranslation(Mat3::Identity(), t);
    }

    constexpr real& At(int row, int col) noexcept { return rows[row][col]; }
    constexpr real At(int row, int col) const noexcept { return rows[row][col]; }

    /// 取左上角 3x3（旋转部分）。
    constexpr Mat3 UpperLeft3x3() const noexcept {
        return Mat3(rows[0].XYZ(), rows[1].XYZ(), rows[2].XYZ());
    }

    /// 取平移部分（第 4 列的前三个分量）。
    constexpr Vec3 Translation() const noexcept {
        return Vec3(rows[0].w, rows[1].w, rows[2].w);
    }

    //--------------------------------------------------------------------------
    // 变换点与方向
    //--------------------------------------------------------------------------

    /// 变换一个点（隐含 w = 1，因此会被平移）。
    constexpr Vec3 TransformPoint(const Vec3& p) const noexcept {
        return Vec3(Dot(rows[0].XYZ(), p) + rows[0].w,
                    Dot(rows[1].XYZ(), p) + rows[1].w,
                    Dot(rows[2].XYZ(), p) + rows[2].w);
    }

    /// 变换一个方向（隐含 w = 0，因此不受平移影响）。
    constexpr Vec3 TransformDirection(const Vec3& d) const noexcept {
        return Vec3(Dot(rows[0].XYZ(), d), Dot(rows[1].XYZ(), d), Dot(rows[2].XYZ(), d));
    }

    //--------------------------------------------------------------------------
    // 与渲染层交换数据
    //--------------------------------------------------------------------------

    /// 按**列优先**写出 16 个 float —— 这正是 OpenGL 的 glUniformMatrix4fv
    /// （transpose 参数传 GL_FALSE 时）和 glm::mat4 期望的内存布局。
    ///
    /// 转置发生在这里：out[col*4 + row] = At(row, col)。
    void ToColumnMajorArray(float out[16]) const noexcept {
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                out[col * 4 + row] = static_cast<float>(At(row, col));
            }
        }
    }

    /// 按行优先写出 16 个 float（D3D 风格 / 直接镜像内部存储）。
    void ToRowMajorArray(float out[16]) const noexcept {
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                out[row * 4 + col] = static_cast<float>(At(row, col));
            }
        }
    }

    //--------------------------------------------------------------------------
    // 求逆（仅限刚体变换）
    //--------------------------------------------------------------------------

    /// 刚体变换（旋转 + 平移，无缩放无剪切）的快速求逆。
    ///
    /// 利用正交性：R^-1 == R^T，于是
    ///     M^-1 = [ R^T | -R^T * T ]
    /// 完全不需要通用的 4x4 求逆（那要算 4x4 行列式，慢且数值更差）。
    ///
    /// 前提：这个矩阵必须是由 FromTransform 之类构造的刚体变换。
    /// 对含缩放/投影的矩阵调用它会得到错误结果 —— 引擎内部不产生那种矩阵。
    Mat4 InverseRigid() const noexcept {
        const Mat3 rt = UpperLeft3x3().Transposed();
        const Vec3 t = Translation();
        return FromRotationTranslation(rt, -(rt * t));
    }
};

inline constexpr Mat4 operator*(const Mat4& a, const Mat4& b) noexcept {
    Mat4 result = Mat4::Zero();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            real sum = real(0);
            for (int k = 0; k < 4; ++k) {
                sum += a.rows[i][k] * b.rows[k][j];
            }
            result.rows[i][j] = sum;
        }
    }
    return result;
}

inline constexpr Vec4 operator*(const Mat4& m, const Vec4& v) noexcept {
    return Vec4(Dot(m.rows[0], v), Dot(m.rows[1], v), Dot(m.rows[2], v), Dot(m.rows[3], v));
}

inline constexpr bool NearlyEqual(const Mat4& a, const Mat4& b,
                                  real eps = kEpsilon) noexcept {
    return NearlyEqual(a.rows[0], b.rows[0], eps) &&
           NearlyEqual(a.rows[1], b.rows[1], eps) &&
           NearlyEqual(a.rows[2], b.rows[2], eps) &&
           NearlyEqual(a.rows[3], b.rows[3], eps);
}

}  // namespace pe
