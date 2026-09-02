#pragma once
//==============================================================================
// pe/math/Mat3.h
//
// 3x3 矩阵。物理引擎里它有两个身份，两个都很重要：
//   1. 旋转矩阵 R（正交阵，R^-1 == R^T）
//   2. 转动惯量张量 I（对称阵，描述刚体绕各轴转动的"惯性"）
//
//==============================================================================
// 存储与乘法约定（**整个引擎唯一约定，和渲染层对接时必须核对这一段**）
//==============================================================================
//
//   - 行优先存储：rows[i] 是第 i 行。元素 m(i,j) = rows[i][j]，i 是行、j 是列。
//   - 列向量右乘：v' = M * v，展开为 v'[i] = sum_j m(i,j) * v[j]
//   - 复合变换从右往左读：(A * B) * v 等价于 A * (B * v)，即先做 B 再做 A。
//
//   如果你的渲染层是列优先 + 行向量左乘（例如某些 D3D 风格的代码），
//   那么把这里的矩阵传过去之前需要转置。Mat4::ToColumnMajorArray() 已经
//   帮你做了 OpenGL/glm 需要的那种转置。
//
//==============================================================================

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Vec3.h"

namespace pe {

struct Mat3 {
    /// rows[i] 是第 i 行。用 Vec3 存行，是因为 M*v 的每个分量恰好是"行 · v"，
    /// 写出来就是 Dot(rows[i], v)，可读性远好于双下标循环。
    Vec3 rows[3];

    Mat3() = default;

    constexpr Mat3(const Vec3& row0, const Vec3& row1, const Vec3& row2) noexcept
        : rows{row0, row1, row2} {}

    /// 按行展开的九个元素，m(行, 列)。
    constexpr Mat3(real m00, real m01, real m02,
                   real m10, real m11, real m12,
                   real m20, real m21, real m22) noexcept
        : rows{Vec3(m00, m01, m02), Vec3(m10, m11, m12), Vec3(m20, m21, m22)} {}

    //--------------------------------------------------------------------------
    // 构造
    //--------------------------------------------------------------------------

    static constexpr Mat3 Zero() noexcept {
        return Mat3(Vec3(0, 0, 0), Vec3(0, 0, 0), Vec3(0, 0, 0));
    }

    static constexpr Mat3 Identity() noexcept {
        return Mat3(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1));
    }

    /// 对角阵。惯量张量在物体的主轴坐标系下就是对角阵，所以这个构造用得很多。
    static constexpr Mat3 Diagonal(const Vec3& d) noexcept {
        return Mat3(Vec3(d.x, 0, 0), Vec3(0, d.y, 0), Vec3(0, 0, d.z));
    }

    static constexpr Mat3 FromRows(const Vec3& r0, const Vec3& r1, const Vec3& r2) noexcept {
        return Mat3(r0, r1, r2);
    }

    /// 用三个列向量构造。旋转矩阵常常是"把局部坐标轴写成列"得到的：
    /// 若 (ex, ey, ez) 是局部坐标轴在世界空间的表示，则 R = FromColumns(ex, ey, ez)
    /// 满足 R * (1,0,0) == ex，即把局部向量变换到世界空间。
    static constexpr Mat3 FromColumns(const Vec3& c0, const Vec3& c1, const Vec3& c2) noexcept {
        return Mat3(Vec3(c0.x, c1.x, c2.x),
                    Vec3(c0.y, c1.y, c2.y),
                    Vec3(c0.z, c1.z, c2.z));
    }

    /// 反对称矩阵（叉积矩阵）[v]x，满足 [v]x * w == Cross(v, w)。
    ///
    ///          |  0   -vz   vy |
    ///  [v]x =  |  vz   0   -vx |
    ///          | -vy   vx   0  |
    ///
    /// 为什么需要它：约束求解里经常出现 "r × (I^-1 * (r × n))" 这样的嵌套叉积。
    /// 把叉积写成矩阵之后，整串运算可以合并成单个 3x3 矩阵，推导有效质量矩阵
    /// （K 矩阵）时必须用到这个形式。
    static constexpr Mat3 Skew(const Vec3& v) noexcept {
        return Mat3(Vec3(real(0), -v.z, v.y),
                    Vec3(v.z, real(0), -v.x),
                    Vec3(-v.y, v.x, real(0)));
    }

    /// 外积（张量积）a ⊗ b，元素为 a[i]*b[j]。
    /// 惯量张量的平行轴定理里会用到：I_new = I_cm + m * (|d|^2 * E - d ⊗ d)
    static constexpr Mat3 Outer(const Vec3& a, const Vec3& b) noexcept {
        return Mat3(b * a.x, b * a.y, b * a.z);
    }

    //--------------------------------------------------------------------------
    // 元素访问
    //--------------------------------------------------------------------------

    constexpr real& At(int row, int col) noexcept { return rows[row][col]; }
    constexpr real At(int row, int col) const noexcept { return rows[row][col]; }

    constexpr Vec3 Row(int i) const noexcept { return rows[i]; }
    constexpr Vec3 Column(int j) const noexcept {
        return Vec3(rows[0][j], rows[1][j], rows[2][j]);
    }

    //--------------------------------------------------------------------------
    // 运算
    //--------------------------------------------------------------------------

    constexpr Mat3 Transposed() const noexcept {
        return Mat3(Column(0), Column(1), Column(2));
    }

    /// 行列式。几何含义是变换对体积的缩放倍率；为 0 表示矩阵把空间压扁了，不可逆。
    constexpr real Determinant() const noexcept {
        // 沿第一行做代数余子式展开
        return rows[0].x * (rows[1].y * rows[2].z - rows[1].z * rows[2].y) -
               rows[0].y * (rows[1].x * rows[2].z - rows[1].z * rows[2].x) +
               rows[0].z * (rows[1].x * rows[2].y - rows[1].y * rows[2].x);
    }

    /// 一般逆矩阵：M^-1 = adj(M) / det(M)，adj 是伴随矩阵（代数余子式矩阵的转置）。
    /// 奇异（det 接近 0）时返回零矩阵而不是 inf/NaN —— 零矩阵会让后续的冲量
    /// 计算得到 0，表现为"这个约束不起作用"，比让 NaN 扩散到整个世界安全得多。
    ///
    /// 注意：旋转矩阵求逆**不要**用这个函数，直接用 Transposed()（正交阵的逆
    /// 就是转置），既快又没有数值误差。
    constexpr Mat3 Inverse() const noexcept {
        // 先算出伴随矩阵的三列（也就是代数余子式）
        const Vec3 c0 = Cross(rows[1], rows[2]);
        const Vec3 c1 = Cross(rows[2], rows[0]);
        const Vec3 c2 = Cross(rows[0], rows[1]);

        // det = row0 · (row1 × row2)，即混合积，和上面的展开等价但更简洁
        const real det = Dot(rows[0], c0);
        if (Abs(det) < real(1e-12)) {
            return Zero();
        }
        const real invDet = real(1) / det;

        // adj(M) = cofactor(M)^T。上面的 c0/c1/c2 已经是余子式矩阵的"行"，
        // 所以按列摆放就完成了转置。
        return Mat3(Vec3(c0.x, c1.x, c2.x),
                    Vec3(c0.y, c1.y, c2.y),
                    Vec3(c0.z, c1.z, c2.z)) * invDet;
    }

    /// 对称矩阵求逆的别名。惯量张量总是对称的，语义上标注一下，
    /// 提醒读者这里的输入有额外的结构（将来若要换成 Cholesky 分解从这里下手）。
    constexpr Mat3 InverseSymmetric() const noexcept { return Inverse(); }

    constexpr Mat3 operator*(real s) const noexcept {
        return Mat3(rows[0] * s, rows[1] * s, rows[2] * s);
    }

    constexpr Mat3 operator+(const Mat3& o) const noexcept {
        return Mat3(rows[0] + o.rows[0], rows[1] + o.rows[1], rows[2] + o.rows[2]);
    }
    constexpr Mat3 operator-(const Mat3& o) const noexcept {
        return Mat3(rows[0] - o.rows[0], rows[1] - o.rows[1], rows[2] - o.rows[2]);
    }
};

//------------------------------------------------------------------------------
// 矩阵 * 向量：v' = M * v（v 视为列向量）
//------------------------------------------------------------------------------
inline constexpr Vec3 operator*(const Mat3& m, const Vec3& v) noexcept {
    return Vec3(Dot(m.rows[0], v), Dot(m.rows[1], v), Dot(m.rows[2], v));
}

//------------------------------------------------------------------------------
// 矩阵 * 矩阵：结果 (i,j) = A 的第 i 行 · B 的第 j 列
//------------------------------------------------------------------------------
inline constexpr Mat3 operator*(const Mat3& a, const Mat3& b) noexcept {
    const Vec3 bc0 = b.Column(0);
    const Vec3 bc1 = b.Column(1);
    const Vec3 bc2 = b.Column(2);
    return Mat3(Vec3(Dot(a.rows[0], bc0), Dot(a.rows[0], bc1), Dot(a.rows[0], bc2)),
                Vec3(Dot(a.rows[1], bc0), Dot(a.rows[1], bc1), Dot(a.rows[1], bc2)),
                Vec3(Dot(a.rows[2], bc0), Dot(a.rows[2], bc1), Dot(a.rows[2], bc2)));
}

inline constexpr Mat3 operator*(real s, const Mat3& m) noexcept { return m * s; }

inline constexpr bool NearlyEqual(const Mat3& a, const Mat3& b,
                                  real eps = kEpsilon) noexcept {
    return NearlyEqual(a.rows[0], b.rows[0], eps) &&
           NearlyEqual(a.rows[1], b.rows[1], eps) &&
           NearlyEqual(a.rows[2], b.rows[2], eps);
}

//------------------------------------------------------------------------------
// 惯量张量的坐标变换
//------------------------------------------------------------------------------

/// 把局部空间的惯量张量（或它的逆）变换到世界空间：I_world = R * I_local * R^T
///
/// 为什么是这个形式：惯量张量是二阶张量，不是向量。它描述的是"角速度 -> 角动量"
/// 这个线性映射：L = I * omega。当我们换到世界坐标系，输入的 omega 是世界向量，
/// 要先用 R^T 转回局部（R^T * omega），在局部空间做 I_local 映射，
/// 再用 R 把结果的角动量转回世界。三步合起来就是 R * I_local * R^T。
///
/// 这个函数每帧对每个动态刚体调用一次（刚体转了，惯量在世界空间的表现就变了），
/// 结果缓存在 RigidBody::invInertiaWorld 里，供求解器反复使用。
inline constexpr Mat3 TransformInertia(const Mat3& rotation, const Mat3& inertiaLocal) noexcept {
    return rotation * inertiaLocal * rotation.Transposed();
}

}  // namespace pe
