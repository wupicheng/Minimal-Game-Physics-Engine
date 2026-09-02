#pragma once
//==============================================================================
// pe/collision/AABB.h
//
// 轴对齐包围盒（Axis-Aligned Bounding Box）。
//
// 它在引擎里有三个完全不同的用途，别混淆：
//   1. **宽相位的代理**：每个碰撞体算一个世界 AABB 塞进均匀网格。
//      AABB 重叠只是"可能碰撞"的粗筛，真正判定交给窄相位。
//   2. **碰撞形状本身**：静态地图的墙体、箱子可以直接用 AABB 做碰撞形状
//      （不旋转的话比 OBB 快得多）。
//   3. **射线查询的加速**：先测射线与 AABB，不中就跳过昂贵的精确求交。
//
// AABB 永远在世界轴向上，所以它对旋转不敏感 —— 物体一转，它的世界 AABB 就要
// 重算并且会变大（"膨胀"）。这是 AABB 的固有代价，换来的是重叠判定只要 6 次比较。
//==============================================================================

#include <limits>

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Vec3.h"

namespace pe {

struct AABB {
    Vec3 min;
    Vec3 max;

    /// 不初始化（保持平凡类型）。要一个可用的初值请用 Invalid() 或 FromPoints()。
    AABB() = default;

    constexpr AABB(const Vec3& mn, const Vec3& mx) noexcept : min(mn), max(mx) {}

    //--------------------------------------------------------------------------
    // 构造
    //--------------------------------------------------------------------------

    /// "空"包围盒：min 取 +huge，max 取 -huge。
    ///
    /// 这个反直觉的初值是为了配合 Include() 做累积：任何一个点被 Include 进来，
    /// 都会同时把 min 拉小、把 max 拉大，于是第一个点之后盒子就正确了。
    /// 若初值取 (0,0,0)-(0,0,0)，累积出的盒子会永远错误地包含原点。
    ///
    /// 用 numeric_limits::max() 而不是 infinity：inf 参与减法会得到 inf，
    /// 而 inf - inf = NaN，一旦有人对空盒子调用 Size() 就会污染下游。
    static AABB Invalid() noexcept {
        const real huge = std::numeric_limits<real>::max();
        return AABB(Vec3(huge, huge, huge), Vec3(-huge, -huge, -huge));
    }

    static constexpr AABB FromCenterHalfExtents(const Vec3& center,
                                                const Vec3& halfExtents) noexcept {
        return AABB(center - halfExtents, center + halfExtents);
    }

    static constexpr AABB FromMinMax(const Vec3& a, const Vec3& b) noexcept {
        return AABB(MinPerComponent(a, b), MaxPerComponent(a, b));
    }

    /// 包住一组点的最小 AABB。凸包形状算局部 AABB 时用得上。
    static AABB FromPoints(const Vec3* points, std::size_t count) noexcept {
        AABB box = Invalid();
        for (std::size_t i = 0; i < count; ++i) {
            box.Include(points[i]);
        }
        return box;
    }

    //--------------------------------------------------------------------------
    // 查询
    //--------------------------------------------------------------------------

    constexpr Vec3 Center() const noexcept { return (min + max) * real(0.5); }

    /// 半尺寸（从中心到面的距离）。注意和 Size() 差两倍，别用混。
    constexpr Vec3 HalfExtents() const noexcept { return (max - min) * real(0.5); }

    /// 全尺寸（宽高深）。
    constexpr Vec3 Size() const noexcept { return max - min; }

    constexpr real Volume() const noexcept {
        const Vec3 s = Size();
        return s.x * s.y * s.z;
    }

    /// 表面积。现在没人用，但 BVH 的 SAH（表面积启发式）建树完全依赖它 ——
    /// 这是为 UPGRADE_NOTES 里"均匀网格升级为 BVH"提前留的接口。
    constexpr real SurfaceArea() const noexcept {
        const Vec3 s = Size();
        return real(2) * (s.x * s.y + s.y * s.z + s.z * s.x);
    }

    /// 是不是一个有意义的盒子（min 各分量都不大于 max）。
    /// Invalid() 返回的空盒子在这里会得到 false。
    constexpr bool IsValid() const noexcept {
        return min.x <= max.x && min.y <= max.y && min.z <= max.z;
    }

    //--------------------------------------------------------------------------
    // 布尔判定
    //
    // 全部用**闭区间**语义：刚好贴面算相交/包含。
    // 宽相位宁可多报一对（窄相位会否掉），也不能漏报。
    //--------------------------------------------------------------------------

    constexpr bool Contains(const Vec3& p) const noexcept {
        return p.x >= min.x && p.x <= max.x &&
               p.y >= min.y && p.y <= max.y &&
               p.z >= min.z && p.z <= max.z;
    }

    constexpr bool Contains(const AABB& other) const noexcept {
        return other.min.x >= min.x && other.max.x <= max.x &&
               other.min.y >= min.y && other.max.y <= max.y &&
               other.min.z >= min.z && other.max.z <= max.z;
    }

    /// 分离轴判定的最简形式：两个 AABB 不相交，当且仅当存在某个坐标轴，
    /// 它们在该轴上的投影区间不重叠。所以只要有一个轴分离就返回 false。
    constexpr bool Overlaps(const AABB& other) const noexcept {
        if (max.x < other.min.x || min.x > other.max.x) return false;
        if (max.y < other.min.y || min.y > other.max.y) return false;
        if (max.z < other.min.z || min.z > other.max.z) return false;
        return true;
    }

    //--------------------------------------------------------------------------
    // 修改
    //--------------------------------------------------------------------------

    constexpr void Include(const Vec3& p) noexcept {
        min = MinPerComponent(min, p);
        max = MaxPerComponent(max, p);
    }

    constexpr void Include(const AABB& other) noexcept {
        min = MinPerComponent(min, other.min);
        max = MaxPerComponent(max, other.max);
    }

    /// 各方向外扩 margin。宽相位的 "fat AABB" 就靠它：
    /// 物体在膨胀壳内小幅移动时不需要重建空间结构。
    constexpr void Expand(real margin) noexcept {
        const Vec3 m(margin, margin, margin);
        min -= m;
        max += m;
    }

    constexpr AABB Expanded(real margin) const noexcept {
        AABB result = *this;
        result.Expand(margin);
        return result;
    }

    //--------------------------------------------------------------------------
    // 距离
    //--------------------------------------------------------------------------

    /// 盒内的、离 p 最近的点。p 在盒内时返回 p 自身。
    /// 用途：球/胶囊 对 盒 的窄相位（M4）就是从这个函数出发的。
    constexpr Vec3 ClosestPoint(const Vec3& p) const noexcept {
        return Vec3(Clamp(p.x, min.x, max.x),
                    Clamp(p.y, min.y, max.y),
                    Clamp(p.z, min.z, max.z));
    }

    /// 点到盒子的距离平方（点在盒内为 0）。开方留给调用方决定要不要做。
    constexpr real DistanceSq(const Vec3& p) const noexcept {
        return DistanceSq_(p);
    }

private:
    constexpr real DistanceSq_(const Vec3& p) const noexcept {
        real sum = real(0);
        for (int i = 0; i < 3; ++i) {
            // 只有落在区间外的那些轴才贡献距离
            const real v = p[i];
            if (v < min[i]) {
                const real d = min[i] - v;
                sum += d * d;
            } else if (v > max[i]) {
                const real d = v - max[i];
                sum += d * d;
            }
        }
        return sum;
    }
};

//------------------------------------------------------------------------------
// 自由函数
//------------------------------------------------------------------------------

inline constexpr AABB Merge(const AABB& a, const AABB& b) noexcept {
    return AABB(MinPerComponent(a.min, b.min), MaxPerComponent(a.max, b.max));
}

/// 把 AABB 沿位移 d 扫掠，得到覆盖整个运动轨迹的 AABB。
///
/// 现在还没人调用，但它是 CCD（连续碰撞检测）和"运动感知的 fat AABB"的基础：
/// 用扫掠盒做宽相位，高速物体就不会因为两帧之间跳过障碍而漏掉候选对。
/// 见 UPGRADE_NOTES 的 CCD 一节。
inline constexpr AABB Sweep(const AABB& box, const Vec3& displacement) noexcept {
    AABB result = box;
    result.Include(AABB(box.min + displacement, box.max + displacement));
    return result;
}

inline bool NearlyEqual(const AABB& a, const AABB& b, real eps = kEpsilon) noexcept {
    return NearlyEqual(a.min, b.min, eps) && NearlyEqual(a.max, b.max, eps);
}

}  // namespace pe
