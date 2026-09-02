#pragma once
//==============================================================================
// pe/collision/Ray.h
//
// 射线与射线命中结果。FPS 里这是最高频的查询：
//   - hitscan 子弹的命中判定（每次开枪至少一条）
//   - AI 视线遮挡（每个 AI 每帧一条到玩家）
//   - 角色控制器的地面探测（每帧一条向下）
//
//------------------------------------------------------------------------------
// 两条硬性约定
//------------------------------------------------------------------------------
// 1. **direction 永远是单位向量**。构造函数会归一化，不要绕过构造函数直接改
//    这个字段（除非你自己保证已归一化，比如把射线变换到局部空间时 —— 旋转
//    保长，所以变换后仍是单位向量）。
//    这条约定的价值：参数 t 直接就是**米为单位的距离**，而不是"多少个方向向量
//    的长度"。比较距离、限制射程、排序命中点全都不需要额外换算。
//
// 2. **起点在物体内部时，返回命中且 distance = 0，法线取 -direction**。
//    另一种常见约定是"起点在内部就算不中"。这里选前者，理由是 FPS 的安全性：
//    玩家贴着墙开枪、枪口模型陷进墙里时，"立即命中"会让子弹停在原地，
//    而"不中"会让子弹穿墙打到墙后的人 —— 后者是能被玩家利用的 bug。
//==============================================================================

#include <limits>

#include "pe/core/Types.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Vec3.h"

namespace pe {

struct Ray {
    Vec3 origin;
    Vec3 direction;    ///< 单位向量（构造函数保证）
    real maxDistance;  ///< 射程上限，单位米

    /// 不初始化。内部代码把射线变换到局部空间时会逐字段赋值，走这个构造。
    Ray() = default;

    /// 会把 dir 归一化。零向量方向退化为 +X 且射程为 0（等于一条不命中任何
    /// 东西的射线），而不是产生 NaN。
    Ray(const Vec3& o, const Vec3& dir,
        real maxDist = std::numeric_limits<real>::max()) noexcept
        : origin(o), direction(dir.Normalized()), maxDistance(maxDist) {
        if (direction.IsZero()) {
            direction = Vec3::UnitX();
            maxDistance = real(0);
        }
    }

    /// 从 a 射向 b，射程恰好是两点距离。
    /// 视线遮挡判定的标准写法：`Ray::FromTo(eyePos, targetPos)`，
    /// 这样目标之后的墙不会误判为遮挡。
    static Ray FromTo(const Vec3& a, const Vec3& b) noexcept {
        const Vec3 delta = b - a;
        const real dist = delta.Length();
        Ray r;
        r.origin = a;
        r.direction = dist > real(1e-6) ? delta * (real(1) / dist) : Vec3::UnitX();
        r.maxDistance = dist;
        return r;
    }

    /// 射线上参数为 t 处的点。因为 direction 是单位向量，t 就是距离。
    Vec3 PointAt(real t) const noexcept { return origin + direction * t; }

    /// 终点（射程上限处）。
    Vec3 End() const noexcept { return PointAt(maxDistance); }
};

//------------------------------------------------------------------------------
// 命中结果
//
// 注意这里**没有** `bool hit` 字段：所有求交函数用返回值表示中没中，
// 命中信息只有在返回 true 时才有效。多存一份 bool 迟早会和返回值不一致。
//
// 也**没有** collider / body 句柄：collision 层按设计不知道刚体的存在
// （见 ARCHITECTURE.md §2）。World 层（M8）会在这个结构外面再包一层，
// 补上 ColliderHandle / BodyHandle。
//------------------------------------------------------------------------------
struct RaycastHit {
    real distance;  ///< 沿 direction 的距离（米）。起点在内部时为 0。
    Vec3 point;     ///< 命中点，世界空间
    Vec3 normal;    ///< 命中处的表面外法线，单位向量，总是朝向射线来的一侧

    RaycastHit() = default;
};

}  // namespace pe
