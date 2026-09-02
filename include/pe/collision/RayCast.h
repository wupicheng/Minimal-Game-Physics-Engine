#pragma once
//==============================================================================
// pe/collision/RayCast.h
//
// 各形状的射线求交（解析闭式解，不迭代）。
//
// 全部遵守 Ray.h 里的两条约定：
//   - direction 是单位向量，所以返回的 distance 就是米
//   - 射线起点在物体内部时返回 true，distance = 0，normal = -direction
//
// 返回 false 时 out 的内容未定义，不要读。
//
//------------------------------------------------------------------------------
// TODO(upgrade): 连续碰撞检测 CCD
//   这里全是"静态形状 vs 射线"。高速**抛射物**（火箭、手雷）如果用离散模拟，
//   一帧移动的距离可能超过墙的厚度，就会穿墙。
//   升级路径是加一组 ShapeCast（形状扫掠）函数：把移动的形状沿位移扫掠，
//   本质上是"闵可夫斯基和之后的射线求交"—— 球扫掠退化成对膨胀后形状的射线，
//   任意凸形状则需要 M5 的 GJK 配合 conservative advancement。
//   接口预留在这里，World 层的 ShapeCast 会调用它们。
//
//   注意 hitscan 子弹（步枪、手枪）不需要 CCD —— 它们本来就是一条射线，
//   没有"两帧之间跳过去"的问题。
//==============================================================================

#include "pe/collision/AABB.h"
#include "pe/collision/Ray.h"
#include "pe/collision/Shape.h"
#include "pe/core/Types.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec3.h"

namespace pe {

//------------------------------------------------------------------------------
// 单个形状
//------------------------------------------------------------------------------

/// 射线 vs 球（球心 + 半径，世界空间）。
bool RaycastSphere(const Vec3& center, real radius, const Ray& ray,
                   RaycastHit& out) noexcept;

/// 射线 vs 轴对齐包围盒。
/// 这个函数有双重身份：既是 AABB 形状的精确求交，也是宽相位里给其它形状
/// 做粗筛的加速手段，所以它值得写得特别快。
bool RaycastAABB(const AABB& box, const Ray& ray, RaycastHit& out) noexcept;

/// 射线 vs 有向包围盒（盒子的位姿 + 半尺寸）。
bool RaycastOBB(const Transform& transform, const Vec3& halfExtents, const Ray& ray,
                RaycastHit& out) noexcept;

/// 射线 vs 胶囊（位姿 + 半径 + 圆柱段半长，轴沿局部 +Y）。
bool RaycastCapsule(const Transform& transform, real radius, real halfHeight,
                    const Ray& ray, RaycastHit& out) noexcept;

//------------------------------------------------------------------------------
// 分派
//------------------------------------------------------------------------------

/// 按形状类型分派到上面对应的函数。World 层的射线查询走这个入口。
bool RaycastShape(const Shape& shape, const Transform& transform, const Ray& ray,
                  RaycastHit& out) noexcept;

}  // namespace pe
