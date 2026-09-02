#pragma once
//==============================================================================
// pe/collision/NarrowPhase.h
//
// 窄相位：宽相位吐出的候选对进来，接触流形出去。
//
//------------------------------------------------------------------------------
// 双分派与"规范顺序"
//------------------------------------------------------------------------------
// 球-盒 和 盒-球 是同一个几何问题，但如果两边各写一份实现，就有两份代码可以
// 各自写错、各自漂移。这里只实现**规范顺序**的那一半：形状对按 ShapeType 的
// 枚举值从小到大排列（Sphere < Capsule < Box），一共 6 个函数：
//
//     Sphere-Sphere   Sphere-Capsule   Sphere-Box
//                     Capsule-Capsule  Capsule-Box
//                                      Box-Box
//
// Collide() 这个分派入口负责：传入顺序不规范时交换 A、B 调用，然后把法线取反。
// 因为接触点用的是"两个见证点的中点"（见 Manifold.h），交换之后接触点位置完全
// 不变，所以取反法线就是全部需要做的事。
//
// 于是得到一条很强的、可测试的契约：
//
//     Collide(A,B) 与 Collide(B,A) 给出相同的接触点、相同的穿透深度、相反的法线。
//
// 盒-盒是唯一的例外，见下面 CollideBoxBox 的说明。
//
//------------------------------------------------------------------------------
// 为什么是解析解而不是统一走 GJK
//------------------------------------------------------------------------------
// GJK/EPA（M5）能处理任意凸形状，但它是迭代的：几十次支撑点查询、EPA 还要维护
// 一个不断扩张的多面体。而球、胶囊、盒这三种形状占了 FPS 场景的 99%：
// 角色是胶囊、地图是盒、手雷是球。为它们写闭式解，一次几十条浮点指令就出结果，
// 而且没有"迭代不收敛"这种失败模式。
//
// GJK 的价值在于**兜底**：等 M5 加入 ConvexHull 之后，任何涉及凸包的组合都走
// 通用路径。到那时这里的解析解会成为快路径，两条路径的结果要能互相对拍
// —— 那正是 M5 的验收条件。
//
//------------------------------------------------------------------------------
// 推测性接触（speculative contact）
//------------------------------------------------------------------------------
// 窄相位不只在"已经重叠"时才报接触，两个形状相距 kSpeculativeMargin 以内就会
// 报出一个 penetration 为负的接触点。原因和宽相位用 fat AABB 是同一个：
// 求解器的 warm starting 需要接触在真正碰上**之前**就存在，否则每次碰撞的第一帧
// 都要从零收敛，堆叠会肉眼可见地一沉一弹。
//
// 求解器看到负的 penetration 时不会施加分离冲量，只会限制接近速度 ——
// 所以多报这些"即将接触"不会让物体悬空。
//==============================================================================

#include "pe/collision/Manifold.h"
#include "pe/collision/Shape.h"
#include "pe/core/Types.h"
#include "pe/math/Transform.h"

namespace pe {

/// 两个形状表面相距多远以内就开始生成接触点（米）。
///
/// 取 2 倍的 kLinearSlop：比求解器允许的穿透量大一档，保证物体在稳定接触状态下
/// 接触点不会因为浮点噪声而反复"存在—消失"（那会让 warm starting 的缓存每帧失效，
/// 等于没做 warm starting）。
inline constexpr real kSpeculativeMargin = real(2) * kLinearSlop;

//------------------------------------------------------------------------------
// 分派入口
//------------------------------------------------------------------------------

/// 计算两个形状之间的接触流形。
///
/// 返回 true 表示生成了至少一个接触点（含"即将接触"的推测性接触）。
/// 返回 false 时 out 已被 Clear()，可以安全读取（pointCount == 0）。
///
/// A、B 的顺序随意 —— normal 保证由 A 指向 B。
bool Collide(const Shape& a, const Transform& ta, const Shape& b, const Transform& tb,
             Manifold& out) noexcept;

/// 只问"碰没碰"，不生成接触点。比 Collide() 快，因为不用做面裁剪。
/// 用途：触发器的重叠判定、角色控制器的"这个位置能不能站"。
bool Overlap(const Shape& a, const Transform& ta, const Shape& b,
             const Transform& tb) noexcept;

//------------------------------------------------------------------------------
// 各形状对（规范顺序）
//
// 直接调这些函数可以省掉一次分派，但调用方必须自己保证形状类型对得上 ——
// 传错了是未定义行为（debug 下有断言）。不确定就用 Collide()。
//------------------------------------------------------------------------------

bool CollideSphereSphere(const Shape& a, const Transform& ta, const Shape& b,
                         const Transform& tb, Manifold& out) noexcept;

bool CollideSphereCapsule(const Shape& a, const Transform& ta, const Shape& b,
                          const Transform& tb, Manifold& out) noexcept;

bool CollideSphereBox(const Shape& a, const Transform& ta, const Shape& b,
                      const Transform& tb, Manifold& out) noexcept;

bool CollideCapsuleCapsule(const Shape& a, const Transform& ta, const Shape& b,
                           const Transform& tb, Manifold& out) noexcept;

bool CollideCapsuleBox(const Shape& a, const Transform& ta, const Shape& b,
                       const Transform& tb, Manifold& out) noexcept;

/// 盒 vs 盒：SAT（15 根分离轴）+ Sutherland-Hodgman 面裁剪。
///
/// **这是唯一不满足 A/B 交换对称性的一对**：SAT 找到最小穿透轴之后，要在两个盒子
/// 里挑一个当"参考面"，挑选规则带有偏置（优先选面轴而不是边叉积轴，理由见 .cpp），
/// 交换 A、B 有可能选中另一个盒子做参考面，于是裁剪出来的点集不同。
/// 法线和穿透深度仍然是对称的（差一个符号），只有接触点的分布可能不一样。
///
/// 这不是缺陷：两组点描述的是同一块接触区域，求解器对哪一组都能正确工作。
bool CollideBoxBox(const Shape& a, const Transform& ta, const Shape& b,
                   const Transform& tb, Manifold& out) noexcept;

//------------------------------------------------------------------------------
// 通用凸形状路径（M5）
//------------------------------------------------------------------------------

/// GJK + EPA 的通用路径：不看形状类型，只要是凸的就能算。
///
/// 它和上面那些解析解**算的是同一件事**，所以两者可以互相对拍 —— 这正是 M5 的
/// 验收方式：同一对盒子，SAT 那条路和 GJK/EPA 这条路，法线与穿透深度必须一致。
/// 解析解那一侧已经被 M4 的构造用例和随机不变量钉死了，所以它是可信的基准。
///
/// **`Collide()` 默认不走这条路**。原因有两个：
///   1. 解析解快得多，而球/胶囊/盒覆盖了 FPS 场景的 99%；
///   2. 这条路只生成**一个**接触点。要生成完整的面接触流形，需要从 EPA 给出的
///      法线出发做面裁剪，而那需要形状能枚举自己的面 —— 盒子可以，但真正需要
///      通用路径的 ConvexHull 还没有面数据（见文件末尾的说明）。
///
/// 它现在的价值是：给解析解做交叉验证的独立第二实现，以及 ConvexHull 到位之后
/// 立刻可用的兜底路径。
bool CollideConvex(const Shape& a, const Transform& ta, const Shape& b,
                   const Transform& tb, Manifold& out) noexcept;

/// 两个凸形状表面之间的距离（相交时为 0）。
/// outPointA / outPointB 是各自表面上最近的那一对点。
///
/// 这是解析解那一套**给不了**的东西：它们只在"已经接触"时才有输出。
/// 距离查询是角色控制器（"前面还有多远"）和 AI（"够不够得着"）需要的。
real ConvexDistance(const Shape& a, const Transform& ta, const Shape& b,
                    const Transform& tb, Vec3& outPointA, Vec3& outPointB) noexcept;

//------------------------------------------------------------------------------
// TODO(upgrade): ConvexHull 形状
//
// GJK/EPA 已经就位，加 ConvexHull 缺的只剩"顶点存在哪儿"：
//   1. `ShapeType` 里加 `ConvexHull`，`-Wswitch` 会把所有需要补的分派点列出来
//   2. `Shape` 的 union 里加 `struct { uint32_t hullIndex; } hull;`
//      —— 存下标而不是指针，`Shape` 才能继续保持"可 memcpy 的 POD"
//   3. 顶点池需要一个**持有者**。它应该是 M8 的 PhysicsWorld：
//      World 在查询开始时把 hullIndex 解析成指针，填进 `ConvexProxy`
//      （那是个瞬时的栈上对象，放裸指针完全没问题 —— 见 GJK.h 的说明）
//   4. `LocalAABB` 由顶点集算；`Volume` 用四面体分解求和
//
// 之所以现在不加：加一个没有存储支撑的枚举值，只会让每个 switch 多出一个
// 静默返回错误结果的分支。这和 M2 当初推迟它的理由完全一样，只是当时缺的是
// 算法，现在缺的是存储。
//
// TODO(upgrade): 通用路径的多点流形
//   EPA 只给一个接触点。要生成完整的面接触，需要拿 EPA 的法线在两个形状上各找
//   一个"最迎着法线"的面，再做和盒-盒一样的 Sutherland-Hodgman 裁剪。
//   前提是形状能枚举自己的面 —— 所以这一条和 ConvexHull 是同一件事的两半。
//------------------------------------------------------------------------------

}  // namespace pe
