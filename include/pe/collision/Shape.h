#pragma once
//==============================================================================
// pe/collision/Shape.h
//
// 碰撞形状。**POD 变体**，不用继承 + 虚函数。
//
//------------------------------------------------------------------------------
// 为什么不用 `class Shape { virtual ... }` 的经典 OOP 写法
//------------------------------------------------------------------------------
//   1. 虚函数意味着每个形状一次堆分配 + 一个 vptr，遍历时指针追逐打散缓存。
//      窄相位每帧要访问成百上千个形状，这个代价是实打实的。
//   2. 窄相位本来就是**双分派**（球-盒 和 盒-球 是不同的算法），虚函数解决不了
//      双分派，还是得写分派表。既然要写表，单分派的虚函数就没有价值了。
//   3. POD 才能整块 memcpy、放进数组、直接作为 ECS 组件 —— 这是架构里
//      "数据与逻辑分离"那条原则的具体落地。
//
// 代价是加新形状要改所有 switch。这其实是**好事**：`-Wswitch`（-Wall 的一部分）
// 会把每一处没处理新形状的地方标出来，比运行时才发现"某个形状没实现"安全得多。
//
//------------------------------------------------------------------------------
// 局部坐标约定
//------------------------------------------------------------------------------
//   - 形状的**局部原点就是它的几何中心**（也是 M6 里的质心）。
//     需要偏移就靠外面的 Transform，形状本身不带偏移。
//   - 胶囊体的轴沿局部 **+Y**（和"角色站立"的直觉一致）。
//   - 盒子在局部空间是轴对齐的；配上 Transform 的旋转就是 OBB。
//     换句话说，引擎里没有独立的 "OBB 形状"，只有"带旋转的 Box"。
//
//------------------------------------------------------------------------------
// 关于 ConvexHull（任意凸多面体）
//------------------------------------------------------------------------------
// 架构文档里规划了 ConvexHull，但它**故意还没加进 ShapeType**。
// 原因：加一个没有任何算法支持的枚举值，只会让每个 switch 里多出一个
// "什么都不做"的分支，而这种分支不会报错、只会静默返回错误结果。
// 等 M5 把 GJK/EPA 写完、凸包真正可用时再加 —— 那时 `-Wswitch` 会精确地
// 列出所有需要补充的分派点，一个都跑不掉。
//==============================================================================

#include <cstdint>

#include "pe/collision/AABB.h"
#include "pe/core/Types.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec3.h"

namespace pe {

enum class ShapeType : std::uint8_t {
    Sphere = 0,
    Capsule = 1,
    Box = 2,
    // ConvexHull —— M5 加入，见文件头说明
};

struct Shape {
    ShapeType type;

    union {
        /// 球：以局部原点为中心。
        struct {
            real radius;
        } sphere;

        /// 胶囊：轴沿局部 +Y 的线段 [(0,-halfHeight,0), (0,+halfHeight,0)]，
        /// 外扩 radius。
        ///
        /// 注意 halfHeight 是**圆柱段**的半长，不含两端的半球。
        /// 所以整个胶囊的总高 = 2*halfHeight + 2*radius。
        /// 这是最容易搞错的一个参数 —— 给角色建胶囊时，
        /// 若角色身高 1.8 米、半径 0.3 米，halfHeight 应该是 0.6 而不是 0.9。
        struct {
            real radius;
            real halfHeight;
        } capsule;

        /// 盒：局部轴对齐，从中心到各面的距离。
        /// 即整个盒子的尺寸是 2*halfExtents。
        struct {
            Vec3 halfExtents;
        } box;
    };

    /// 不初始化。请用下面的工厂函数构造。
    Shape() = default;

    //--------------------------------------------------------------------------
    // 工厂
    //--------------------------------------------------------------------------

    static Shape MakeSphere(real radius) noexcept {
        Shape s;
        s.type = ShapeType::Sphere;
        s.sphere.radius = radius;
        return s;
    }

    /// halfHeight 是圆柱段半长，不含半球端盖（见上面 capsule 成员的说明）。
    static Shape MakeCapsule(real radius, real halfHeight) noexcept {
        Shape s;
        s.type = ShapeType::Capsule;
        s.capsule.radius = radius;
        s.capsule.halfHeight = halfHeight;
        return s;
    }

    /// 按"角色总身高"构造胶囊，省得每次自己换算。
    /// totalHeight 必须大于 2*radius（否则退化成球）。
    static Shape MakeCapsuleFromHeight(real radius, real totalHeight) noexcept {
        const real halfHeight = Max(real(0), totalHeight * real(0.5) - radius);
        return MakeCapsule(radius, halfHeight);
    }

    static Shape MakeBox(const Vec3& halfExtents) noexcept {
        Shape s;
        s.type = ShapeType::Box;
        s.box.halfExtents = halfExtents;
        return s;
    }

    /// 立方体。
    static Shape MakeCube(real halfSide) noexcept {
        return MakeBox(Vec3(halfSide, halfSide, halfSide));
    }

    //--------------------------------------------------------------------------
    // 几何属性
    //--------------------------------------------------------------------------

    /// 参数是否合法（半径为正等）。创建碰撞体时应该断言这个。
    bool IsValid() const noexcept;

    /// 以局部原点为球心的最小包围球半径。
    /// 用途：宽相位的快速粗筛、休眠判定的运动阈值换算。
    real BoundingRadius() const noexcept;

    /// 体积（立方米）。M6 由它和密度算出质量。
    real Volume() const noexcept;

    /// 局部空间的 AABB（未经任何变换）。
    AABB LocalAABB() const noexcept;
};

//------------------------------------------------------------------------------
// 世界空间 AABB
//------------------------------------------------------------------------------

/// 把形状按给定变换放到世界里，算出**紧致的**世界 AABB。
///
/// "紧致"很重要：偷懒的做法是拿 BoundingRadius 画一个球形 AABB，那对细长的
/// 盒子/胶囊会大到离谱，宽相位会产生海量无效候选对。这里对每种形状都用了
/// 各自的精确算法，具体推导见 Shape.cpp。
///
/// 这是宽相位每帧对每个动态碰撞体都要调用的函数，属于热点路径。
AABB ComputeWorldAABB(const Shape& shape, const Transform& transform) noexcept;

//------------------------------------------------------------------------------
// 胶囊的世界空间线段
//------------------------------------------------------------------------------

/// 取胶囊在世界空间的中轴线段端点。
/// 窄相位（胶囊-球、胶囊-胶囊）和射线求交都从这两个点出发，所以单独提出来。
/// shape 必须是 Capsule。
void GetCapsuleSegment(const Shape& shape, const Transform& transform, Vec3& outA,
                       Vec3& outB) noexcept;

}  // namespace pe
