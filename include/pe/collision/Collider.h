#pragma once
//==============================================================================
// pe/collision/Collider.h
//
// 碰撞体：把一个 Shape 挂到刚体上，并带上材质、层过滤、触发器标志。
//
//------------------------------------------------------------------------------
// 为什么 Shape 和 Collider 是两个东西
//------------------------------------------------------------------------------
// `Shape` 是**纯几何**（一个球、一个盒），不知道自己在哪儿、属于谁、什么材质。
// `Collider` 才是场景里的实体：它说"这个形状挂在那个刚体上、偏移这么多、
// 表面是木头、只和这些层碰撞、而且它是个触发器"。
//
// 拆开的好处是同一个 Shape 可以被无数个 Collider 复用（一张地图上几百个一样的
// 箱子只有一份形状数据），而且几何算法（M2-M5）完全不需要知道 Collider 的存在。
//
//------------------------------------------------------------------------------
// 一个刚体可以挂多个 Collider
//------------------------------------------------------------------------------
// 这是做"复合形状"的唯一办法 —— 引擎不支持凹形状，一把锤子只能用
// "长条盒 + 方块盒"两个碰撞体拼出来。
//
// 代价是刚体的质心不再是形状原点：World 会把各个碰撞体的质量属性按平行轴定理
// 合成（`MassProperties.h` 里的 `TranslateInertia` 就是为此准备的），
// 得到一个偏离刚体原点的**复合质心**。`RigidBody::position` 存的是那个质心，
// 而不是用户设定的"物体原点"—— 两者的换算由 World 负责，见 PhysicsWorld.h。
//==============================================================================

#include "pe/collision/BroadPhase.h"
#include "pe/collision/Shape.h"
#include "pe/core/Handle.h"
#include "pe/core/Types.h"
#include "pe/dynamics/MassProperties.h"
#include "pe/dynamics/Material.h"
#include "pe/math/Transform.h"

namespace pe {

//------------------------------------------------------------------------------
// 创建描述
//------------------------------------------------------------------------------
struct ColliderDesc {
    Shape shape;

    /// 相对**刚体原点**的偏移。默认是原点，也就是"形状中心就是物体中心"。
    Transform localTransform = Transform(Vec3::Zero(), Quat::Identity());

    Material material = Material::Default();

    LayerMask layer = layers::kDefault;
    LayerMask layerMask = kLayerAll;

    /// 触发器：只判定重叠、**不生成接触约束**，物体会直接穿过去。
    /// 拾取区、买枪区、伤害区都是这个。
    bool isTrigger = false;

    /// 密度（kg/m^3）。只在 `BodyDesc::mass <= 0`（"让引擎自己算"）时用到。
    real density = densities::kDefault;
};

//------------------------------------------------------------------------------
// 碰撞体
//------------------------------------------------------------------------------
struct Collider {
    Shape shape;
    Transform localTransform;
    Material material;

    LayerMask layer;
    LayerMask layerMask;

    /// 挂在哪个刚体上。碰撞体不能脱离刚体存在 —— 想要"纯静态的地图几何"，
    /// 就挂在一个 `BodyType::Static` 的刚体上（invMass = 0，不积分，几乎零成本）。
    BodyHandle body;

    bool isTrigger;

    /// 在宽相位里的代理 id。World 内部维护，用户不用管。
    ProxyId proxy;

    /// 密度（kg/m^3）。重算质量属性时要用（加/删碰撞体都会触发重算）。
    real density;

    /// 缓存的世界位姿。每帧由 World 从刚体位姿和 localTransform 算出来，
    /// 避免窄相位、射线查询、扫掠各算一遍。
    Transform worldTransform;
};

}  // namespace pe
