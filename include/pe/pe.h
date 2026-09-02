#pragma once
//==============================================================================
// pe/pe.h  —  PhysEngine 汇总头
//
// 使用方只需要 #include <pe/pe.h>。
//
// 当前进度：M2（数学库 + core + 形状/AABB/射线求交）。
// 后续模块会按 ARCHITECTURE.md 第 5 节的里程碑顺序追加到下面。
//==============================================================================

// ---- core：类型、句柄、对象池 ----
#include "pe/core/Handle.h"
#include "pe/core/SlotArray.h"
#include "pe/core/Types.h"
#include "pe/core/Version.h"

// ---- math：向量、矩阵、四元数、变换 ----
#include "pe/math/Mat3.h"
#include "pe/math/Mat4.h"
#include "pe/math/MathUtil.h"
#include "pe/math/Quat.h"
#include "pe/math/Transform.h"
#include "pe/math/Vec2.h"
#include "pe/math/Vec3.h"
#include "pe/math/Vec4.h"

// ---- collision：形状、包围盒、射线 ----
#include "pe/collision/AABB.h"
#include "pe/collision/GeometryUtil.h"
#include "pe/collision/Ray.h"
#include "pe/collision/RayCast.h"
#include "pe/collision/ShapeCast.h"
#include "pe/collision/Collider.h"
#include "pe/collision/Shape.h"

// ---- collision：宽相位（M3 完成）；窄相位待 M4/M5 ----
#include "pe/collision/BroadPhase.h"
#include "pe/collision/UniformGrid.h"
#include "pe/collision/Manifold.h"
#include "pe/collision/EPA.h"
#include "pe/collision/GJK.h"
#include "pe/collision/NarrowPhase.h"

// ---- dynamics：刚体、质量属性、材质、积分器（M6 完成）----
#include "pe/dynamics/Integrator.h"
#include "pe/dynamics/MassProperties.h"
#include "pe/dynamics/Material.h"
#include "pe/dynamics/RigidBody.h"

// ---- solver：接触约束与顺序冲量求解器（M7 完成）----
#include "pe/solver/ContactConstraint.h"
#include "pe/solver/IConstraint.h"
#include "pe/solver/SequentialImpulseSolver.h"

// ---- character / scene：角色控制器、触发器、事件（M8 完成）----
#include "pe/character/CharacterController.h"
#include "pe/scene/Events.h"
#include "pe/scene/PhysicsWorld.h"
#include "pe/scene/TriggerSystem.h"
// #include "pe/scene/PhysicsWorld.h"
