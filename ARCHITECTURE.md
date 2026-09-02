# PhysEngine — 3D FPS 物理引擎架构设计文档

> 本文件是这个引擎的**上下文文档**。每次找 AI 迭代升级时，先让它读这一份，
> 就能拿到完整的设计决策、模块边界和依赖关系，不需要通读代码。
> 每完成一个模块都会回来更新对应章节的「实现状态」与「已知局限」。

- 版本：v1.0（**M1–M9 全部完成**：碰撞检测 + 动力学 + 求解器 + 角色控制器 + 触发器 + PhysicsWorld + Demo）
- 最后更新：2026-09-01
- 目标：类 CS 的第一人称射击游戏，3D
- C++ 标准：C++20 ｜ 构建：CMake >= 3.20 ｜ 测试：Catch2 v3.7.1（FetchContent 拉取）
- 实际工具链（本机）：MinGW-w64 GCC 16.2.0 (UCRT) + CMake 4.4.2，解压在 `C:/tools/`，未写入系统 PATH；用 `bash scripts/build.sh` 一键构建
- **可执行文件静态链接运行时**（`CMakeLists.txt` 里 MinGW 分支的 `-static-libgcc -static-libstdc++ -static`）。
  不加的话 exe 会依赖 `libgcc_s_seh-1.dll` / `libstdc++-6.dll` / `libwinpthread-1.dll`，
  而这三个躺在没进 PATH 的 `C:\tools\mingw64\bin` —— 构建脚本里跑没事，
  双击 exe 或者拷到别的机器上就报"找不到 libgcc_s_seh-1.dll"。
  静态之后只剩 KERNEL32/USER32 + UCRT（Windows 10 起系统自带），拷哪儿都能跑
- 形态：**独立静态库 `physengine`**，不含渲染，通过纯数据接口对接任意渲染层
- 性能目标：60 fps 稳定，单场景 50-100 个动态刚体 + 约 500-2000 个静态碰撞体，单线程

---

## 0. 已确认的关键决策

| 决策点 | 选择 | 理由 |
|---|---|---|
| 维度 | 3D | FPS 需求 |
| 动力学 | 完整刚体（含角速度 / 转动惯量张量）+ 独立角色控制器 | 一步到位，避免二次重构 |
| 窄相位 | 解析闭式解（球/胶囊/AABB）+ GJK/EPA（OBB/任意凸包） | 常见形状走快路径，任意凸包有通用兜底 |
| 碰撞响应 | Sequential Impulse（顺序冲量）+ Baumgarte 位置修正 + 库仑摩擦锥 | 真实弹跳与稳定堆叠，是后续一切约束的基础 |
| 宽相位 | 均匀网格 Uniform Grid | 实现简单，FPS 地图尺度均匀，未来可换 BVH |
| 积分器 | 半隐式欧拉（Semi-implicit / Symplectic Euler） | 能量不发散，与冲量求解器天然配套 |
| 内存 | POD 数据存 `std::vector` + `Handle{index, generation}` 句柄；生命周期由 World 持有，用户侧只拿句柄 | 数据与逻辑分离，直通 ECS 化；避免悬垂指针 |
| 并发 | 第一版单线程 | 先保证正确性，热点已按可并行结构划分 |
| 关节/约束 | 不实现，但求解器抽象为 `IConstraint`，门/绳索可后接 | 预留升级点 |

---

## 1. 目录结构

```
c:\game\
├─ CMakeLists.txt              # 顶层：physengine 库 + tests + demo
├─ ARCHITECTURE.md             # 本文件
├─ UPGRADE_NOTES.md            # 每个模块的已知局限 / 未来升级方向清单
├─ include/pe/                 # 公开头文件（对外 API 面）
│  ├─ pe.h                     # 汇总头，用户只需 #include <pe/pe.h>
│  ├─ core/
│  │   ├─ Types.h              # real(float) 别名、常量、BodyType、Layer 掩码
│  │   ├─ Handle.h             # 泛型 Handle<Tag>{uint32 index, uint32 generation}
│  │   └─ SlotArray.h          # 句柄化对象池（ID 复用 + 代际号防悬垂）
│  ├─ math/
│  │   ├─ Vec2.h Vec3.h Vec4.h
│  │   ├─ Mat3.h Mat4.h        # Mat3 用于惯量张量/旋转，Mat4 用于渲染层交换
│  │   ├─ Quat.h               # 姿态表示的唯一真相源
│  │   ├─ Transform.h          # {Vec3 position, Quat rotation}（无缩放，见 §2）
│  │   └─ MathUtil.h           # clamp/lerp/slerp/近似相等/安全归一化
│  ├─ collision/
│  │   ├─ Shape.h              # ShapeType 枚举 + Shape 变体（POD，无虚函数）
│  │   ├─ AABB.h  Ray.h
│  │   ├─ Collider.h           # Shape + 局部偏移 + 材质 + 层掩码 + 所属刚体
│  │   ├─ Manifold.h           # 接触流形：法线 + 最多 4 个接触点 + 穿透深度
│  │   ├─ BroadPhase.h         # IBroadPhase 接口
│  │   ├─ UniformGrid.h        # IBroadPhase 的第一版实现
│  │   ├─ NarrowPhase.h        # 形状对 -> 分派表
│  │   ├─ GJK.h  EPA.h         # 通用凸包相交 + 穿透深度
│  │   └─ RayCast.h            # 各形状的射线求交闭式解
│  ├─ dynamics/
│  │   ├─ RigidBody.h          # 纯 POD 状态块
│  │   ├─ MassProperties.h     # 各形状的质量/惯量张量计算
│  │   ├─ Material.h           # 恢复系数 / 摩擦系数
│  │   └─ Integrator.h         # 半隐式欧拉，速度积分与位置积分分开两个函数
│  ├─ solver/
│  │   ├─ IConstraint.h        # 约束抽象：Prepare / WarmStart / SolveVelocity
│  │   ├─ ContactConstraint.h  # 接触点约束（法向 + 两个切向）
│  │   └─ SequentialImpulseSolver.h
│  ├─ character/
│  │   └─ CharacterController.h # 胶囊体扫掠移动，不走刚体模拟
│  └─ scene/
│      ├─ PhysicsWorld.h       # 门面：Step / 创建销毁 / 查询
│      ├─ TriggerSystem.h      # Enter/Stay/Exit 状态机
│      └─ Events.h             # 事件队列（碰撞开始/结束、触发器）
├─ src/                        # 与 include 镜像的实现
├─ tests/                      # Catch2 单元测试，一个模块一个文件
├─ demo/                       # 整合 Demo（胶囊角色 + AABB 地图 + hitscan）
├─ platform/                   # 两个游戏共用的显示层，和引擎、和玩法都无关
│   ├─ Window.h                # Win32 + GDI 窗口，只做"贴一块像素上去"
│   └─ Canvas.h                # 32 位画布 + 3x5 点阵字模 + PPM 输出
├─ game/                       # 示例游戏一：FPS（见 game/README.md）
└─ racing/                     # 示例游戏二：赛车（见 racing/README.md）
                               # 引擎里没有"车"：一个刚体 + 四条向下的射线
```

**构建目标**：`physengine`（静态库）、`pe_tests`（Catch2）、`pe_demo`、
`pe_game`（FPS）、`pe_racing`（赛车）。由 `PE_BUILD_TESTS` / `PE_BUILD_DEMO` /
`PE_BUILD_GAME` / `PE_BUILD_RACING` 四个 CMake 选项控制，默认全 ON。

两个游戏是引擎的**使用者**，不是引擎的一部分：`physengine` 不知道它们的存在，
也不含任何渲染或输入代码。它们互相之间也没有依赖，只共用 `platform/`。
两个游戏用引擎的方式差别很大 —— FPS 用运动学角色控制器 + 射线渲染，
赛车用动态刚体 + 按子步施加的悬挂/轮胎力 —— 所以它们合起来才是一份有意义的
集成测试（`pe_game_smoke` 和 `pe_racing_smoke`）。

---

## 2. 模块依赖关系（严格单向，禁止反向 include）

```
            ┌──────────┐
            │   core   │  Types / Handle / SlotArray
            └────┬─────┘
                 │
            ┌────▼─────┐
            │   math   │  Vec3 Mat3 Quat Transform
            └────┬─────┘
                 │
        ┌────────▼────────┐
        │    collision    │  Shape / BroadPhase / NarrowPhase / GJK / Ray
        └───┬─────────┬───┘
            │         │
   ┌────────▼──┐   ┌──▼──────────┐
   │  dynamics │   │  character  │  角色控制器只用碰撞查询，不依赖 solver
   └────┬──────┘   └──────┬──────┘
        │                 │
   ┌────▼──────┐          │
   │  solver   │          │
   └────┬──────┘          │
        │                 │
     ┌──▼─────────────────▼──┐
     │        scene          │  PhysicsWorld 门面 + Trigger + Events
     └───────────────────────┘
```

规则：

- `collision` **不知道**质量、速度的存在，只吐几何结果（Manifold / RaycastHit）。
  这样将来换掉整个动力学层，碰撞层能原样复用。
- `solver` **不做**碰撞检测，只消费 Manifold 数组和刚体状态数组。
- `character` 与 `solver` 平级、互不依赖 —— 角色控制器是「游戏手感」逻辑，
  绝不参与冲量求解，否则手感会被物理模拟污染（业界标准做法）。
- `scene` 是唯一允许 include 全部下层的模块，它是编排者。

**关于 Transform 不含缩放**：物理形状的非均匀缩放会破坏惯量张量与 GJK 支撑函数的
正确性。需要不同尺寸就在创建 Shape 时给不同参数，而不是缩放变换。

---

## 3. 核心数据结构（POD，为 ECS 化预留）

```cpp
using real = float;   // 单一 typedef，将来换 double 只改一行

// ---- core/Handle.h ----
template <class Tag>
struct Handle {                 // 8 字节，可平凡拷贝，可直接放进 ECS 组件
    uint32_t index      = kInvalidIndex;  // 在 SlotArray 中的槽位
    uint32_t generation = 0;              // 代际号：槽位被复用后旧句柄自动失效
    bool IsValid() const;
};
using BodyHandle     = Handle<struct BodyTag>;
using ColliderHandle = Handle<struct ColliderTag>;

// ---- dynamics/RigidBody.h ----
// 纯数据：无虚函数、无构造逻辑、无指针 —— 整块 memcpy 安全，
// 未来可直接拆成 ECS 的若干组件（Transform / Velocity / MassData）。
struct RigidBody {
    // 位姿（世界空间）
    Vec3 position;
    Quat rotation;
    // 速度
    Vec3 linearVelocity;
    Vec3 angularVelocity;        // 世界空间，单位 rad/s
    // 累积外力（每帧 Step 结束清零）
    Vec3 force;
    Vec3 torque;
    // 质量属性
    real invMass;                // 0 表示静态/运动学（等价无穷大质量）
    Mat3 invInertiaLocal;        // 局部空间惯量张量的逆（对角阵，由形状决定）
    Mat3 invInertiaWorld;        // 每帧由 R * I_local^-1 * R^T 更新并缓存
    // 阻尼与休眠
    real linearDamping, angularDamping;
    real sleepTimer;
    bool isSleeping;
    BodyType type;               // Static / Kinematic / Dynamic
    uint32_t layer, layerMask;   // 碰撞过滤
};

// ---- collision/Shape.h ----（M2 已实现）
// ConvexHull 待 M5（GJK/EPA）时加入枚举，理由见 §4.2
enum class ShapeType : uint8_t { Sphere = 0, Capsule = 1, Box = 2 };
struct Shape {                   // 变体式 POD，不用继承和虚函数
    ShapeType type;
    union {
        struct { real radius; }             sphere;
        struct { real radius, halfHeight; } capsule;  // 轴沿局部 +Y；
                                                      // halfHeight 是圆柱段半长，
                                                      // 不含端盖半球
        struct { Vec3 halfExtents; }        box;      // 局部轴对齐，加旋转即 OBB
    };
};

// ---- collision/Ray.h ----（M2 已实现）
struct Ray {
    Vec3 origin;
    Vec3 direction;              // 永远是单位向量（构造函数保证）
    real maxDistance;
};
struct RaycastHit {              // 纯几何结果：没有 bool hit（用返回值），
    real distance;               // 也没有句柄（collision 层不知道刚体的存在）。
    Vec3 point;                  // World 层（M8）会在外面包一层补上句柄。
    Vec3 normal;
};

// ---- collision/Manifold.h ----（M4 已实现）
struct ContactPoint {
    Vec3 position;               // 接触点世界坐标 = 两个见证点的中点
    real penetration;            // 穿透深度（正值重叠；负值 = 推测性接触）
    uint32_t featureId;          // 特征 ID，用于跨帧匹配做 warm starting
};
struct Manifold {
    Vec3 normal;                 // 约定：始终由 A 指向 B，单位向量
    uint8_t pointCount;          // <= 4
    ContactPoint points[4];
};
```

> **和最初的草图相比，Manifold 瘦了一圈**，实现时把 `ColliderHandle a, b`、
> `friction / restitution`、`tangent[2]`、以及接触点上的 `rA/rB` 和累积冲量
> 全部移了出去。理由是本文件 §2 自己定下的那条规矩 ——
> *collision 层不知道质量、速度的存在，只吐几何结果*：
>
> - **摩擦/恢复系数**是材质属性。窄相位要么得跨层去读材质，要么把字段留着不填；
>   而"结构里躺着一个没人负责填的字段"正是 `Ray.h` 里痛斥过的、迟早会不一致的
>   设计。它们归 M7 的 `ContactConstraint`。
> - **累积冲量 / rA / rB** 是求解器的迭代状态。窄相位每帧从零重算几何，
>   让它捎带一份求解器状态没有道理。M7 的接触缓存按 `featureId` 把上一帧的冲量
>   匹配回来即可 —— 这正是 `featureId` 存在的全部意义。
> - **句柄**：和 `RaycastHit` 完全同一个理由。纯几何结果不带身份，
>   World 层（M8）在外面包一层 `ContactPair { ColliderHandle a, b; Manifold m; }`。
> - **切向基**：`Vec3.h` 的 `BuildOrthonormalBasis(n, t1, t2)` 已经能由 normal
>   现算，存一份只是让两份数据有机会不一致。
>
> 换来的是 Manifold 成为一个纯 POD、语义完全自洽的几何结果，
> 和 `RaycastHit` 处在同一个抽象层次上。

---

## 4. 各模块设计详述

### 4.1 数学基础库 `math`（模块 1）—— ✅ 已完成

> 实现：`include/pe/core/*`、`include/pe/math/*`（全部 header-only，短小函数需要内联到热循环）
> 测试：`tests/test_core.cpp`、`tests/test_math.cpp` —— **52 用例 / 293 断言全过**
> 局限与升级方向见 `UPGRADE_NOTES.md` 的 M1 一节

实测已确认的性质（都有测试守着）：
- 头文件在 `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast`
  下零警告。`-Wconversion` 尤其重要：它保证将来把 `real` 换成 double 时，
  不会有静默的精度截断。
- 所有数学类型都是**平凡可复制**的 POD（`static_assert` 守着），
  可以 memcpy、可以放进 `Shape` 的 union（M2 要用）、可以直接作为 ECS 组件。
- 退化输入不产生 NaN：零向量归一化、奇异矩阵求逆、`acos` 超出定义域、
  `BuildOrthonormalBasis` 传入 ±Y —— 每一条都有专门的 `[degenerate]` 用例。
  这条很关键：NaN 一旦进入物理状态就会顺着 位置 → AABB → 宽相位 扩散，
  整个世界报废且极难定位。

接口要点：
- `Vec3`：点积、叉积、长度、`Normalized()`（长度为 0 时返回零向量而非 NaN）、
  `Normalize()`（原地归一化并返回原长度）、逐分量工具、
  `BuildOrthonormalBasis()`（摩擦切向基，用 Erin Catto 的无退化构造法）。
- `Quat`：单位四元数表示姿态。提供 `FromAxisAngle`、`ToMat3`、`Slerp`、
  `Integrate(omega, dt)`（角速度积分）、`Normalize`（每帧重归一化防数值漂移）。
- `Mat3`：旋转矩阵与惯量张量共用。`Transpose`、`Inverse`、`operator*`。
- `Mat4`：**只用于与渲染层交换矩阵**。物理内部一律用 `Transform`（位置 + 四元数），
  因为四元数插值稳定、内存小、无剪切退化。
- 约定：**行优先存储，列向量右乘**（`v' = M * v`）。这一条写死在头文件注释里 ——
  和渲染层对接时最容易出错的就是这里。

角速度积分的数学：四元数导数 `qdot = 0.5 * omega_quat * q`，其中
`omega_quat = (0, wx, wy, wz)` 是纯四元数。所以
`q(t+dt) ~= q + 0.5 * dt * (omega_quat * q)`，再归一化。代码里会写完整推导。

**这一层钉死的约定**（后续所有模块必须遵守，每条都有对应的测试用例守着；
清单见 `UPGRADE_NOTES.md` 的「M1 留下的约定」表）：

| 约定 | 守护它的测试 |
|---|---|
| 右手坐标系，X × Y == Z | `Vec3 点积与叉积` |
| 行优先存储，列向量右乘 `v' = M * v` | `Mat3 矩阵乘向量是列向量右乘` |
| 四元数 `q1 * q2` 表示"先做 q2 再做 q1" | `Quat 复合顺序` |
| 正角度 = 逆时针（从轴正方向朝原点看） | `Quat 绕轴旋转方向符合右手定则` |
| 点会被平移，方向不会 | `Transform 点与方向的变换不同` |
| 导出给 OpenGL 是列优先，平移在下标 12/13/14 | `Mat4 列优先导出符合 OpenGL 布局` |

> ⚠️ 与渲染层对接前请先核对最后一条。如果你的渲染层是列优先 + 行向量左乘，
> 现在改比到 M9 再改便宜得多。

**一个实现上的坑（已修，记下来避免重蹈）**：类内定义的成员函数看不到在该类之后
才声明的自由运算符（C++ 的非限定名字查找发生在定义点，不是 TU 末尾）。
`Vec3::Normalized()` 和 `Quat::Integrate()` 最初都踩了这个坑。
前者改成直接构造返回值，后者把实现挪到了头文件末尾。

### 4.2 碰撞检测 `collision`（模块 2）

> **进度**：形状定义、AABB、射线求交已完成（M2）；宽相位已完成（M3）；窄相位（M4/M5）未开始。
>
> 形状层的几个决定：
> - `Shape` 是 **POD 变体（union）**，不用继承 + 虚函数。窄相位本来就是双分派，
>   虚函数解决不了双分派、还得写表；而 POD 能整块 memcpy、放数组、当 ECS 组件。
>   代价是加新形状要改所有 switch —— 这其实是好事，`-Wswitch` 会精确列出遗漏点。
> - **`ConvexHull` 故意还没进 `ShapeType` 枚举**。加一个没有算法支持的枚举值，
>   只会让每个 switch 多出一个静默返回错误结果的分支。等 M5 的 GJK/EPA 写完
>   再加，那时 `-Wswitch` 会把所有需要补充的分派点一个不漏地列出来。
> - 胶囊的 `halfHeight` 是**圆柱段半长，不含端盖半球**（总高 = 2*halfHeight + 2*radius）。
>   这是最容易搞错的参数，所以另外提供了 `MakeCapsuleFromHeight(radius, 总身高)`。
> - `ComputeWorldAABB` 对旋转盒子用 `abs(R) * halfExtents` 求紧致 AABB，
>   而不是退化成包围球。宽相位的候选对数量对 AABB 体积极其敏感，
>   测试里有专门的用例守着这个紧致性。

**宽相位 — Uniform Grid** —— ✅ 已完成（M3）

实现：`include/pe/collision/{BroadPhase,UniformGrid}.h`、`src/collision/UniformGrid.cpp`

- 世界按 `cellSize`（默认取场景平均物体尺寸的 2 倍）划分为无限稀疏网格，
  用 `unordered_map<CellKey, vector<ProxyId>>`，`CellKey` 由 (x,y,z) 整数坐标哈希。
  坐标换算必须用 `floor` 而非截断取整，否则原点附近的格子会变成双倍大 ——
  这类只在负半轴出问题的 bug 有专门的用例守着。
- 每个碰撞体注册一个 proxy 并缓存其世界 AABB。用「fat AABB」留 margin，
  小幅移动不重建，减少哈希表抖动。**配对判定用的是 fat AABB 而不是紧致 AABB**：
  求解器的 warm starting 要求接触流形在真正接触之前就诞生，否则堆叠会在
  第一帧接触时"陷一下再弹回来"。
- **三张表**：静态一张（构建一次不再更新）、动态一张（每帧更新）、超大体一个
  线性列表。配对只遍历动态表的格子，再去静态表查同一个 key —— 静态-静态对
  连产生的机会都没有，不需要事后过滤。
  跨越超过 `kMaxCellsPerProxy`（4096）个格子的物体（地面、天空盒）转入
  超大体列表走暴力，避免往哈希表里塞几万个条目。
- 候选对去重**不用哈希集合**，用「归属格」：两个 proxy 的格子区间交集的最小角
  只由这两个 proxy 决定、与遍历顺序无关，规定只有那个格子吐出这一对。
  O(1)、无分配、无哈希。
- 过滤规则集中在自由函数 `BroadPhaseShouldCollide`（静态-静态丢弃；双方都休眠
  丢弃；层掩码必须双方都同意；最后才做 AABB 重叠）。它必须被所有 IBroadPhase
  实现共用 —— 测试里的暴力参考实现也调用它，这样对拍检验的才是空间划分逻辑
  本身，而不是"两份过滤规则抄得一不一样"。
- `QueryRay` 用 Amanatides & Woo 的 3D DDA 体素遍历，先把射线裁到网格总包围盒
  再逐格推进，天然由近及远（上层可在第一次精确命中后提前退出，所以不排序）。
- 接口 `IBroadPhase { Insert / Remove / Update / QueryPairs / QueryAABB / QueryRay }`。
  **升级点**：换成 BVH 只需实现同一接口，World 代码不动。

实测（2000 静态 + 100 动态，`cellSize = 2`，RelWithDebInfo）：
每帧 100 次 `Update` + 一次 `QueryPairs` + 一条 `QueryRay` 合计 **0.075 ms**，
同场景同过滤规则的暴力 O(n²) 是 **3.96 ms** —— 约 53 倍。

**窄相位 — 分派表** —— ✅ 解析解部分已完成（M4）

实现：`include/pe/collision/{Manifold,NarrowPhase}.h`、`src/collision/NarrowPhase.cpp`

| 形状对 | 算法 | 状态 |
|---|---|---|
| Sphere-Sphere | 闭式：圆心距 vs 半径和 | ✅ M4 |
| Sphere-Capsule / Capsule-Capsule | 闭式：线段-线段最近点 + 半径 | ✅ M4 |
| Sphere/Capsule-Box | 闭式：点/线段到 OBB 的最近点 | ✅ M4 |
| Box-Box | SAT（15 根分离轴：3+3 面法线 + 9 条边叉积），面裁剪生成流形 | ✅ M4 |
| 任意凸形状（通用兜底） | GJK 判相交/距离 -> EPA 求穿透 | ✅ M5（单接触点） |
| ConvexHull 形状本身 | 同上，外加面裁剪生成多点流形 | 待顶点池有归属，见下 |

> **ConvexHull 为什么还没进 `ShapeType`**：算法（GJK/EPA）在 M5 已经就位，
> 现在缺的是**存储**。`Shape` 必须保持"不含指针、可 memcpy 的 POD"，所以凸包只能
> 存一个 `uint32_t hullIndex` 索引进某个顶点池 —— 而那个池需要一个持有者，
> 它应该是 M8 的 `PhysicsWorld`：World 在查询开始时把 hullIndex 解析成裸指针，
> 填进 `ConvexProxy`（那是个瞬时的栈上对象，放指针完全没问题）。
> 这就是"POD 存储 + 瞬时代理"的分工，两边都不用妥协。
>
> 理由和 M2 当初推迟它时一样：加一个没有支撑的枚举值，只会让每个 switch 多出一个
> 静默返回错误结果的分支。区别只在于当时缺的是算法，现在缺的是存储。

窄相位的几个决定：

- **只实现"规范顺序"的一半**。形状对按 `ShapeType` 枚举值升序排列，一共 6 个函数；
  `Collide()` 分派时若顺序反了就交换着调用再把法线取反。因为接触点取的是
  **两个见证点的中点**，交换 A、B 不改变接触点位置，取反法线就够了。
  于是得到一条很强的可测契约：`Collide(A,B)` 与 `Collide(B,A)` 给出相同的接触点、
  相同的穿透深度、相反的法线（盒-盒除外，见下）。
- **三个形状对共用一个内核**。球-胶囊、胶囊-胶囊在化简之后都是球-球 ——
  胶囊本来就是"扫掠球体"，只要先把"轴线上哪一点参与接触"解出来就退化了。
  实际上只有三个和盒子有关的对需要独立算法。
- **推测性接触**：两个形状表面相距 `kSpeculativeMargin`（= 2×`kLinearSlop`）以内
  就生成接触点，此时 `penetration` 为负。理由和宽相位用 fat AABB 一样：
  warm starting 要求接触在真正碰上之前就存在。
- **盒-盒的面/边偏置**：SAT 找到最小穿透轴后，只有当边叉积轴**明显**更浅
  （相对 0.95 + 绝对 5 毫米双阈值）才用它。原因是边叉积轴的数值质量差得多
  （两根近乎平行的轴叉出来的方向误差能到几度），而且边-边只能给一个接触点，
  撑不住堆叠。代价是盒-盒**不满足** A/B 交换对称性（可能选中另一个盒子做参考面），
  但法线与穿透深度仍然对称。
- **`Manifold` 只装几何**。架构文档最初的草图里它还带着句柄、摩擦/恢复系数和
  累积冲量，实现时全部移出去了 —— 详见 §3 的说明。

**GJK 原理（代码中会写完整注释）**：两个凸集 A、B 相交，当且仅当它们的闵可夫斯基差
`A - B = {a - b}` 包含原点。GJK 不显式构造这个差集，而是用**支撑函数**
`support(d) = maxPoint(A, d) - maxPoint(B, -d)` 增量地构建最多 4 个顶点的单纯形，
每轮朝「当前单纯形最接近原点的方向」搜索新的支撑点。若新点在搜索方向上的投影小于 0，
说明原点在闵可夫斯基差之外 -> 不相交，提前退出。

**EPA 原理**：GJK 返回的四面体包住原点之后，EPA 把它当作初始多面体，反复寻找
**离原点最近的面**，沿该面法线取新支撑点扩张多面体，直到扩张量小于容差。
收敛时那个最近面的法线即碰撞法线，距离即穿透深度。之所以必须要 EPA，是因为
GJK 只回答「相不相交」，无法给出分离所需的最小平移向量（MTV）。

**GJK / EPA 实现状态** —— ✅ 已完成（M5）

实现：`include/pe/collision/{GJK,EPA}.h`、`src/collision/{GJK,EPA}.cpp`，
通用入口是 `CollideConvex()` 与 `ConvexDistance()`。

实现时最关键的一个决定：**GJK/EPA 只跑在「核」上，半径事后再加**。
球是「一个点外扩半径」、胶囊是「一条线段外扩半径」。若让支撑函数把半径也算进去，
GJK 就要在**曲面**上迭代 —— 曲面没有顶点，收敛会明显变慢，EPA 更要在球面上堆出
成百上千个三角形才够精度。所以支撑函数只返回核（球→圆心，胶囊→轴线端点，盒→角点），
半径由 `CoreRadius()` 单独给出：

```
核不相交:  穿透 = rA + rB - 核距离      （GJK 一步到位，不需要 EPA）
核相交:    穿透 = 核穿透 + rA + rB      （核穿透由 EPA 给出）
```

好处是核全是**多面体**，GJK 能在有限步内精确终止；而球/胶囊之间的碰撞
根本走不到 EPA。这也正是 Bullet 的 margin 机制在做的事。

代价是**闵可夫斯基差可能没有体积**：球⊖球是一个点、球⊖胶囊是一条线段、
胶囊⊖胶囊是一个平行四边形。这带来两个必须处理的陷阱，两者都有专门的测试守着：

- GJK 的四面体最近点判定里，任意 4 个共面顶点会让「原点在不在这个面外侧」
  的判据全部失效，从而**误报相交**。解法是先算有向体积、用棱长无量纲化，
  扁到一定程度就不走「内部」那条路（`ClosestOnTetrahedron` 的 `flat` 分支）。
- EPA 撑不起初始四面体。此时核穿透确实是 0，还需要的只有方向：
  从点集实际张成的维数反推（2 维取平面法线，1 维取任意垂直方向）。
  注意**不能拿顶点个数当维数用** —— 撑四面体的过程中可能塞进来几个共线的点。

实测（2 万对随机旋转的盒子，其中 1.5 万对重叠）：

| 路径 | 每对耗时 | |
|---|---|---|
| SAT + 面裁剪（解析解） | 0.77 us | 4 个接触点 |
| GJK + EPA（通用路径） | 1.97 us | 1 个接触点，慢 2.6 倍 |

GJK 平均 3.5 轮收敛，最多 8 轮。这个差距就是 `Collide()` 默认不走通用路径的理由。

**接触流形生成**：单点接触无法阻止旋转（盒子会绕接触点翻转），所以要把 EPA/SAT
给出的单一法线，通过**参考面 vs 入射面的 Sutherland-Hodgman 裁剪**扩成多个接触点，
再按「保留面积最大的四边形」原则精简到 4 点。

**跨帧持久化**：Manifold 按 `(colliderA, colliderB)` 缓存在哈希表里，新旧接触点按
`featureId` 匹配，继承上一帧的累积冲量做 **warm starting** —— 这是堆叠稳定性的关键，
少了它，箱子堆会肉眼可见地下陷抖动。

### 4.3 刚体动力学 `dynamics`（模块 3）—— ✅ 已完成（M6）

> 实现：`include/pe/dynamics/{RigidBody,MassProperties,Material,Integrator}.h`、
> `src/dynamics/{MassProperties,Integrator}.cpp`

**半隐式欧拉**，拆成两段，中间夹着求解器：

```
// 阶段 A：积分速度（求解器之前）
v += (F/m + g) * dt
w += I_world^-1 * (tau - w x (I_world * w)) * dt

// ---- 求解器在这里直接修改 v、w ----

// 阶段 B：积分位置（求解器之后）
x += v * dt
q  = Normalize(q + 0.5 * dt * (w_quat * q))
```

**为什么是半隐式**：显式欧拉用旧速度更新位置，系统能量单调增长，弹簧和堆叠必然爆炸；
半隐式先更新速度、再用**新速度**更新位置，属于辛（symplectic）积分器，能量在真值
附近有界振荡而不发散。而且它与「求解器直接修改速度」的冲量法完美契合 ——
求解器改完速度，位置积分立刻就能用上，不需要重新求力。

`- w x (I*w)` 是欧拉方程里的陀螺项，保证自由旋转刚体的角动量守恒
（不写这一项，长条盒子的翻滚会不自然）。

**质量属性**：`MassProperties` 按形状解析计算惯量张量 —— 实心球 `2/5 * m * r^2`、
盒子 `1/12 * m * (h^2 + d^2)`、胶囊按圆柱 + 两个半球分段积分并做平行轴定理修正。
每个公式在代码里给出推导来源。

**休眠**：线速度与角速度连续 `kSleepTime`（0.5s）低于阈值则休眠，被新碰撞唤醒。
50-100 个动态体的性能目标，主要就靠休眠 + 宽相位剪枝达成。
计时器要求**连续**低速：任何一项超标就清零重计，一个反复被撞醒的箱子攒不够时间。
阈值不能取 0 —— 物体在弹跳最高点速度瞬时为零，立刻休眠会让它卡在半空。

**实现时补充的几个决定（M6）**：

- **阻尼用 `pow(1 - damping, dt)` 而不是 `1 - damping * dt`**。前者是指数衰减的
  精确解，与步长无关 —— 同一个阻尼系数在 30fps 和 144fps 下衰减一致；
  后者在大步长时会变成负数，把速度整个反向。有专门的用例守着这条。
  另外阻尼在这里**不是空气阻力的物理模型**，而是数值稳定手段：
  无阻尼模拟里求解器的残余误差会累积成永不停歇的微小抖动，物体永远睡不着。
- **陀螺项要限幅**。`w x (I*w)` 随角速度二次增长，高速自旋时显式积分一步就能发散。
  所以限制它单步造成的角速度变化不超过当前角速度本身。这在物理上是"错的"，
  但发散的模拟比略有误差的模拟错得多。
- **`invInertiaWorld` 的刷新点**。`I_world^-1 = R * I_local^-1 * R^T` ——
  注意可以直接变换**逆矩阵**（因为 R 正交，`(R I R^T)^-1 = R I^-1 R^T`），
  每帧省掉一次 3x3 求逆。`IntegratePosition` 内部会刷新，但任何直接改写
  `rotation` 的地方（传送、编辑器拖拽、网络同步）都必须手动跟着调
  `UpdateWorldInertia`，否则求解器会拿着上一帧姿态的惯量算冲量。
- **材质的组合规则**：摩擦取几何平均 `sqrt(mu_a * mu_b)`，恢复取最大值。
  摩擦取几何平均是为了让"任何一方是理想冰面则结果为 0"成立（算术平均做不到）；
  恢复取最大值是为了让"弹力球砸到不弹的地面照样弹"这个设定生效。
  两条规则的共同点是**让极端材质的意图能够生效**。
- **`MassProperties` 的验证方式**：惯量公式里那串系数（2/5、1/12、3/8…）单看数字
  无法判断对错，所以测试用**蒙特卡洛数值积分**（在形状里撒 40 万个点按定义累加）
  做独立对拍。这是唯一真正独立的判据 —— 和闭式公式没有任何共享代码。

**角色控制器 `CharacterController`（与刚体完全分离）** —— ✅ 已完成（M8）

> 实现：`include/pe/character/CharacterController.h`、`src/character/CharacterController.cpp`
> 依赖的形状扫掠：`include/pe/collision/ShapeCast.h`、`src/collision/ShapeCast.cpp`

- 状态：`position`、`velocity`、`isGrounded`、`groundNormal`。
- 每帧流程：施加重力与输入 -> **胶囊扫掠（collide-and-slide，最多 4 次迭代）**
  -> 地面检测（向下短扫掠，坡度 <= `maxSlopeAngle` 才算可站立地面）
  -> 台阶处理（`stepOffset` 高度内自动抬升）。
- 滑动的数学：撞墙后把剩余位移投影到墙面切平面 `v' = v - (v . n) * n`，
  用剩余位移继续下一次扫掠。这就是 FPS 贴墙滑行手感的来源。
- **不参与** solver，不受冲量影响。它可以单向推动动态刚体（对刚体施加冲量），
  但动态刚体不能反推角色 —— 这是 CS 类游戏的标准手感约定。

**实现时踩到的三个坑（M8），每一个都有专门的用例守着**：

1. **贴着地面时扫掠会返回垃圾法线**。保守推进靠"两个最近点的连线"定方向，
   而站在地面上的角色与地面**距离恰好为 0**，两点重合、方向退化。
   原来的代码退回用位移方向当法线，于是一个朝前走的角色会得到一个指向**后方**的
   "墙面"法线，被自己脚下的地面当成墙挡住，一步也走不动。
   改成用 `CollideConvex` 取法线（推测性接触区间里它给的是真正的表面法线），
   并补上一条规则：**已经贴着、且位移不朝表面里去时，不算命中** ——
   这是角色能沿着地面走路的前提。

2. **台阶落点不能用 `IsWalkable` 判**。胶囊底部是半球，落向台阶时最先碰到的是
   台阶的**棱**；对 0.3 米台阶配 0.4 米半径的胶囊，棱法线与竖直方向约 74 度，
   远超 `maxSlopeAngle`，于是每一次上台阶都被否掉。
   放宽成"这个面朝上"（`Dot(n, up) > 0`）即可 —— 垂直墙面（0）和天花板（负）
   仍然过不了这一关，而落在棱上是安全的（位置是扫掠出来的，一定不穿透），
   站不站得稳交给随后的 `UpdateGrounded` 判断，站不稳自然滑下来。

3. **陡坡上不能靠"滑"往上爬**。撞到站不住的斜面时，把水平输入投影到坡面上会得到
   一个**向上**的分量，角色会顺着 70 度的悬崖一路滑上去，`maxSlopeAngle` 形同虚设。
   去掉这个向上分量即可，但条件必须写成 `0 < Dot(n, up) < cos(maxSlope)`：
   排除垂直墙面（否则"靠着墙就跳不起来"），也排除天花板。

**关于 `ICharacterWorld`**：角色控制器按 §2 的分层规定不依赖 scene 层，
所以它不直接持有 `PhysicsWorld`，而是声明一个只有扫掠查询的窄接口，由 M9 的
`PhysicsWorld` 实现。好处在测试里体现得很直接 —— 给它一个二十行的假世界
（几个静态盒子）就能测，不需要把刚体、求解器、宽相位全拉起来。

### 4.4 碰撞响应 `solver`（模块 4）—— ✅ 已完成（M7）

> 实现：`include/pe/solver/{IConstraint,ContactConstraint,SequentialImpulseSolver}.h`、
> `src/solver/{ContactConstraint,SequentialImpulseSolver}.cpp`

**Sequential Impulse（顺序冲量）**，每帧固定迭代次数（默认速度迭代 8 次）。

法向约束：要求接触点处的**相对法向速度**非负（两物体不再互相靠近）：

```
Cdot = (vB + wB x rB - vA - wA x rA) . n >= 0
```

施加法向冲量 `lambda` 使其满足。有效质量（约束空间的等效质量倒数）：

```
k = invMassA + invMassB
  + n . [ (invIA * (rA x n)) x rA ]
  + n . [ (invIB * (rB x n)) x rB ]

lambda = -(Cdot + bias) / k
```

其中 `bias` 含两项：

1. **Baumgarte 位置修正** `-(beta / dt) * max(0, penetration - slop)`，beta 约 0.2，
   slop 约 0.005 m。留 slop 是为了避免物体在完美接触处高频抖动。
2. **恢复系数项** `e * v_rel_initial`，只在接近速度超过阈值时启用，
   否则静止物体会因数值噪声永远微弹。

**累积冲量钳制（关键技巧，容易写错）**：不能直接把每次迭代算出的 `lambda` 钳制到
非负，必须钳制**累积值**：`lambda_total_new = max(0, lambda_total_old + lambda)`，
然后只施加两者的差值。否则多次迭代之间会互相抵消，堆叠永远收敛不了。

摩擦：沿两个切向各解一次，用**库仑锥**近似为方盒钳制 `|lambda_t| <= mu * lambda_n`
（mu 取两侧材质的几何平均）。切向正交基由法线用 Erin Catto 的无分支构造法生成，
避免法线接近 +/-Y 时的退化。

`IConstraint { Prepare(dt) / WarmStart() / SolveVelocity() }` 抽象已就位，
未来加铰链、距离、绳索约束只是新增实现类，求解循环本身不动。

**实现时的补充与实测（M7）**：

- **接触约束刻意不继承 `IConstraint`**。一帧里接触有几百上千个、关节只有个位数，
  让接触也走虚函数，每个接触点每次迭代都要付一次间接跳转（8 轮 × 1000 点 × 3 个方向），
  而且虚函数会挡住向量化。所以按**数量分层**：多而同质的（接触）走具体类型的紧凑循环，
  少而异质的（关节）走多态。两者在求解循环里并列，迭代次数与顺序完全一致。
- **摩擦解完之后要按更新过的法向冲量重新收一次锥**。原来的顺序是"先摩擦（用上一轮的
  法向冲量当上限）后法向"，但法向冲量在本轮可能变小甚至归零，于是会留下
  "法向压力为零、却还有摩擦力"的接触点 —— 那是一个凭空产生的切向力，
  实测让一摞静止的箱子横向漂移。补一遍重新钳制（**差值照常施加**，
  只改累加器会让账目对不上）之后，5 秒的横向偏移从 0.44 米降到 0.24 米。
- **warm starting 不是优化，是承重结构**。实测同一座 10 层箱塔、8 轮迭代：
  开着 warm starting 顶层稳在 9.48 米（初始 9.59）；关掉之后**整座塔在 5 秒内
  彻底塌掉**，顶层落到 0.65 米。
  （量它的时候别看"最后一帧的穿透量"—— 塔塌了之后箱子摊平在地上，穿透反而变小，
  看上去像是没什么区别。塌掉的证据是高度。）
- **迭代次数的实测标定**（10 层箱塔，5 秒）：

  | velocityIterations | 结果 |
  |---|---|
  | 4 | 约 7 秒时**塌掉** |
  | 8（默认） | 稳定，顶层横向摇晃 ±0.24 米 |
  | 16 | 稳定，摇晃 ±0.10 米 |
  | 32 | 稳定，摇晃 ±0.13 米（收益已经饱和） |

  摇晃是**来回晃**不是单向漂移（分别在 1/3/5/7/9 秒采样，符号来回变），
  属于 Gauss-Seidel 收敛不足的固有表现。8 轮是 10 层塔的下限，
  真要堆更高（20 层）就得上 16 轮。

### 4.5 射线检测 `RayCast`（模块 5）—— ✅ 形状级已完成（World 级查询待 M8）

> 实现：`include/pe/collision/{Ray,RayCast,GeometryUtil}.h`、`src/collision/RayCast.cpp`
> 测试：`tests/test_raycast.cpp`
>
> **两条硬约定**（写在 `Ray.h` 顶部，有测试守着）：
> 1. `Ray::direction` 永远是单位向量（构造函数保证）。于是返回的 `distance`
>    直接就是米，比较射程、排序命中点都不需要换算。
> 2. **射线起点在物体内部时，返回命中、distance = 0、法线取 -direction。**
>    另一种常见约定是"起点在内部就算不中"。这里选前者是出于 FPS 的安全性：
>    玩家贴墙开枪、枪口陷进墙里时，"立即命中"让子弹停在原地；
>    "不中"则会让子弹穿墙打到墙后的人 —— 那是能被玩家利用的 bug。
>
> 测试策略值得记一笔（后续模块沿用）：除了定点用例，还做了
> **交叉验证**（halfHeight=0 的胶囊 vs 球；旋转 90° 的 OBB vs 交换轴的 AABB；
> 分派 vs 直接调用）和**随机不变量测试**（命中点必须真的落在表面上，
> 即"到胶囊中轴线段的距离 == 半径"）。随机射线用瞄准式采样而非纯随机方向，
> 否则命中率太低，测试会变成空跑。

- `Ray{origin, direction(单位), maxDistance}` ->
  `RaycastHit{hit, t, point, normal, collider, body}`。
- 各形状闭式解：球（解二次方程）、AABB（slab 法）、OBB（变换到局部空间后走 slab）、
  胶囊（射线-线段最近距离 + 两端球）、凸包（射线对各面做半空间裁剪求区间交）。
- World 层：宽相位用 **3D DDA（Amanatides-Woo 体素步进）** 沿射线遍历网格单元，
  按 t 递增顺序访问 —— 第一个命中即可提前退出，`RaycastClosest` 因此非常快。
- 三个查询 API：`RaycastClosest` / `RaycastAny`（视线遮挡判定用，最快）/ `RaycastAll`。
- **hitscan 子弹**用射线即可，不存在穿墙问题；**抛射物**（手雷、火箭）走离散模拟，
  高速时的隧穿问题在 `UPGRADE_NOTES.md` 里标为 CCD 升级点（预留 `ShapeCast` 接口）。

### 4.6 触发器 `TriggerSystem`（模块 6）—— ✅ 已完成（M8）

> 实现：`include/pe/scene/{Events,TriggerSystem}.h`、`src/scene/TriggerSystem.cpp`

- Collider 上一个 `isTrigger` 标志。窄相位对它们**只判定重叠、不生成约束**。
- 维护「上一帧重叠集合」`unordered_set<pair>`，与本帧做差集：
  新增 -> `OnTriggerEnter`，交集 -> `OnTriggerStay`，减少 -> `OnTriggerExit`。
- 事件**不在检测过程中直接回调**，而是压入 `EventQueue`，在 `Step()` 末尾统一派发。
  原因：回调里如果创建/销毁物体，会让正在遍历的容器失效。
- 用途：拾取区、买枪区、炸弹安放区、伤害区。

### 4.7 空间加速结构（模块 7）—— ✅ 已完成（并入 M3）

已并入 4.2 的宽相位。`cellSize` 可在 `WorldConfig` 里调（M8 接上）。
**静态几何单独一张网格**（构建一次永不更新），动态体一张网格（每帧更新），
查询时两张都查 —— 这个拆分能省掉绝大部分静态体的更新开销，对 FPS 地图
（静态体远多于动态体）收益很大。配对时只以动态表的格子为起点，所以静态-静态
对连产生的机会都没有。

实现里还多了第三条路径：**超大体列表**。跨越超过 4096 个格子的物体（地面、
天空盒）不进哈希表，配对与查询时对它们做暴力 AABB 测试。没有这条路径的话，
一块 200×200 米的地面在 0.5 米的网格里要往哈希表塞 16 万个条目，而且此后每一次
射线查询扫到的每个格子都会重复命中它。

### 4.8 物理世界 `PhysicsWorld`（模块 8）—— ✅ 已完成（M9）

> 实现：`include/pe/scene/PhysicsWorld.h`、`src/scene/PhysicsWorld.cpp`、
> `include/pe/collision/Collider.h`

```cpp
class PhysicsWorld {
public:
    explicit PhysicsWorld(const WorldConfig& cfg);

    BodyHandle     CreateBody(const BodyDesc&);
    void           DestroyBody(BodyHandle);
    ColliderHandle AddCollider(BodyHandle, const ColliderDesc&);
    RigidBody*     GetBody(BodyHandle);          // 句柄失效时返回 nullptr

    void Step(real deltaTime);                   // 主循环，内部固定步长累加器

    bool Raycast(const Ray&, RaycastHit& out, uint32_t layerMask = ~0u) const;
    void OverlapAABB(const AABB&, std::vector<ColliderHandle>& out) const;

    void SetEventListener(IPhysicsEventListener*);   // 碰撞 / 触发器回调
    Transform GetInterpolatedTransform(BodyHandle, real alpha) const;
};
```

**`Step(dt)` 内部顺序**（固定步长 1/60，累加器 + 每帧最多 4 次子步防「螺旋死亡」）：

```
1.  派发上一帧遗留的事件
2.  同步 Collider 世界 AABB -> 更新宽相位 proxy
3.  宽相位：QueryPairs() -> 候选对（已过滤 + 去重）
4.  窄相位：分派 -> Manifold 数组；isTrigger 的分流到触发器重叠集
5.  流形持久化：与上一帧按 featureId 匹配，继承累积冲量
6.  积分速度：重力、外力、阻尼（半隐式欧拉 阶段 A）
7.  求解器：Prepare -> WarmStart -> N 次速度迭代（法向 + 摩擦）
8.  积分位置：用求解后的速度更新 position / rotation（阶段 B）
9.  更新 invInertiaWorld、休眠状态；清零 force / torque
10. 触发器差集计算 -> 事件入队 -> 派发
```

> 注意步骤 6 排在 3-5 之后：碰撞检测用的是**上一帧末的位置**。这是离散引擎的标准
> 做法，也正是隧穿的根源，对应 CCD 升级点。

**实现时的补充（M9）**：

- **CCD 接在步骤 8 里**（M8 留下的 P0，已关闭）。只对"这一步位移超过自身包围球
  半径"的刚体做形状扫掠，命中就把位置截断到接触点，**不改速度** ——
  下一步的离散检测会看到接触，由求解器正常处理弹跳与摩擦。
  在这里动速度会让碰撞响应分裂成两套逻辑。
  实测：300 m/s 的球撞 20 厘米厚的墙，开 CCD 被拦在墙前，关掉直接穿过去飞走。
- **fat AABB 的速度预测**（M3 留下的 P0，已关闭）。踩了一个坑：一开始是
  World 自己把 AABB 沿速度扫掠好再传给 `Update()`，**完全无效** ——
  下一帧传进去的"已扫掠 AABB"同样朝前伸出一截，永远装不进上一帧的壳，
  于是每帧都重建。两种写法实测重建率都是 94%，一模一样地没用。
  正确做法是把预测量作为**单独的参数**传进 `Update()`：判定用紧致 AABB，
  预测只加在新建的壳上。改对之后重建率掉到个位数。
- **唤醒时机**：只在接触**刚刚建立**时唤醒，不能"有接触就唤醒"——
  后者会让静止在地面上的物体每帧被叫醒，休眠计时器永远清零，整个休眠机制失效。
- **一方醒着、另一方睡着时必须叫醒睡着的那个**。不叫醒会出一个很隐蔽的错：
  求解器只看 `invMass`，它会照常给睡着的物体施加冲量改速度，但
  `IntegratePosition` 对睡着的物体直接返回 —— 那些冲量凭空消失，接触约束等于失效。
  实测症状是一摞 8 层的箱子塌成一坨、穿透深到 0.7 米。
  代价是**成堆的物体睡不着**（A 睡了被 B 叫醒、B 睡了被 A 叫醒，来回拉锯），
  真正的解法是求解器孤岛，记在 UPGRADE_NOTES 里。
- **复合质心**：一个刚体挂多个碰撞体时，`RigidBody::position` 存的是**复合质心**，
  而用户设定/读取的是**物体原点**，两者由 `centerOfMassLocal` 换算。
  加一个偏心碰撞体时必须同步挪动 `position`，否则物体会自己跳一下 ——
  而且跳的方向毫无规律，非常难查。

---

## 5. 实现与验证计划（按此顺序交付）

| # | 里程碑 | 交付物 | 验收测试 |
|---|---|---|---|
| M1 ✅ | 数学库 + core | Vec/Mat/Quat/Transform/Handle/SlotArray | 四元数与矩阵往返、slerp、句柄代际失效 —— **52 用例 / 293 断言全过** |
| M2 ✅ | 形状 + AABB + 射线求交 | Shape/AABB/Ray/RayCast/GeometryUtil | 每种形状的命中、不命中、切线边界；交叉验证 + 随机不变量 —— **101 用例 / 10296 断言全过** |
| M3 ✅ | 宽相位 Uniform Grid | BroadPhase（IBroadPhase + 共用过滤）/ UniformGrid + DDA 射线遍历 | 与暴力 O(n^2) 对拍：配对 / AABB / 射线三类查询、随机 1000 体、30 帧增删改搅动、四种 cellSize 结果一致 —— **116 用例 / 11865 断言全过** |
| M4 ✅ | 窄相位解析解 | Manifold + NarrowPhase（6 个形状对 + 分派 + Overlap） | 已知穿透深度的构造用例、法线方向约定、退化胶囊 ≡ 球的交叉验证、"沿法线推开穿透深度后必须分离"与"有公共点就必须报接触"两条随机不变量 —— **127 用例 / 29295 断言全过** |
| M5 ✅ | GJK + EPA | GJK 距离查询 + EPA 穿透深度 + `CollideConvex` / `ConvexDistance` 通用入口（ConvexHull 形状本身待顶点池有归属，见 §4.2） | 与解析解对拍：盒-盒两条路径的 MTV 一致（面/边偏置带来的差值有解析上界）、所有形状对的深度与法线一致、通用路径自己的"推开必须分离"不变量、退化维数的闵可夫斯基差 —— **135 用例 / 63492 断言全过** |
| M6 ✅ | 刚体 + 积分器 | RigidBody/MassProperties/Material/Integrator | 自由落体误差随步长线性收敛（确认一阶）、无外力自由旋转的角动量守恒（盯陀螺项）、惯量张量与蒙特卡洛积分对拍、显式 vs 半隐式的能量对照、阻尼与步长无关 —— **151 用例 / 63620 断言全过** |
| M7 ✅ | 顺序冲量求解器 | IConstraint/ContactConstraint/SequentialImpulseSolver + 接触缓存（warm starting） | 弹跳高度符合 e² 关系；10 层盒堆 5 秒不下沉、不溃散；warm starting 开关对照（关掉整塔塌）；斜坡摩擦符合 mu vs tan(theta)；自由对撞动量守恒；深度穿透不弹飞 —— **170 用例 / 78550 断言全过** |
| M8 ✅ | 角色控制器 + 触发器 | ShapeCast（保守推进）+ CharacterController + TriggerSystem/Events | 扫掠接触位置对解析解、高速穿墙被抓住、贴墙滑行、台阶上得去 / 太高上不去、25 度坡能走 70 度坡不能、跳跃高度、卡墙里能脱困、Enter/Stay/Exit 时序与销毁补发 Exit —— **193 用例 / 80109 断言全过** |
| M9 ✅ | World 整合 + Demo | Collider + PhysicsWorld（固定步长 / CCD / 复合质量 / 查询 / 事件 / 调试绘制）+ demo | 生命周期与句柄失效、复合质心与惯量、固定步长与帧率无关、死亡螺旋、CCD 开关对照（关掉直接穿墙）、速度预测把宽相位重建率从 94% 压到个位数、射线/扫掠/重叠查询、事件在 Step 末尾派发且回调里增删物体安全、101 个刚体跑 4 秒不发散 —— **212 用例 / 80186 断言全过** |

**Demo（M9）**：一个胶囊角色，在由 6-8 个 AABB 组成的静态地图（地面、几面墙、
一段台阶、一个斜坡）里用 WASD 移动；场景中放几个可推动的动态箱子；
点击发射 hitscan 射线并打印命中的物体与命中点。
无渲染依赖 —— 先做**控制台俯视图可视化 + 逐帧状态转储**，同时提供
`GetDebugDrawData()` 供你接入现有渲染层。

---

## 6. 与渲染层的对接约定

- 引擎不持有任何渲染资源。每帧 `Step` 之后，渲染层遍历句柄取
  `RigidBody::position / rotation`，自行转成 `Mat4`。
- `GetInterpolatedTransform(handle, alpha)`：固定步长物理 + 可变帧率渲染**必须**做
  插值，否则画面会有规律性抖动（alpha = 累加器余量 / 固定步长）。
- `IDebugDraw` 接口（画线 / 画 AABB / 画接触点与法线）——
  调试碰撞问题时这是最重要的工具，第一版就做。

---

## 7. 明确不做的事（避免范围蔓延）

软体、布料、流体、车辆、破碎、多线程、手写 SIMD、网络回滚同步。
这些若将来要做，见 `UPGRADE_NOTES.md` 的分级清单。
