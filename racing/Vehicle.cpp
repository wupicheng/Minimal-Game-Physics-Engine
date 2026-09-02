//==============================================================================
// racing/Vehicle.cpp
//==============================================================================

#include "racing/Vehicle.h"

namespace racing {

namespace {

//------------------------------------------------------------------------------
// 接触点上沿某个方向的**有效质量**。
//
//     1 / (1/m + n · ((I⁻¹ (r × n)) × r))
//
// 这是求解器里每个约束都要算的那个量，物理含义是"在 r 这一点、沿 n 这个方向，
// 把速度改变 1 m/s 需要多大的冲量"。
//
// 为什么轮胎必须用它，而不能图省事写 `质量 / 4`：
// 侧向力作用在离质心很远的接触点上，它同时改变平动和转动。用总质量的四分之一
// 去算的话，靠前的轮子会被推过头、靠后的推不够，车在直线上都会自己扭。
// 用有效质量算出来的冲量，**恰好**让那一点的侧向速度归零 —— 一次到位，不震荡。
//------------------------------------------------------------------------------
real EffectiveMass(const RigidBody& body, const Vec3& r, const Vec3& n) noexcept {
    const Vec3 rn = Cross(r, n);
    const real denom = body.invMass + Dot(n, Cross(body.invInertiaWorld * rn, r));
    return denom > real(1e-9) ? real(1) / denom : real(0);
}

/// 把向量投影到以 normal 为法线的平面上并归一化。轮胎的前进方向必须贴着路面，
/// 否则上坡时驱动力会有一个往地里插的分量。
Vec3 ProjectOnPlane(const Vec3& v, const Vec3& normal) noexcept {
    const Vec3 projected = v - normal * Dot(v, normal);
    const real length = projected.Length();
    return length > real(1e-6) ? projected * (real(1) / length) : Vec3::Zero();
}

}  // namespace

//==============================================================================
// 创建
//==============================================================================

void Vehicle::Spawn(PhysicsWorld& world, const VehicleConfig& config,
                    const Vec3& position, real yaw) {
    m_config = config;

    //--------------------------------------------------------------------------
    // 四个轮子挂在车身四角，稍微往内收一点（真车的轮子也不在保险杠外面）。
    // 前轮转向、后轮驱动 + 手刹 —— 后驱是最好开也最容易漂的布局。
    //--------------------------------------------------------------------------
    //--------------------------------------------------------------------------
    // **悬挂上端点在质心上方**（attachY > 0），这不是笔误。
    //
    // 车翻不翻，由一个比值决定：
    //     翻车阈值(g) = 半轮距 / 质心离地高度
    // 轮胎能给的横向加速度超过它，车就是先翻、不是先滑。
    //
    // 第一版把上端点放在车身底部（attachY < 0），于是质心离地 0.98 米、
    // 半轮距 0.79 米 —— 阈值只有 0.8 g，而轮胎能给 2.0 g。结果是每个弯都翻，
    // 而且怎么加防倾杆都救不回来：差了两倍半。
    //
    // 把上端点提到质心上方之后，质心离地 0.45 米、半轮距 0.84 米，阈值 1.85 g，
    // 高于轮胎的 1.55 g —— 车在极限时**先滑后翻**，这才是赛车该有的行为。
    // 真车也是这么布置的：减震塔在质心上方，发动机和乘员在下方。
    //--------------------------------------------------------------------------
    const real halfTrack = config.halfExtents.x - real(0.06);
    const real wheelBase = config.halfExtents.z - real(0.35);
    const real attachY = real(0.28);

    m_wheelConfig[0] = {Vec3(-halfTrack, attachY, wheelBase), true, false, false};
    m_wheelConfig[1] = {Vec3(halfTrack, attachY, wheelBase), true, false, false};
    m_wheelConfig[2] = {Vec3(-halfTrack, attachY, -wheelBase), false, true, true};
    m_wheelConfig[3] = {Vec3(halfTrack, attachY, -wheelBase), false, true, true};

    BodyDesc desc;
    desc.position = position;
    desc.rotation = Quat::FromAxisAngle(Vec3(0, real(1), 0), yaw);
    desc.type = BodyType::Dynamic;
    desc.mass = config.mass;
    // 阻尼交给我们自己的空气动力模型，引擎的线性阻尼调到几乎不管事 ——
    // 两套减速叠在一起会让"松油门滑行多远"完全不可预测
    desc.linearDamping = real(0.0);
    desc.angularDamping = real(0.35);
    m_body = world.CreateBody(desc);

    ColliderDesc collider;
    collider.shape = Shape::MakeBox(config.halfExtents);
    // 车壳很滑：撞墙应该是**蹭着滑走**，不是黏在墙上。摩擦留给轮胎去做，
    // 车身自己的摩擦只会制造"贴着墙就停住"这种莫名其妙的手感。
    collider.material = Material(real(0.15), real(0.1));
    collider.layer = kLayerCar;
    world.AddCollider(m_body, collider);

    //--------------------------------------------------------------------------
    // 下面这些刚体全都是**纯装饰**：轮子和车壳。
    //
    // 它们不参与任何物理：`layerMask = 0` 表示"我不和任何东西碰撞"
    // （引擎的配对判定是双向的，见 BroadPhase.h）。但渲染射线用的是
    // 单向的 `layer & mask`，所以它们照样看得见。
    //
    // 这是分层最漂亮的一个用法：同一个物体，在碰撞里不存在，在画面里存在。
    // 碰撞用的还是那个干净的盒子 —— 商业引擎里也是这么分的：碰撞代理永远
    // 比渲染模型简单，尾翼不需要参与碰撞。
    //--------------------------------------------------------------------------
    const auto addDecor = [&](BodyHandle body, const Shape& shape, const Vec3& offset,
                              const Quat& rotation) {
        ColliderDesc decor;
        decor.shape = shape;
        decor.localTransform = Transform(offset, rotation);
        decor.layer = kLayerDecor;
        decor.layerMask = 0;  // 不和任何东西发生碰撞
        world.AddCollider(body, decor);
    };

    //--------------------------------------------------------------------------
    // 轮子：一块**沿车身左右方向很薄**的板，立在前进方向上。
    //
    // 之前是球。球从任何角度看都是个圆点，既看不出朝向、又和车一样宽 ——
    // 画面上就是四个黑球，分不清是在滚还是横着蹭。**"薄"比"圆"重要得多**：
    // 轮子之所以一眼能认出来，靠的是"它在车侧面立着、很窄"这个比例。
    //
    // 引擎只有球 / 胶囊 / 盒，没有圆柱，而这三个都做不出"又薄又圆"：
    //   - 胶囊沿轴向最窄也有 2*radius（0.6 米），做出来是个滚筒
    //   - 胶囊 halfHeight 收到 0 就退化成球，等于没改
    //   - 盒子的并集只会**更尖**（两个 45 度交叉的方块并起来是八角星，
    //     不是八边形；八边形要靠交集，而碰撞体只能并不能交）
    // 所以就用一块方板，比例对、朝向对。
    //
    // **而且它不跟着自转。** 试过让它绕轮轴转（`spinAngle`），结果是画面上
    // 四个菱形在原地转 —— 方块转起来看着就是方块在转，比不转更假。
    // 滚动感由车在动本身给，轮子只负责"在正确的位置、以正确的姿态立着"。
    //--------------------------------------------------------------------------
    const real tireHalfWidth = real(0.13);
    const real r = config.wheelRadius;

    for (int i = 0; i < kWheelCount; ++i) {
        BodyDesc wheelDesc;
        wheelDesc.position = position;
        wheelDesc.type = BodyType::Kinematic;
        m_wheelBodies[i] = world.CreateBody(wheelDesc);

        addDecor(m_wheelBodies[i], Shape::MakeBox(Vec3(tireHalfWidth, r, r)),
                 Vec3::Zero(), Quat::Identity());

        m_wheels[i] = WheelState{};
    }

    //--------------------------------------------------------------------------
    // 车壳：一个跟着底盘走的运动学刚体，挂几块盒子拼出车的轮廓。
    //
    // 物理上车永远是**一个盒子**（上面那个碰撞体），这一层只是让它看起来像车：
    // 车身、机盖、座舱、尾箱、尾翼。座舱往后坐、机盖压低，是最少的、
    // 能让人一眼认出"这是辆车而不是一个箱子"的几块。
    //--------------------------------------------------------------------------
    BodyDesc shellDesc;
    shellDesc.position = position;
    shellDesc.rotation = desc.rotation;
    shellDesc.type = BodyType::Kinematic;
    m_shellBody = world.CreateBody(shellDesc);

    // 车的侧面轮廓（+Z 是车头，y=0 是质心，地面在 y = -0.41）：
    //
    //                    ___尾翼___
    //          ______屋顶______
    //         /前风挡        后风挡\        <- 两块**斜**的板
    //     ___/                     \____
    //    |   机盖        车身        尾箱 |
    //    +----(轮)-------------(轮)-------+
    //
    // 关键是那两块斜板。全用轴对齐的盒子拼，座舱就是个方盒子扣在车身上，
    // 看着像面包车；前风挡一斜，整台车立刻就"是辆跑车"了。
    const Quat none = Quat::Identity();
    const Vec3 axisX(real(1), 0, 0);

    // 车身：从头到尾一整块，上表面就是机盖和尾箱
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.92), real(0.30), real(1.92))),
             Vec3(0, real(0.06), 0), none);
    // 前唇：车头贴地的一小条，压低视觉重心
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.94), real(0.06), real(0.12))),
             Vec3(0, real(-0.16), real(1.95)), none);

    //--------------------------------------------------------------------------
    // 前风挡：从机盖后缘 (z=0.75, y=0.36) 斜上到车顶前缘 (z=0.00, y=0.78)。
    //
    // 盒子默认沿 +Z 方向"长"，绕 +X 转 θ 会把 +Z 端压低：
    //     R_x(θ) * (0,0,1) = (0, -sinθ, cosθ)
    // 要让车头那端低、车尾那端高，θ 取正的 29 度（就是这段斜率的仰角）。
    //
    // 座舱要**几乎和车身一样宽**（0.78 对 0.92）。第一版做窄了一大圈，
    // 侧面看是一大片平车顶上扣着个小盒子，像面包车不像跑车。
    //--------------------------------------------------------------------------
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.78), real(0.06), real(0.43))),
             Vec3(0, real(0.57), real(0.375)),
             Quat::FromAxisAngle(axisX, DegToRad(real(29))));
    // 座舱内腔：填住风挡/车顶/后窗之间那块空的，不然侧面能看穿
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.72), real(0.21), real(0.55))),
             Vec3(0, real(0.57), real(-0.55)), none);
    // 车顶
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.74), real(0.05), real(0.56))),
             Vec3(0, real(0.81), real(-0.55)), none);
    // 后风挡：从车顶后缘斜下到尾箱，方向和前风挡相反，所以角度取负
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.74), real(0.06), real(0.31))),
             Vec3(0, real(0.57), real(-1.325)),
             Quat::FromAxisAngle(axisX, DegToRad(real(-43))));

    // 尾翼：两根立柱 + 翼面。它对"这是辆赛车"的贡献远超过它的体积
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.05), real(0.13), real(0.05))),
             Vec3(real(0.58), real(0.75), real(-1.82)), none);
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.05), real(0.13), real(0.05))),
             Vec3(real(-0.58), real(0.75), real(-1.82)), none);
    addDecor(m_shellBody, Shape::MakeBox(Vec3(real(0.84), real(0.04), real(0.15))),
             Vec3(0, real(0.92), real(-1.82)), none);

    m_position = position;
    m_rotation = desc.rotation;
    m_velocity = Vec3::Zero();
    m_steerAngle = real(0);
}

void Vehicle::Respawn(PhysicsWorld& world, const Vec3& position, real yaw) {
    RigidBody* body = world.GetBody(m_body);
    if (body == nullptr) return;

    const Quat rotation = Quat::FromAxisAngle(Vec3(0, real(1), 0), yaw);
    world.SetBodyTransform(m_body, Transform(position, rotation));
    // 速度必须清干净。留着复位前那一下撞墙的速度，车会在复位点原地弹射出去。
    body->linearVelocity = Vec3::Zero();
    body->angularVelocity = Vec3::Zero();
    body->WakeUp();

    m_position = position;
    m_rotation = rotation;
    m_velocity = Vec3::Zero();
    m_steerAngle = real(0);
    for (WheelState& wheel : m_wheels) wheel = WheelState{};
}

//==============================================================================
// 每个子步
//==============================================================================

void Vehicle::Update(PhysicsWorld& world, const DriveInput& input, real h) {
    RigidBody* body = world.GetBody(m_body);
    if (body == nullptr || h <= real(0)) return;

    //--------------------------------------------------------------------------
    // 车永远不许睡着。
    //
    // 引擎会让长时间没动静的刚体休眠（省下宽相位和求解的开销），而休眠的刚体
    // 不被积分 —— 施力全部石沉大海。停在起跑线上等发车的车正好满足休眠条件，
    // 于是"踩油门没反应"。比赛期间它就该一直是活跃的。
    //--------------------------------------------------------------------------
    body->WakeUp();

    m_position = body->position;
    m_rotation = body->rotation;
    m_velocity = body->linearVelocity;

    //-- 方向盘：往目标角度靠，不瞬间到位 ---------------------------------------
    const real speed = Speed();
    const real falloff =
        Clamp(speed / Max(m_config.steerFalloffSpeed, real(1)), real(0), real(1));
    const real steerLimit =
        m_config.maxSteerAngle *
        (real(1) + (m_config.highSpeedSteerScale - real(1)) * falloff);
    const real target = Clamp(input.steer, real(-1), real(1)) * steerLimit;
    const real rate = (m_config.maxSteerAngle * real(2)) /
                      Max(m_config.steerLockTime, real(0.01));
    m_steerAngle += Clamp(target - m_steerAngle, -rate * h, rate * h);

    UpdateSuspension(world, *body, h);
    UpdateAntiRollBars(*body);
    UpdateTireForces(*body, input, h);
    UpdateAerodynamics(*body, input, h);
    SyncVisualBodies(world, h);
}

//------------------------------------------------------------------------------
// 悬挂：一条射线 + 一个弹簧阻尼器
//------------------------------------------------------------------------------
void Vehicle::UpdateSuspension(PhysicsWorld& world, RigidBody& body, real h) {
    (void)h;
    const Vec3 up = Up();
    const Vec3 down = -up;
    const real maxLength = m_config.suspensionRest + m_config.suspensionTravel;

    for (int i = 0; i < kWheelCount; ++i) {
        WheelState& wheel = m_wheels[i];
        const Vec3 attach = m_position + m_rotation.Rotate(m_wheelConfig[i].attach);

        //----------------------------------------------------------------------
        // 射线从悬挂上端点往车底方向打，长度 = 最长悬挂 + 轮子半径。
        // 掩码里没有车身也没有轮子，否则第一个命中永远是自己（见 RacingTypes.h）。
        //----------------------------------------------------------------------
        const real rayLength = maxLength + m_config.wheelRadius;
        WorldRaycastHit hit;
        const bool grounded =
            world.Raycast(Ray(attach, down, rayLength), hit, kSuspensionMask);

        wheel.grounded = grounded;
        wheel.steerAngle = m_wheelConfig[i].steered ? m_steerAngle : real(0);

        if (!grounded) {
            // 悬空：悬挂自然伸到最长，轮子挂在下面
            wheel.suspensionLength = maxLength;
            wheel.compression = real(0);
            wheel.normalForce = real(0);
            wheel.slip = real(0);
            wheel.center = attach + down * maxLength;
            wheel.contactNormal = up;
            continue;
        }

        const real length =
            Clamp(hit.distance - m_config.wheelRadius, real(0), maxLength);
        wheel.suspensionLength = length;
        wheel.compression = m_config.suspensionRest - length;
        wheel.center = attach + down * length;
        wheel.contactPoint = hit.point;
        wheel.contactNormal = hit.normal;

        //----------------------------------------------------------------------
        // 弹簧 + 阻尼。
        //
        // 阻尼用的是**接触点处沿悬挂方向的速度**，不是车身质心的速度：
        // 过弯侧倾时，外侧轮在压缩、内侧轮在伸张，而质心的竖直速度可能是零。
        // 用质心速度算的话，阻尼在过弯时就完全不起作用了。
        //----------------------------------------------------------------------
        const real spring = m_config.suspensionStiffness * wheel.compression;
        const real suspensionVelocity = Dot(body.VelocityAtPoint(hit.point), up);
        const real damper = -m_config.suspensionDamping * suspensionVelocity;

        // 悬挂只能**推**，不能拉：轮子离地之后就不该再有力了
        wheel.normalForce = Max(real(0), spring + damper);

        // 用力而不是冲量：它是持续作用的（见 RigidBody.h 里那句判据）。
        // 加在**接触点**上而不是质心：偏心的力矩正是车过弯侧倾、刹车点头的来源。
        body.ApplyForceAtPoint(up * wheel.normalForce, hit.point);
    }
}

//------------------------------------------------------------------------------
// 防倾杆
//
// 把左右轮的压缩差转成一对反向的力：外侧被压得多，就抬外侧、压内侧。
// 真车上是一根扭杆，这里就是一行减法。
//
// **它是"车过弯不翻"的关键**。没有它，2.0 的侧向抓地力会在质心处产生一个
// 远大于轮距能扛住的侧倾力矩，任何一个快弯都是翻车。加大轮距或者压低重心
// 也能解决，但那会让车看起来像块砖。
//------------------------------------------------------------------------------
void Vehicle::UpdateAntiRollBars(RigidBody& body) {
    const Vec3 up = Up();
    const int pairs[2][2] = {{0, 1}, {2, 3}};  // 前轴、后轴

    for (const auto& pair : pairs) {
        const WheelState& left = m_wheels[pair[0]];
        const WheelState& right = m_wheels[pair[1]];
        // 有一边悬空就不作用 —— 那时候扭杆只会把着地的那侧也顶起来
        if (!left.grounded || !right.grounded) continue;

        const real force =
            m_config.antiRollStiffness * (left.compression - right.compression);
        body.ApplyForceAtPoint(up * -force, left.contactPoint);
        body.ApplyForceAtPoint(up * force, right.contactPoint);
    }
}

//------------------------------------------------------------------------------
// 轮胎力：摩擦圆
//------------------------------------------------------------------------------
void Vehicle::UpdateTireForces(RigidBody& body, const DriveInput& input, real h) {
    int drivenGrounded = 0;
    for (int i = 0; i < kWheelCount; ++i) {
        if (m_wheelConfig[i].driven && m_wheels[i].grounded) ++drivenGrounded;
    }

    const real forwardSpeed = ForwardSpeed();
    // 接近极速时驱动力线性收到零。硬夹一个速度上限会让车"撞到一堵看不见的墙"，
    // 收力则是自然的"推不动了"。
    const real speedFade =
        Clamp(real(1) - Abs(forwardSpeed) / Max(m_config.maxSpeed, real(1)), real(0),
              real(1));

    // 停住之后继续踩刹车 = 倒挡。单独一个倒挡键在键盘上很别扭。
    const bool reversing = input.brake > real(0.1) && forwardSpeed < real(0.6);

    for (int i = 0; i < kWheelCount; ++i) {
        WheelState& wheel = m_wheels[i];
        if (!wheel.grounded || wheel.normalForce <= real(0)) {
            wheel.slip = real(0);
            continue;
        }

        const WheelConfig& cfg = m_wheelConfig[i];
        const Vec3 normal = wheel.contactNormal;

        //----------------------------------------------------------------------
        // 轮胎坐标系：前进方向（含转向角）和侧向，都贴着路面
        //----------------------------------------------------------------------
        const Quat steer = Quat::FromAxisAngle(Up(), wheel.steerAngle);
        const Vec3 forward = ProjectOnPlane(steer.Rotate(Forward()), normal);
        if (forward.LengthSq() < real(0.5)) continue;  // 几乎垂直的面，放弃
        const Vec3 side = Cross(normal, forward);

        const Vec3 r = wheel.contactPoint - body.position;
        const Vec3 velocity = body.VelocityAtPoint(wheel.contactPoint);
        const real vSide = Dot(velocity, side);
        const real vForward = Dot(velocity, forward);

        //----------------------------------------------------------------------
        // 摩擦圆：这一轮这一步最多能给出多大的冲量。
        //
        //     |冲量| <= μ * 法向力 * h
        //
        // 侧向和纵向**共用**这一个额度，先满足侧向（不然车会失控地横着走），
        // 剩下的给驱动/刹车。这一条就是"全油门出弯会推头""刹车时打方向刹不住"
        // 这些真实行为的全部来源 —— 没有任何一行代码在专门实现它们。
        //----------------------------------------------------------------------
        // 前轮抓地力低于后轮 —— 见 VehicleConfig 里那段关于转向不足的说明
        real lateralGrip = cfg.steered ? m_config.gripLateralFront
                                       : m_config.gripLateralRear;
        if (input.handbrake && cfg.handbraked) {
            lateralGrip *= m_config.handbrakeGripScale;
        }
        const real budget = wheel.normalForce * h;
        const real maxLateral = lateralGrip * budget;

        // 侧向：算出"把侧滑速度完全消掉"需要多大冲量，再按摩擦圆裁掉
        const real lateralMass = EffectiveMass(body, r, side);
        const real wantLateral = -vSide * lateralMass;
        const real lateral = Clamp(wantLateral, -maxLateral, maxLateral);
        body.ApplyImpulseAtPoint(side * lateral, wheel.contactPoint);

        // 侧滑程度 = 想要的和给得起的之差。1 表示完全滑开了，HUD 拿它显示打滑
        wheel.slip = Abs(wantLateral) > real(1e-3)
                         ? Clamp(real(1) - Abs(lateral) / Abs(wantLateral), real(0),
                                 real(1))
                         : real(0);

        //----------------------------------------------------------------------
        // 纵向：驱动 / 刹车 / 滚阻，用摩擦圆剩下的额度
        //----------------------------------------------------------------------
        const real remaining = Sqrt(Max(
            real(0), m_config.gripLongitudinal * budget * m_config.gripLongitudinal *
                             budget -
                         lateral * lateral));

        real longitudinal = real(0);
        if (cfg.driven && drivenGrounded > 0) {
            if (reversing) {
                longitudinal -= m_config.reverseForce * input.brake * h /
                                static_cast<real>(drivenGrounded);
            } else {
                longitudinal += m_config.driveForce * input.throttle * speedFade * h /
                                static_cast<real>(drivenGrounded);
            }
        }

        // 刹车：只在还往前走的时候作用，否则会把车往后推
        if (!reversing && input.brake > real(0)) {
            const real brake = m_config.brakeForce * input.brake * h /
                               real(kWheelCount);
            longitudinal -= Sign(vForward) * Min(brake, Abs(vForward) *
                                                            EffectiveMass(body, r,
                                                                          forward));
        }
        // 手刹锁死的轮子直接按住
        if (input.handbrake && cfg.handbraked) {
            longitudinal -= Sign(vForward) *
                            Min(m_config.brakeForce * h / real(kWheelCount),
                                Abs(vForward) * EffectiveMass(body, r, forward));
        }
        // 滚动阻力：松油门时车自己慢下来
        if (input.throttle <= real(0.01) && input.brake <= real(0.01)) {
            longitudinal -= Sign(vForward) * m_config.rollingResistance * h /
                            real(kWheelCount);
        }

        longitudinal = Clamp(longitudinal, -remaining, remaining);
        body.ApplyImpulseAtPoint(forward * longitudinal, wheel.contactPoint);

        // 轮子转多快（纯视觉）：贴地滚动就是接触点速度除以半径
        wheel.spinAngle += vForward / Max(m_config.wheelRadius, real(0.01)) * h;
    }
}

//------------------------------------------------------------------------------
// 空气动力 + 腾空姿态
//------------------------------------------------------------------------------
void Vehicle::UpdateAerodynamics(RigidBody& body, const DriveInput& input, real h) {
    const real speed = Speed();

    // 风阻 ∝ v²，方向和速度相反。它决定极速，也决定松油门之后滑行多远。
    if (speed > real(0.1)) {
        const Vec3 drag = m_velocity * (-m_config.dragCoefficient * speed);
        body.ApplyForce(drag);
    }

    //--------------------------------------------------------------------------
    // 下压力 ∝ v²，沿**车顶反方向**压。
    //
    // 它是高速段抓地力的来源：法向力变大 -> 摩擦圆变大 -> 快弯才敢进。
    // 注意方向要用车身的 up 而不是世界的 -Y —— 侧倾时用世界坐标会把车往侧面推。
    //--------------------------------------------------------------------------
    const int grounded = GroundedWheels();
    if (grounded > 0) {
        const real downforce = m_config.downforceCoefficient * speed * speed;
        body.ApplyForce(Up() * -downforce);
    }

    //--------------------------------------------------------------------------
    // 腾空时用方向键调姿态。
    //
    // 现实里飞起来是控不了的，但玩家会本能地想在空中把车摆正 —— 不给的话
    // 每一次跳台都以翻车收场，跳台就从"爽点"变成"惩罚"。这是玩法，不是物理。
    //--------------------------------------------------------------------------
    if (grounded == 0) {
        const Vec3 pitch = Right() * (input.throttle - input.brake) *
                           m_config.airPitchControl;
        const Vec3 roll = Forward() * (-input.steer * m_config.airPitchControl *
                                       real(0.5));
        body.ApplyTorque(pitch + roll);
    }
    (void)h;
}

//------------------------------------------------------------------------------
// 把轮子的位姿抄给那四个装饰刚体
//------------------------------------------------------------------------------
void Vehicle::SyncVisualBodies(PhysicsWorld& world, real h) {
    (void)h;

    // 车壳直接跟着底盘 —— 它就是底盘的"外衣"
    if (m_shellBody.IsValid()) {
        world.SetBodyTransform(m_shellBody, Transform(m_position, m_rotation));
    }

    for (int i = 0; i < kWheelCount; ++i) {
        if (!m_wheelBodies[i].IsValid()) continue;

        //----------------------------------------------------------------------
        // 轮子的朝向 = 车身姿态 + 转向角。
        //
        // 转向绕的是**车顶方向**（世界空间里车身的 up），不是固定的世界 Y ——
        // 写成世界 Y 的话，车一侧倾轮子就会歪到车身外面去。世界空间的旋转
        // 从左边乘。
        //
        // 这里**没有自转**：方块轮子转起来看着就是方块在转，见 Spawn 里的说明。
        //----------------------------------------------------------------------
        const Quat steer = Quat::FromAxisAngle(Up(), m_wheels[i].steerAngle);
        world.SetBodyTransform(m_wheelBodies[i],
                               Transform(m_wheels[i].center, steer * m_rotation));
    }
}

//==============================================================================
// 查询
//==============================================================================

int Vehicle::GroundedWheels() const noexcept {
    int count = 0;
    for (const WheelState& wheel : m_wheels) {
        if (wheel.grounded) ++count;
    }
    return count;
}

real Vehicle::MaxSlip() const noexcept {
    real worst = real(0);
    for (const WheelState& wheel : m_wheels) worst = Max(worst, wheel.slip);
    return worst;
}

}  // namespace racing
