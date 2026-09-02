//==============================================================================
// src/character/CharacterController.cpp
//
// collide-and-slide、地面检测、台阶抬升。设计理由见 CharacterController.h。
//==============================================================================

#include "pe/character/CharacterController.h"

namespace pe {

namespace {

/// 把位移沿平面法线投影掉，剩下贴着平面的那部分：`v - Dot(v, n) * n`
inline Vec3 ProjectOnPlane(const Vec3& v, const Vec3& n) noexcept {
    return v - n * Dot(v, n);
}

}  // namespace

//==============================================================================
// collide-and-slide
//==============================================================================

CharacterMoveResult CharacterController::SlideMove(const ICharacterWorld& world,
                                                   const Vec3& displacement,
                                                   const Vec3& startPosition,
                                                   Vec3& outPosition) const {
    CharacterMoveResult result;
    result.displacement = Vec3::Zero();
    result.collided = false;
    result.collidedSides = false;
    result.collidedBelow = false;
    result.collidedAbove = false;
    result.steppedUp = false;

    const Shape shape = GetShape();
    Vec3 pos = startPosition;
    Vec3 remaining = displacement;

    for (int iter = 0; iter < config.maxSlideIterations; ++iter) {
        const real remainingLength = remaining.Length();
        if (remainingLength <= config.skinWidth * real(0.1)) break;

        ShapeCastHit hit;
        if (!world.SweepCharacter(shape, Transform(pos, Quat::Identity()), remaining,
                                  config.layerMask, hit)) {
            // 畅通，一步走完
            pos += remaining;
            remaining = Vec3::Zero();
            break;
        }

        result.collided = true;

        //----------------------------------------------------------------------
        // 起点就卡在东西里：先脱困，**不消耗**这一次的位移。
        //
        // 这条路径在真实游戏里并不罕见：出生点摆在了墙里、被传送进几何体、
        // 或者上一帧被移动平台夹住。多推一个 skinWidth 是为了脱开之后留出余量，
        // 否则下一轮扫掠又会判定为重叠，原地打转直到迭代用尽。
        //----------------------------------------------------------------------
        if (hit.startPenetrating) {
            pos += hit.normal * (hit.depth + config.skinWidth);
            continue;
        }

        // 撞到的是什么：地面、天花板、还是墙
        if (IsWalkable(hit.normal)) {
            result.collidedBelow = true;
        } else if (Dot(hit.normal, config.up) < -kEpsilon) {
            result.collidedAbove = true;
        } else {
            result.collidedSides = true;
        }

        //----------------------------------------------------------------------
        // 走到接触点前 skinWidth 处。
        //
        // 留这条缝是必须的：走到"恰好接触"的话，浮点误差会让下一帧的重叠判定在
        // 碰与不碰之间抖动，表现为贴着墙时角色微微颤抖。
        //----------------------------------------------------------------------
        const Vec3 direction = remaining * (real(1) / remainingLength);
        const real contactDistance = hit.fraction * remainingLength;
        const real advance = Max(real(0), contactDistance - config.skinWidth);
        pos += direction * advance;

        //----------------------------------------------------------------------
        // 把**没走完**的那部分投影到墙面切平面上，作为下一轮的位移。
        //
        // 这一行就是 FPS 贴墙滑行手感的全部来源：撞墙之后不是停下，
        // 而是沿着墙面把剩下的力气用掉。
        //
        // 注意投影的是"接触点之后剩下的位移"，不是"整段位移"——
        // 用整段的话角色会凭空多走一截。
        //----------------------------------------------------------------------
        remaining = ProjectOnPlane(remaining - direction * contactDistance, hit.normal);

        //----------------------------------------------------------------------
        // 陡坡上不许靠"滑"往上爬。
        //
        // 撞到一个站不住的斜面时，把水平输入投影到坡面上会得到一个**向上**的
        // 分量 —— 于是角色会顺着 70 度的悬崖一路滑上去，maxSlopeAngle 形同虚设。
        // 把这个向上的分量去掉，角色就只能沿着坡面横向或向下走，
        // 也就是"站不住就往下滑"。
        //
        // 条件写成 `0 < Dot(n, up) < cos(maxSlope)` 是有讲究的：
        //   - 排除了**垂直的墙**（Dot 恰为 0）。贴着墙跳的时候必须还能往上，
        //     不然玩家会发现"靠着墙就跳不起来"。
        //   - 排除了**天花板**（Dot 为负），那种情况本来也不会向上。
        // 只针对"朝上但太陡"这一档，正是 maxSlopeAngle 想管的那一档。
        //----------------------------------------------------------------------
        const real slopeDot = Dot(hit.normal, config.up);
        if (slopeDot > kEpsilon && slopeDot < Cos(config.maxSlopeAngle)) {
            const real upward = Dot(remaining, config.up);
            if (upward > real(0)) remaining -= config.up * upward;
        }
    }

    outPosition = pos;
    result.displacement = pos - startPosition;
    return result;
}

//==============================================================================
// Move：collide-and-slide + 台阶
//==============================================================================

CharacterMoveResult CharacterController::Move(const ICharacterWorld& world,
                                              const Vec3& displacement) {
    const Vec3 up = config.up;
    const Vec3 horizontal = ProjectOnPlane(displacement, up);

    //--------------------------------------------------------------------------
    // 先老老实实走一遍
    //--------------------------------------------------------------------------
    Vec3 directPosition = position;
    CharacterMoveResult direct = SlideMove(world, displacement, position, directPosition);

    //--------------------------------------------------------------------------
    // 要不要试台阶
    //
    // 三个条件缺一不可：
    //   - 确实被侧面挡住了（撞到地面或天花板都不该触发抬升）
    //   - 开启了台阶功能
    //   - 确实想往水平方向走（纯竖直移动时抬升毫无意义，而且会让下落变成飘）
    //--------------------------------------------------------------------------
    const real horizontalLength = horizontal.Length();
    if (!direct.collidedSides || config.stepOffset <= real(0) ||
        horizontalLength <= config.skinWidth) {
        position = directPosition;
        return direct;
    }

    const Shape shape = GetShape();

    //--------------------------------------------------------------------------
    // 台阶三步走：抬起来 -> 往前走 -> 落回去
    //--------------------------------------------------------------------------

    // (1) 抬起来。头顶可能有东西，所以是扫掠而不是直接加。
    real liftHeight = config.stepOffset;
    {
        ShapeCastHit hitUp;
        if (world.SweepCharacter(shape, Transform(position, Quat::Identity()),
                                 up * config.stepOffset, config.layerMask, hitUp)) {
            if (hitUp.startPenetrating) {
                // 已经卡住了，台阶这条路走不通，用直接移动的结果
                position = directPosition;
                return direct;
            }
            liftHeight = Max(real(0), hitUp.fraction * config.stepOffset - config.skinWidth);
        }
    }

    // 抬不起来（头顶就是天花板）就没必要继续了
    if (liftHeight <= config.skinWidth) {
        position = directPosition;
        return direct;
    }

    const Vec3 liftedPosition = position + up * liftHeight;

    // (2) 在抬高的位置往前走。只走水平分量 —— 竖直分量已经被"抬起"这一步用掉了。
    Vec3 forwardPosition = liftedPosition;
    const CharacterMoveResult forward =
        SlideMove(world, horizontal, liftedPosition, forwardPosition);

    // (3) 落回去。往下扫最多 liftHeight，看能踩到什么。
    ShapeCastHit hitDown;
    if (!world.SweepCharacter(shape, Transform(forwardPosition, Quat::Identity()),
                              -up * liftHeight, config.layerMask, hitDown) ||
        hitDown.startPenetrating) {
        // 下面是空的（跨过去是个坑），或者落点被卡住 —— 都不接受这次抬升
        position = directPosition;
        return direct;
    }

    //--------------------------------------------------------------------------
    // 落点的判据是"这个面**朝上**"，而不是更严的"这个面能站人"。
    //
    // 为什么不能用 IsWalkable：胶囊的底部是个半球，它落向台阶时最先碰到的是
    // 台阶的**棱**，而棱的法线介于"水平的立面"和"竖直的顶面"之间 —— 对一个
    // 0.3 米的台阶配 0.4 米半径的胶囊，棱法线与竖直方向的夹角约 74 度，
    // 远超 maxSlopeAngle。用 IsWalkable 判的话每一次上台阶都会被否掉，
    // 台阶功能等于没有。
    //
    // 放宽到"朝上"仍然挡得住真正该挡的东西：垂直的墙面 Dot 为 0、
    // 天花板 Dot 为负，两者都过不了这一关。而落在棱上是**安全**的 ——
    // 那个位置是扫掠出来的，一定不穿透；站不站得稳交给随后的 UpdateGrounded
    // 判断，站不稳就自然滑下来，不需要在这里提前否决。
    //--------------------------------------------------------------------------
    if (Dot(hitDown.normal, up) <= kEpsilon) {
        position = directPosition;
        return direct;
    }

    const Vec3 steppedPosition =
        forwardPosition - up * Max(real(0), hitDown.fraction * liftHeight - config.skinWidth);

    //--------------------------------------------------------------------------
    // 只有当台阶路线**确实走得更远**时才采用。
    //
    // 少了这一条，角色在贴着墙侧向移动时会被无谓地抬上抬下 ——
    // 那种上下抽搐是台阶逻辑最典型的 bug 表现。
    //--------------------------------------------------------------------------
    const Vec3 horizontalDirection = horizontal * (real(1) / horizontalLength);
    const real directProgress =
        Dot(ProjectOnPlane(directPosition - position, up), horizontalDirection);
    const real steppedProgress =
        Dot(ProjectOnPlane(steppedPosition - position, up), horizontalDirection);

    if (steppedProgress <= directProgress + config.skinWidth) {
        position = directPosition;
        return direct;
    }

    CharacterMoveResult result = forward;
    result.displacement = steppedPosition - position;
    result.collided = true;
    result.steppedUp = true;
    result.collidedBelow = true;
    position = steppedPosition;
    return result;
}

//==============================================================================
// 地面检测
//==============================================================================

void CharacterController::UpdateGrounded(const ICharacterWorld& world) {
    isGrounded = false;
    groundNormal = config.up;

    ShapeCastHit hit;
    if (!world.SweepCharacter(GetShape(), Transform(position, Quat::Identity()),
                              -config.up * config.groundProbeDistance, config.layerMask,
                              hit)) {
        return;
    }

    if (hit.startPenetrating) {
        // 已经陷进地面里了。法线朝上就当作站着 —— 这一帧 SlideMove 会把它推出来。
        if (IsWalkable(hit.normal)) {
            isGrounded = true;
            groundNormal = hit.normal;
        }
        return;
    }

    // 探到的面必须够平才算"站得住"。站在 70 度的陡坡上时这里是 false，
    // 于是角色会继续被重力加速、往下滑 —— 这正是想要的行为。
    if (IsWalkable(hit.normal)) {
        isGrounded = true;
        groundNormal = hit.normal;
    }
}

//==============================================================================
// 每帧更新
//==============================================================================

CharacterMoveResult CharacterController::Update(const ICharacterWorld& world,
                                                const Vec3& desiredHorizontalVelocity,
                                                real dt, const Vec3& gravity) {
    const Vec3 up = config.up;

    //--------------------------------------------------------------------------
    // 竖直速度由物理管，水平速度由输入**直接设定**。
    //
    // 这个拆分是角色手感的核心：水平方向不做加速度积分，玩家松开键立刻停、
    // 按下键立刻走。想要"惯性"的话由游戏逻辑自己对 desiredHorizontalVelocity
    // 做平滑 —— 那是玩法调校，不该埋在物理里。
    //--------------------------------------------------------------------------
    real verticalSpeed = Dot(velocity, up);
    verticalSpeed += Dot(gravity, up) * dt;

    const bool wasGrounded = isGrounded;
    if (wasGrounded && verticalSpeed < real(0)) {
        //----------------------------------------------------------------------
        // 站在地上就把下落速度清零，**不要**塞一个"贴地速度"进去。
        //
        // 早期版本是给一个小的向下速度把角色"粘"在地面上。那样做在平地上没问题，
        // 但在**斜坡**上是错的：这个向下的速度会被 collide-and-slide 投影到坡面上，
        // 变成一个沿坡向下的分量 —— 于是角色站在 25 度的缓坡上会自己往下滑，
        // 明明这个坡是可以站人的。实测就是这样：爬上去之后又一路溜回坡底。
        //
        // 正确做法是把"贴地"从速度里拿出来，做成移动之后单独的一步
        // （SnapToGround），那一步只沿竖直方向吸附，不参与滑动投影。
        //----------------------------------------------------------------------
        verticalSpeed = real(0);
    }

    //--------------------------------------------------------------------------
    // 站在斜坡上时，把水平速度投影到坡面上（保持速率不变）。
    //
    // 不投影的话，上坡时角色是"水平撞进斜坡再被滑动逻辑顶上去"，
    // 每帧顶一点，走起来一顿一顿的；下坡则会短暂离地。
    //
    // **关键：投影结果只用于这一帧的位移，绝不能写回 velocity。**
    //
    // 写回去会形成一个正反馈：投影后的速度带着一个向上的分量，下一帧开头
    // `verticalSpeed = Dot(velocity, up)` 把它读出来当成竖直速度，然后
    // `velocity = 水平 + up * verticalSpeed` 又把水平里那份向上分量加了第二遍 ——
    // 每帧翻一倍。实测症状是角色走到台阶棱上之后**像火箭一样窜到 1.9 米高**，
    // 再抛物线落回来。查起来极其费劲，因为代码里没有任何一处"给了向上的速度"。
    //--------------------------------------------------------------------------
    Vec3 slopeAdjusted = desiredHorizontalVelocity;
    if (isGrounded) {
        const real speed = slopeAdjusted.Length();
        if (speed > kEpsilon) {
            const Vec3 onSlope = ProjectOnPlane(slopeAdjusted, groundNormal);
            const real onSlopeLength = onSlope.Length();
            if (onSlopeLength > kEpsilon) {
                slopeAdjusted = onSlope * (speed / onSlopeLength);
            }
        }
    }

    // 存起来的是**没有坡面分量**的速度，所以下一帧读到的 verticalSpeed 干净
    velocity = desiredHorizontalVelocity + up * verticalSpeed;

    CharacterMoveResult result = Move(world, (slopeAdjusted + up * verticalSpeed) * dt);

    //--------------------------------------------------------------------------
    // 撞到东西之后修正速度，否则下一帧还会带着同样的速度往墙里撞
    //--------------------------------------------------------------------------
    if (result.collidedAbove && Dot(velocity, up) > real(0)) {
        velocity = ProjectOnPlane(velocity, up);  // 撞到天花板，上升速度清零
    }

    //--------------------------------------------------------------------------
    // 贴地：原本站在地上、而且没有在往上走时，把角色吸附回地面。
    //
    // 这一步替代了旧的"贴地速度"，两个好处：
    //   - 它只沿竖直方向动，不会被投影成沿坡向下的滑动（缓坡上不再自己往下溜）
    //   - 吸附距离是 stepOffset 而不是一帧的位移，所以**下楼梯**时角色会贴着
    //     台阶走下去，而不是从每级边缘飞出去一小段再落下（画面上下抖动）
    //--------------------------------------------------------------------------
    if (wasGrounded && Dot(velocity, up) <= real(0)) {
        SnapToGround(world);
    }

    UpdateGrounded(world);

    if (isGrounded && Dot(velocity, up) < real(0)) {
        velocity = ProjectOnPlane(velocity, up);  // 落地，下落速度清零
    }

    return result;
}

//==============================================================================
// 贴地吸附
//==============================================================================
//
// 只沿竖直方向把角色吸回地面，**不参与 collide-and-slide 的投影** ——
// 这正是它和"贴地速度"的本质区别，也是缓坡上不再自己往下滑的原因。
//
// 吸附距离取 stepOffset：下楼梯时角色能贴着台阶走下去，而不是从每级边缘
// 飞出去一小段再落下。走下超过 stepOffset 的落差才会真正腾空 —— 那是对的，
// 那本来就该是"掉下去"而不是"走下去"。
//==============================================================================

void CharacterController::SnapToGround(const ICharacterWorld& world) {
    const Vec3 up = config.up;
    const real snapDistance = config.stepOffset + config.skinWidth;

    ShapeCastHit hit;
    if (!world.SweepCharacter(GetShape(), Transform(position, Quat::Identity()),
                              -up * snapDistance, config.layerMask, hit)) {
        return;  // 下面是空的，让它自然下落
    }
    if (hit.startPenetrating) return;      // 已经陷进去了，交给下一帧的脱困逻辑
    if (!IsWalkable(hit.normal)) return;   // 下面是陡坡或者棱，不该被吸上去

    position -= up * Max(real(0), hit.fraction * snapDistance - config.skinWidth);
}

//==============================================================================
// 跳跃
//==============================================================================

bool CharacterController::Jump(real jumpSpeed) {
    if (!isGrounded) return false;

    const Vec3 up = config.up;
    // 先清掉原有的竖直分量再加，免得"贴地速度"把跳跃高度吃掉一截
    velocity = ProjectOnPlane(velocity, up) + up * jumpSpeed;

    // 立刻标记为离地：不然同一帧里的地面检测会把跳跃速度清零，
    // 表现为"偶尔跳不起来"—— 一个极难复现的经典 bug。
    isGrounded = false;
    return true;
}

}  // namespace pe
