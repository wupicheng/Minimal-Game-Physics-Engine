//==============================================================================
// tests/test_core.cpp
//
// core 层测试：Handle 的代际语义、SlotArray 的槽位复用与悬垂检测。
//
// 这一层的正确性是整个引擎内存安全的地基 —— 如果"已删除的刚体句柄还能取到
// 对象"，后面所有模块都会出诡异的 bug。所以这里的用例写得比较偏执。
//==============================================================================

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <unordered_map>
#include <vector>

#include "pe/core/Handle.h"
#include "pe/core/SlotArray.h"
#include "pe/core/Version.h"

using namespace pe;

namespace {
struct TestTag;
using TestHandle = Handle<TestTag>;

/// 一个最小的 POD，代表将来的 RigidBody。
struct Dummy {
    int id = 0;
    real value = real(0);

    Dummy() = default;
    Dummy(int i, real v) : id(i), value(v) {}
};

using DummyPool = SlotArray<Dummy, TestTag>;
}  // namespace

TEST_CASE("Version 信息可用", "[core]") {
    REQUIRE(std::string(VersionString()) == "0.1.0");
    REQUIRE(VersionNumber() == 100u);  // 0*10000 + 1*100 + 0
}

TEST_CASE("Handle 空句柄语义", "[core][handle]") {
    const TestHandle null = TestHandle::Null();
    REQUIRE_FALSE(null.IsValid());

    const TestHandle h(0u, 1u);
    REQUIRE(h.IsValid());
    REQUIRE(h != null);

    // 同 index 不同 generation 必须判为不同句柄 —— 这是防悬垂的核心。
    const TestHandle sameSlotNewGen(0u, 2u);
    REQUIRE(h != sameSlotNewGen);
}

TEST_CASE("Handle 可作为 unordered 容器的键", "[core][handle]") {
    std::unordered_map<TestHandle, int> map;
    map[TestHandle(3u, 1u)] = 10;
    map[TestHandle(3u, 2u)] = 20;  // 同槽位不同代际，必须是两个不同的键

    REQUIRE(map.size() == 2);
    REQUIRE(map.at(TestHandle(3u, 1u)) == 10);
    REQUIRE(map.at(TestHandle(3u, 2u)) == 20);
}

TEST_CASE("SlotArray 基本增删查", "[core][slotarray]") {
    DummyPool pool;
    REQUIRE(pool.Empty());
    REQUIRE(pool.Size() == 0);

    const TestHandle a = pool.Emplace(1, real(1.5));
    const TestHandle b = pool.Emplace(2, real(2.5));

    REQUIRE(pool.Size() == 2);
    REQUIRE(pool.IsValid(a));
    REQUIRE(pool.IsValid(b));

    REQUIRE(pool.Get(a) != nullptr);
    REQUIRE(pool.Get(a)->id == 1);
    REQUIRE(pool.Get(b)->id == 2);

    // 通过指针修改，再取一次应能看到改动
    pool.Get(a)->value = real(99);
    REQUIRE(pool.Get(a)->value == real(99));

    REQUIRE(pool.Remove(a));
    REQUIRE(pool.Size() == 1);
    REQUIRE_FALSE(pool.IsValid(a));
    REQUIRE(pool.Get(a) == nullptr);  // 关键：失效句柄取到的是 nullptr，不是野指针
    REQUIRE(pool.IsValid(b));         // 删 a 不能影响 b
}

TEST_CASE("SlotArray 重复删除是安全的", "[core][slotarray]") {
    DummyPool pool;
    const TestHandle h = pool.Emplace(1, real(0));

    REQUIRE(pool.Remove(h));
    REQUIRE_FALSE(pool.Remove(h));  // 第二次删除返回 false，不崩溃
    REQUIRE(pool.Size() == 0);
}

TEST_CASE("SlotArray 槽位复用后旧句柄失效", "[core][slotarray]") {
    // 这是整个 core 层最重要的一个用例：
    // 如果这里挂了，说明"删除刚体后旧句柄会指向新刚体"，
    // 会造成极难定位的逻辑错误（子弹打到了一个已经死掉的敌人的碰撞体上）。
    DummyPool pool;

    const TestHandle oldHandle = pool.Emplace(111, real(1));
    REQUIRE(pool.Remove(oldHandle));

    const TestHandle newHandle = pool.Emplace(222, real(2));

    // 槽位确实被复用了（否则这个用例就没意义了）
    REQUIRE(newHandle.index == oldHandle.index);
    // 但代际号变了
    REQUIRE(newHandle.generation != oldHandle.generation);

    // 旧句柄不能访问到新对象
    REQUIRE_FALSE(pool.IsValid(oldHandle));
    REQUIRE(pool.Get(oldHandle) == nullptr);

    // 新句柄正常
    REQUIRE(pool.IsValid(newHandle));
    REQUIRE(pool.Get(newHandle)->id == 222);
}

TEST_CASE("SlotArray 空句柄与越界句柄不会崩", "[core][slotarray]") {
    DummyPool pool;
    pool.Emplace(1, real(0));

    REQUIRE_FALSE(pool.IsValid(TestHandle::Null()));
    REQUIRE(pool.Get(TestHandle::Null()) == nullptr);

    // 索引远超槽位数量
    REQUIRE_FALSE(pool.IsValid(TestHandle(9999u, 1u)));
    REQUIRE(pool.Get(TestHandle(9999u, 1u)) == nullptr);
}

TEST_CASE("SlotArray ForEach 跳过空洞", "[core][slotarray]") {
    DummyPool pool;
    const TestHandle h0 = pool.Emplace(0, real(0));
    const TestHandle h1 = pool.Emplace(1, real(0));
    const TestHandle h2 = pool.Emplace(2, real(0));
    (void)h0;
    (void)h2;

    pool.Remove(h1);  // 中间挖个洞

    std::vector<int> visited;
    pool.ForEach([&](TestHandle h, Dummy& d) {
        REQUIRE(pool.IsValid(h));  // 回调拿到的句柄必须是有效的
        visited.push_back(d.id);
    });

    REQUIRE(visited.size() == 2);
    REQUIRE(visited[0] == 0);
    REQUIRE(visited[1] == 2);
}

TEST_CASE("SlotArray 大量增删后仍然自洽", "[core][slotarray]") {
    // 模拟一局游戏里反复生成/销毁弹壳、道具的情形，
    // 确认代际机制在高频复用下不会串号。
    DummyPool pool;
    std::vector<TestHandle> live;

    for (int round = 0; round < 100; ++round) {
        for (int i = 0; i < 10; ++i) {
            live.push_back(pool.Emplace(round * 10 + i, real(round)));
        }
        // 每轮删掉一半
        for (std::size_t i = 0; i < live.size(); i += 2) {
            pool.Remove(live[i]);
        }
        std::vector<TestHandle> remaining;
        for (std::size_t i = 1; i < live.size(); i += 2) {
            remaining.push_back(live[i]);
        }
        live = remaining;
    }

    // 所有还持有的句柄都必须仍然有效，且内容没被别人覆盖
    for (const TestHandle h : live) {
        REQUIRE(pool.IsValid(h));
        REQUIRE(pool.Get(h) != nullptr);
    }
    REQUIRE(pool.Size() == live.size());

    // 空洞率检查：槽位总数不应该无限膨胀（说明复用确实在工作）
    REQUIRE(pool.SlotCount() <= live.size() + 20);
}

TEST_CASE("SlotArray AtUnchecked 与 HandleAt 一致", "[core][slotarray]") {
    DummyPool pool;
    const TestHandle h = pool.Emplace(42, real(7));

    REQUIRE(pool.IsAliveAt(h.index));
    REQUIRE(pool.AtUnchecked(h.index).id == 42);
    REQUIRE(pool.HandleAt(h.index) == h);

    pool.Remove(h);
    REQUIRE_FALSE(pool.IsAliveAt(h.index));
}
