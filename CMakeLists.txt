#==============================================================================
# PhysEngine —— 从零手写的 3D FPS 物理引擎
#
# 目标：
#   physengine  静态库，引擎本体（不依赖任何第三方物理库）
#   pe_tests    Catch2 单元测试
#   pe_demo     整合演示（M9 加入）
#
# 常用命令（Windows + MinGW）：
#   cmake -S . -B build -G "MinGW Makefiles"
#   cmake --build build -j
#   ctest --test-dir build --output-on-failure
#
# 也可以直接跑 scripts/build.sh（已经处理好工具链路径）。
#==============================================================================

cmake_minimum_required(VERSION 3.20)

project(PhysEngine
    VERSION 0.1.0
    DESCRIPTION "Hand-written 3D physics engine for FPS games"
    LANGUAGES CXX)

#------------------------------------------------------------------------------
# 选项
#------------------------------------------------------------------------------
option(PE_BUILD_TESTS "Build unit tests" ON)
option(PE_BUILD_DEMO  "Build the integration demo" ON)
option(PE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

#------------------------------------------------------------------------------
# 全局设置
#------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   # 用标准 C++20，不用 GNU 扩展，保证可移植

# 单配置生成器（Makefiles/Ninja）下默认给一个带调试信息的优化构建。
# 物理引擎在 -O0 下慢得没法做手感调试，所以默认不用纯 Debug。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# 可执行文件统一输出到 build/bin，方便找。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#------------------------------------------------------------------------------
# MinGW：静态链接运行时
#
# 不加这几个开关的话，生成的 exe 会依赖三个 MinGW 的运行时 DLL：
#     libgcc_s_seh-1.dll   libstdc++-6.dll   libwinpthread-1.dll
# 它们躺在 C:\tools\mingw64\bin，而那个目录**没有写进系统 PATH**（见
# ARCHITECTURE.md 里工具链那一条）。于是在开发机的构建脚本里跑没问题，
# 双击 exe 或者拷到别的机器上就报
#     "由于找不到 libgcc_s_seh-1.dll，无法继续执行代码"
#
# 静态进去之后 exe 只依赖 KERNEL32/USER32 和 UCRT（api-ms-win-crt-*），
# 后者从 Windows 10 起是系统自带的。代价是每个 exe 大一两兆，无所谓。
#
# `-static` 是必须的 —— 只写 -static-libgcc -static-libstdc++ 的话
# libwinpthread-1.dll 还是会留下。
#------------------------------------------------------------------------------
if(MINGW)
    add_link_options(-static-libgcc -static-libstdc++ -static)
endif()

#------------------------------------------------------------------------------
# 引擎本体
#------------------------------------------------------------------------------
add_library(physengine STATIC
    src/core/Version.cpp
    src/collision/Shape.cpp
    src/collision/RayCast.cpp
    src/collision/UniformGrid.cpp
    src/collision/NarrowPhase.cpp
    src/collision/GJK.cpp
    src/collision/EPA.cpp
    src/collision/ShapeCast.cpp
    src/character/CharacterController.cpp
    src/scene/TriggerSystem.cpp
    src/scene/PhysicsWorld.cpp
    src/dynamics/MassProperties.cpp
    src/dynamics/Integrator.cpp
    src/solver/ContactConstraint.cpp
    src/solver/SequentialImpulseSolver.cpp
)

add_library(PhysEngine::physengine ALIAS physengine)

target_include_directories(physengine
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_compile_features(physengine PUBLIC cxx_std_20)

# 警告开满。物理引擎里最容易出的浮点/类型错误，编译器大多能提前抓到。
if(MSVC)
    target_compile_options(physengine PRIVATE /W4 /permissive-)
    if(PE_WARNINGS_AS_ERRORS)
        target_compile_options(physengine PRIVATE /WX)
    endif()
else()
    target_compile_options(physengine PRIVATE
        -Wall -Wextra -Wpedantic
        -Wshadow            # 局部变量遮蔽成员，物理代码里 n/r/v 这种短名极易撞
        -Wconversion        # 隐式窄化，float/double 混用的主要来源
        -Wsign-conversion)
    if(PE_WARNINGS_AS_ERRORS)
        target_compile_options(physengine PRIVATE -Werror)
    endif()
endif()

#------------------------------------------------------------------------------
# 单元测试
#
# Catch2 v3 通过 FetchContent 拉取。首次配置需要联网（会 git clone），
# 之后走 build 目录里的缓存，离线也能构建。
#------------------------------------------------------------------------------
if(PE_BUILD_TESTS)
    include(FetchContent)

    FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.7.1
        GIT_SHALLOW    TRUE)
    FetchContent_MakeAvailable(Catch2)

    add_executable(pe_tests
        tests/test_core.cpp
        tests/test_math.cpp
        tests/test_collision_shapes.cpp
        tests/test_raycast.cpp
        tests/test_broadphase.cpp
        tests/test_narrowphase.cpp
        tests/test_gjk.cpp
        tests/test_dynamics.cpp
        tests/test_solver.cpp
        tests/test_character.cpp
        tests/test_world.cpp
    )

    target_link_libraries(pe_tests PRIVATE physengine Catch2::Catch2WithMain)

    if(NOT MSVC)
        target_compile_options(pe_tests PRIVATE -Wall -Wextra)
    endif()

    enable_testing()

    # 注意：这里**不用** Catch2 的 catch_discover_tests()。
    # 它会把每个 TEST_CASE 的名字注册成独立的 ctest 用例，而我们的用例名是中文，
    # 在 Windows 控制台的 GBK 代码页下名字会被转码搞乱，导致 ctest 按名字回调
    # 可执行文件时匹配不上，报出假失败（可执行文件直接跑是全过的）。
    #
    # 改成按 Catch2 的**标签**注册：标签都是 ASCII，不受代码页影响，同时保留了
    # 按模块分别运行的能力。失败详情由 Catch2 自己打印，粒度足够。
    add_test(NAME pe_tests_all COMMAND pe_tests)
    add_test(NAME pe_tests_math COMMAND pe_tests "[math]")
    add_test(NAME pe_tests_core COMMAND pe_tests "[core]")
    add_test(NAME pe_tests_collision COMMAND pe_tests "[collision]")
    add_test(NAME pe_tests_raycast COMMAND pe_tests "[raycast]")
    add_test(NAME pe_tests_broadphase COMMAND pe_tests "[broadphase]")
    add_test(NAME pe_tests_narrowphase COMMAND pe_tests "[narrowphase]")
    add_test(NAME pe_tests_gjk COMMAND pe_tests "[gjk]")
    add_test(NAME pe_tests_dynamics COMMAND pe_tests "[dynamics]")
    add_test(NAME pe_tests_solver COMMAND pe_tests "[solver]")
    add_test(NAME pe_tests_shapecast COMMAND pe_tests "[shapecast]")
    add_test(NAME pe_tests_character COMMAND pe_tests "[character]")
    add_test(NAME pe_tests_trigger COMMAND pe_tests "[trigger]")
    add_test(NAME pe_tests_world COMMAND pe_tests "[world]")
endif()

#------------------------------------------------------------------------------
# 演示程序（M9 加入，目前目录还不存在）
#------------------------------------------------------------------------------
if(PE_BUILD_DEMO AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/demo/main.cpp)
    add_executable(pe_demo demo/main.cpp)
    target_link_libraries(pe_demo PRIVATE physengine)
endif()

#------------------------------------------------------------------------------
# 构建摘要
#------------------------------------------------------------------------------
message(STATUS "")
message(STATUS "PhysEngine ${PROJECT_VERSION}")
message(STATUS "  Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  Compiler   : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  Tests      : ${PE_BUILD_TESTS}")
message(STATUS "  Demo       : ${PE_BUILD_DEMO}")
message(STATUS "")

#------------------------------------------------------------------------------
# 示例游戏（跑在 physengine 上的控制台 FPS）
#
# 它是 engine 的**使用者**，不是 engine 的一部分：引擎库不知道它的存在，
# 也不含任何渲染/输入代码。想换成真正的 GPU 渲染，替换 game/Renderer.* 即可。
#------------------------------------------------------------------------------
option(PE_BUILD_GAME "Build the example FPS game" ON)

if(PE_BUILD_GAME AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/game/main.cpp)
    add_executable(pe_game
        game/main.cpp
        game/Game.cpp
        game/Renderer.cpp
        game/PixelRenderer.cpp
        platform/Window.cpp
        platform/Canvas.cpp
        game/Input.cpp
    )
    # gdi32 / user32 是系统自带的，不需要装任何东西 —— 这正是选 GDI
    # 而不是 SDL/GLFW 的理由（见 platform/Window.h）
    target_link_libraries(pe_game PRIVATE physengine)
    if(WIN32)
        target_link_libraries(pe_game PRIVATE gdi32 user32)
    endif()
    target_include_directories(pe_game PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    if(NOT MSVC)
        target_compile_options(pe_game PRIVATE -Wall -Wextra)
    endif()
endif()

    # 游戏的冒烟测试：让机器人自己打一局。
    #
    # 它验证的是引擎测试覆盖不到的那一层 —— 各个模块**接在一起**还能不能用：
    # 角色控制器 + 射线查询 + 触发器 + 事件回调里销毁刚体 + 逐像素渲染，
    # 一整套跑 200 帧不崩、不发散。
    if(PE_BUILD_GAME AND TARGET pe_game)
        add_test(NAME pe_game_smoke
                 COMMAND pe_game --script --no-color --frames 200)
    endif()

#------------------------------------------------------------------------------
# pe_racing —— 示例赛车游戏
#
# 和 pe_game 一样：它是 engine 的**使用者**，不是 engine 的一部分。
# 引擎里没有任何"车"的概念 —— 整台车是一个刚体加四条向下的射线，
# 悬挂和轮胎力都是游戏这一层用 Raycast / ApplyForceAtPoint /
# ApplyImpulseAtPoint 搭出来的（见 racing/Vehicle.h）。
#
# 两个游戏共用 platform/（窗口和画布），互相之间没有任何依赖。
#------------------------------------------------------------------------------
option(PE_BUILD_RACING "Build the example racing game" ON)

if(PE_BUILD_RACING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/racing/main.cpp)
    add_executable(pe_racing
        racing/main.cpp
        racing/RaceGame.cpp
        racing/Vehicle.cpp
        racing/RaceRenderer.cpp
        racing/Input.cpp
        platform/Window.cpp
        platform/Canvas.cpp
    )
    target_link_libraries(pe_racing PRIVATE physengine)
    if(WIN32)
        target_link_libraries(pe_racing PRIVATE gdi32 user32)
    endif()
    target_include_directories(pe_racing PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    if(NOT MSVC)
        target_compile_options(pe_racing PRIVATE -Wall -Wextra)
    endif()

    # 赛车的冒烟测试：让机器人**跑完整场比赛**。
    #
    # 它验证的东西和 pe_game_smoke 完全不同，因为这个游戏用引擎的方式不同：
    # 按物理子步施力、弹簧阻尼悬挂、摩擦圆轮胎、检查点触发器、自动复位。
    # 跑不完三圈就是失败（进程返回 1），所以"车卡在墙里"这类问题会被测出来，
    # 而不是悄悄地跑过去。
    if(TARGET pe_racing)
        add_test(NAME pe_racing_smoke COMMAND pe_racing --bot --frames 9000)
    endif()
endif()
