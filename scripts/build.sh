#!/usr/bin/env bash
#==============================================================================
# scripts/build.sh —— 一键配置 + 构建 + 跑测试
#
# 这台机器上没有系统级的 C++ 工具链，所以用的是解压到 C:\tools 的便携版：
#   C:\tools\mingw64   MinGW-w64 GCC 16.2.0 (UCRT, posix threads, SEH)
#   C:\tools\cmake     CMake 4.4.2
# 两者都没有写进系统 PATH，只在这个脚本里临时加上，不污染环境。
#
# 用法：
#   bash scripts/build.sh            # 增量构建 + 跑测试
#   bash scripts/build.sh clean      # 删掉 build/ 重新来
#   bash scripts/build.sh notest     # 只构建，不跑测试
#==============================================================================
set -e

TOOLCHAIN_BIN="/c/tools/mingw64/bin"
CMAKE_BIN="/c/tools/cmake/bin"
export PATH="$TOOLCHAIN_BIN:$CMAKE_BIN:$PATH"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$PROJECT_DIR/build"

if [ "$1" == "clean" ]; then
    echo ">>> 清理 $BUILD_DIR"
    rm -rf "$BUILD_DIR"
    shift
fi

echo ">>> 配置"
cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" -G "MinGW Makefiles" \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo ">>> 构建"
cmake --build "$BUILD_DIR" -j "$(nproc)"

if [ "$1" != "notest" ]; then
    echo ">>> 测试"
    "$BUILD_DIR/bin/pe_tests.exe"
fi
