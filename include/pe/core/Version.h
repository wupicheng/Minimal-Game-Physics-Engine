#pragma once
//==============================================================================
// pe/core/Version.h
//
// 引擎版本号。除了给用户看，它还有一个实际用途：将来做状态序列化
// （存档 / 网络同步 / 回放）时，需要版本号来判断数据格式是否兼容。
//==============================================================================

#include <cstdint>

namespace pe {

inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;

/// 形如 "0.1.0" 的版本串。
const char* VersionString() noexcept;

/// 打包成单个整数，方便做大小比较：major*10000 + minor*100 + patch。
inline constexpr std::uint32_t VersionNumber() noexcept {
    return static_cast<std::uint32_t>(kVersionMajor * 10000 + kVersionMinor * 100 +
                                      kVersionPatch);
}

}  // namespace pe
