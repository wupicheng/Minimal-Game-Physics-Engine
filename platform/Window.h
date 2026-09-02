#pragma once
//==============================================================================
// platform/Window.h
//
// 一个真正的窗口，用 **Win32 + GDI**。
//
//------------------------------------------------------------------------------
// 为什么是 GDI 而不是 SDL / GLFW / OpenGL
//------------------------------------------------------------------------------
// 因为它们**一个都不需要装**。`windows.h` 和 GDI 是系统自带的，
// 只要链上 `gdi32` 和 `user32` 就完事 —— 没有第三方依赖、没有 vcpkg、
// 没有下载。对一个"证明引擎能驱动真实画面"的目标来说，这是最短的路径。
//
// GDI 的性能对我们完全够用：每帧要做的只是把一块 32 位像素缓冲 `StretchDIBits`
// 到窗口上。真正的开销在光线投射（27 ms），blit 那点时间可以忽略。
//
// 想换 OpenGL/DirectX 也很简单 —— 这个类只暴露三件事：
// 开窗口、把一块像素缓冲贴上去、读键盘。
//==============================================================================

#include <cstdint>
#include <string>

namespace platform {

class Window {
public:
    /// clientWidth/Height 是**窗口**的像素尺寸；帧缓冲会被拉伸到这个大小。
    /// 内部渲染分辨率低得多（见 PixelRenderer.h），整数倍放大得到复古颗粒感。
    Window(const std::string& title, int clientWidth, int clientHeight);
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    bool IsOpen() const noexcept { return m_open; }

    /// 处理消息队列。每帧调一次，不调窗口会假死。
    void PumpMessages();

    /// 把帧缓冲贴到窗口上（自动拉伸到客户区大小）。
    void Present(const std::uint32_t* pixels, int width, int height);

    /// 某个虚拟键现在按着吗（VK_* 常量）。
    ///
    /// 用 GetAsyncKeyState 读**按住**状态而不是消息队列里的按键事件：
    /// 移动必须是"按住就一直走"，事件式输入会被键盘重复率卡成一顿一顿的。
    bool IsKeyDown(int virtualKey) const;

    /// 鼠标相对上一帧移动了多少（像素）。第一人称视角靠它转身。
    void GetMouseDelta(int& dx, int& dy);

    /// 把鼠标锁在窗口中心并隐藏光标 —— FPS 的标准做法，
    /// 不然鼠标会跑出窗口、点到别的程序上去。
    void SetMouseCaptured(bool captured);
    bool MouseCaptured() const noexcept { return m_mouseCaptured; }

    int ClientWidth() const noexcept { return m_width; }
    int ClientHeight() const noexcept { return m_height; }

    /// 系统上有没有窗口环境（无头/服务会话下没有）。
    static bool IsAvailable();

private:
    void* m_hwnd = nullptr;  ///< HWND，避免在头文件里拖进 windows.h
    int m_width = 0;
    int m_height = 0;
    bool m_open = false;
    bool m_mouseCaptured = false;
    int m_lastMouseX = 0;
    int m_lastMouseY = 0;
};

}  // namespace platform
