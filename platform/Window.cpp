//==============================================================================
// platform/Window.cpp
//==============================================================================

#include "platform/Window.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace platform {

namespace {

constexpr const char* kClassName = "PhysEngineFpsWindow";
bool g_classRegistered = false;

LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) {
    switch (msg) {
        case WM_CLOSE:
            // 只是标记要关，真正的销毁交给析构 —— 在消息回调里删自己很容易出事
            PostQuitMessage(0);
            return 0;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        case WM_ERASEBKGND:
            // 我们每帧都会铺满整个客户区，让系统再擦一遍只会造成闪烁
            return 1;
        default:
            break;
    }
    return DefWindowProcA(hwnd, msg, wparam, lparam);
}

}  // namespace

//==============================================================================

bool Window::IsAvailable() {
    // 无头会话（服务、SSH、CI）里拿不到桌面窗口站，开窗口会失败。
    // 提前问一句，好让调用方干净地退回到控制台模式。
    return GetDesktopWindow() != nullptr;
}

Window::Window(const std::string& title, int clientWidth, int clientHeight)
    : m_width(clientWidth), m_height(clientHeight) {
    const HINSTANCE instance = GetModuleHandleA(nullptr);

    if (!g_classRegistered) {
        WNDCLASSA wc{};
        wc.lpfnWndProc = WindowProc;
        wc.hInstance = instance;
        wc.lpszClassName = kClassName;
        wc.hCursor = LoadCursorA(nullptr, IDC_ARROW);
        wc.style = CS_HREDRAW | CS_VREDRAW;
        if (RegisterClassA(&wc) == 0) return;
        g_classRegistered = true;
    }

    // AdjustWindowRect：传进来的是**客户区**尺寸，要加上标题栏和边框才是窗口尺寸。
    // 不做这一步的话画面会被边框吃掉几十个像素。
    RECT rect{0, 0, clientWidth, clientHeight};
    const DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&rect, style, FALSE);

    const HWND hwnd = CreateWindowExA(
        0, kClassName, title.c_str(), style, CW_USEDEFAULT, CW_USEDEFAULT,
        rect.right - rect.left, rect.bottom - rect.top, nullptr, nullptr, instance,
        nullptr);
    if (hwnd == nullptr) return;

    m_hwnd = hwnd;
    m_open = true;
    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);
    SetForegroundWindow(hwnd);
}

Window::~Window() {
    if (m_hwnd != nullptr) {
        SetMouseCaptured(false);
        DestroyWindow(static_cast<HWND>(m_hwnd));
    }
}

void Window::PumpMessages() {
    MSG msg;
    while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
        if (msg.message == WM_QUIT) {
            m_open = false;
            return;
        }
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }
}

void Window::Present(const std::uint32_t* pixels, int width, int height) {
    if (m_hwnd == nullptr || pixels == nullptr) return;

    const HWND hwnd = static_cast<HWND>(m_hwnd);
    const HDC dc = GetDC(hwnd);
    if (dc == nullptr) return;

    //--------------------------------------------------------------------------
    // 32 位 BI_RGB 的 DIB。**负的 biHeight 表示自上而下**的行序 ——
    // 不写负号的话图像会上下颠倒，这是 Win32 位图最经典的坑。
    //--------------------------------------------------------------------------
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = width;
    info.bmiHeader.biHeight = -height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;

    // 低分辨率放大到窗口。用 COLORONCOLOR（最近邻）而不是插值：
    // 我们要的就是清晰的方块颗粒感，模糊反而糊成一团。
    SetStretchBltMode(dc, COLORONCOLOR);
    StretchDIBits(dc, 0, 0, m_width, m_height, 0, 0, width, height, pixels, &info,
                  DIB_RGB_COLORS, SRCCOPY);

    ReleaseDC(hwnd, dc);
}

bool Window::IsKeyDown(int virtualKey) const {
    // 只在窗口是前台时才响应，免得玩家切出去之后角色还在走
    if (m_hwnd != nullptr && GetForegroundWindow() != static_cast<HWND>(m_hwnd)) {
        return false;
    }
    return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
}

void Window::GetMouseDelta(int& dx, int& dy) {
    dx = 0;
    dy = 0;
    if (m_hwnd == nullptr || !m_mouseCaptured) return;

    const HWND hwnd = static_cast<HWND>(m_hwnd);
    if (GetForegroundWindow() != hwnd) return;

    POINT cursor;
    if (!GetCursorPos(&cursor)) return;

    dx = cursor.x - m_lastMouseX;
    dy = cursor.y - m_lastMouseY;

    //--------------------------------------------------------------------------
    // 每帧把光标拉回客户区中心。
    //
    // 这是第一人称视角的标准做法：不这么做的话，鼠标很快会撞到屏幕边缘，
    // 之后再怎么移动都读不到位移，视角就转不动了。
    //--------------------------------------------------------------------------
    RECT client;
    GetClientRect(hwnd, &client);
    POINT center{(client.right - client.left) / 2, (client.bottom - client.top) / 2};
    ClientToScreen(hwnd, &center);
    SetCursorPos(center.x, center.y);
    m_lastMouseX = center.x;
    m_lastMouseY = center.y;
}

void Window::SetMouseCaptured(bool captured) {
    if (m_hwnd == nullptr || m_mouseCaptured == captured) return;
    m_mouseCaptured = captured;

    const HWND hwnd = static_cast<HWND>(m_hwnd);
    if (captured) {
        RECT client;
        GetClientRect(hwnd, &client);
        POINT center{(client.right - client.left) / 2, (client.bottom - client.top) / 2};
        ClientToScreen(hwnd, &center);
        SetCursorPos(center.x, center.y);
        m_lastMouseX = center.x;
        m_lastMouseY = center.y;
        ShowCursor(FALSE);
    } else {
        ShowCursor(TRUE);
    }
}

}  // namespace platform
