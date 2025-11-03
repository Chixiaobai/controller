#pragma once

// 删除字符串两端的空格和制表符

namespace amw {

#if defined(_WIN32) || defined(_WINDOWS)
typedef unsigned long id_type;  // Process id type
typedef void* fd_type;          // File descriptor type

#include <Windows.h>

class Handle {
public:
    Handle() noexcept : handle(INVALID_HANDLE_VALUE) {}
    ~Handle() noexcept { close(); }
    void close() noexcept {
        if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle);
    }
    HANDLE detach() noexcept {
        HANDLE old_handle = handle;
        handle = INVALID_HANDLE_VALUE;
        return old_handle;
    }
    operator HANDLE() const noexcept { return handle; }
    HANDLE* operator&() noexcept { return &handle; }

    Handle& operator=(HANDLE new_handle) noexcept {
        if (handle != INVALID_HANDLE_VALUE) CloseHandle(handle);
        handle = new_handle;
        return *this;
    }

private:
    HANDLE handle;
};

#else
#include "sys/wait.h"
typedef pid_t id_type;
typedef int fd_type;
#endif

}  // namespace amw