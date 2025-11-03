/**
 * @file OS_Thread.h
 * @brief Header file for thread management functions and class in the BasicSdk.
 */

#ifndef __BASICSDK_OS_THREAD_H__
#define __BASICSDK_OS_THREAD_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"

/**
 * @brief Creates a new thread.
 *
 * @param hHandle Pointer to the thread handle that will be set by this function.
 * @param func The function to be executed by the thread.
 * @param lpParameter A pointer to a variable to be passed to the thread.
 * @param mode The real-time mode (default is USE_DEFAULT).
 * @param uintStackSize The stack size for the thread (default is 0, which means system default).
 */
BASICSDK_API void os_thread_create(OS_THREAD_HANDLE *hHandle, OS_THREAD_FUNCTION func, void *lpParameter,
                                   RT_MODE mode = USE_DEFAULT, size_t uintStackSize = 0);

/**
 * @brief Waits for the thread to terminate.
 *
 * @param hHandle The handle of the thread to wait for.
 * @param mode The real-time mode (default is USE_DEFAULT).
 */
BASICSDK_API void os_thread_join(OS_THREAD_HANDLE hHandle, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Sets the CPU affinity for a specific thread.
 *
 * @param hHandle The handle of the thread.
 * @param iCpuIndex The index of the CPU to which the thread should be bound.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool os_thread_sched_set_thread_affinity(OS_THREAD_HANDLE hHandle, int iCpuIndex);

/**
 * @brief Sets the CPU affinity for the current thread.
 *
 * @param iCpuIndex The index of the CPU to which the thread should be bound.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool os_thread_sched_set_affinity(int iCpuIndex);

/**
 * @brief Atomically adds a value to a variable and returns the original value.
 *
 * @param Addend Pointer to the variable to be modified.
 * @return The original value of the variable before the addition.
 */
BASICSDK_API unsigned long os_thread_sync_fetch_add(unsigned long volatile *Addend);

/**
 * @brief Atomically subtracts a value from a variable and returns the original value.
 *
 * @param Addend Pointer to the variable to be modified.
 * @return The original value of the variable before the subtraction.
 */
BASICSDK_API unsigned long os_thread_sync_fetch_sub(unsigned long volatile *Addend);

/**
 * @brief Atomically compares the value of a variable and swaps it with a new value if the comparison succeeds.
 *
 * @param var Pointer to the variable to be compared and swapped.
 * @param compare_var The value to compare against.
 * @param new_var The new value to set if the comparison succeeds.
 * @return True if the swap was successful, false otherwise.
 */
BASICSDK_API bool os_thread_sync_compare_and_swap(long volatile *var, long compare_var, long new_var);

/**
 * @brief Atomically compares the value of a 64-bit variable and swaps it with a new value if the comparison succeeds.
 *
 * @param var Pointer to the 64-bit variable to be compared and swapped.
 * @param compare_var The value to compare against.
 * @param new_var The new value to set if the comparison succeeds.
 * @return True if the swap was successful, false otherwise.
 */
BASICSDK_API bool os_thread_sync_compare_and_swap_64(long long volatile *var, long long compare_var, long long new_var);

/**
 * @brief Initializes real-time scheduling for the current thread.
 *
 * @param iPriority The priority of the thread.
 * @param mode The real-time mode (default is USE_DEFAULT).
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool os_thread_init_rt_schedule(
    int iPriority, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Exits real-time scheduling for the current thread.
 *
 * @param mode The real-time mode (default is USE_DEFAULT).
 */
BASICSDK_API void os_thread_exit_rt_schedule(RT_MODE mode = USE_DEFAULT);

/**
 * @brief Installs latency target for the current thread.
 *
 * @param intLatencyTargetFd File descriptor for the latency target.
 * @return True if successful, false otherwise.
 */
BASICSDK_API bool os_thread_install_latency(int &intLatencyTargetFd);

/**
 * @brief Uninstalls latency target for the current thread.
 *
 * @param intLatencyTargetFd File descriptor for the latency target.
 */
BASICSDK_API void os_thread_uninstall_latency(int intLatencyTargetFd);

/**
 * @brief Suspends the execution of the current thread for a specified number of milliseconds.
 *
 * @param dwMilliseconds The number of milliseconds to sleep.
 * @param mode The real-time mode (default is USE_DEFAULT).
 */
BASICSDK_API void os_thread_sleep_milliseconds(DWORD dwMilliseconds, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Yields the processor to another thread.
 *
 * @param mode The real-time mode (default is USE_DEFAULT).
 */
BASICSDK_API void os_thread_yield(RT_MODE mode = USE_DEFAULT);

/**
 * @brief Synchronizes threads at a barrier.
 */
BASICSDK_API void os_thread_barrier();

/**
 * @brief Returns the ID of the current thread.
 *
 * @return The ID of the current thread.
 */
BASICSDK_API unsigned long os_thread_current();

namespace OS {
/**
 * @class Thread
 * @brief A class for managing threads.
 */
class Thread {
public:
    /**
     * @brief Creates a new thread.
     *
     * @param hHandle Pointer to the thread handle that will be set by this function.
     * @param func The function to be executed by the thread.
     * @param lpParameter A pointer to a variable to be passed to the thread.
     * @param mode The real-time mode (default is USE_DEFAULT).
     * @param uintStackSize The stack size for the thread (default is 0, which means system default).
     */
    static void create(OS_THREAD_HANDLE *hHandle, OS_THREAD_FUNCTION func, void *lpParameter,
                       RT_MODE mode = USE_DEFAULT, size_t uintStackSize = 0) {
        return os_thread_create(hHandle, func, lpParameter, mode, uintStackSize);
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Creates a new thread with a function pointer.
     *
     * @tparam _Func The type of the function.
     * @param hHandle Pointer to the thread handle that will be set by this function.
     * @param func The function to be executed by the thread.
     * @param lpParameter A pointer to a variable to be passed to the thread.
     * @param mode The real-time mode (default is USE_DEFAULT).
     * @param uintStackSize The stack size for the thread (default is 0, which means system default).
     */
    template <typename _Func,
              typename std::enable_if<std::is_same<_Func, LPVOID (*)(void *)>::value, bool>::type = false>
    static void createEx(OS_THREAD_HANDLE *hHandle, _Func func, void *lpParameter, RT_MODE mode = USE_DEFAULT,
                         size_t uintStackSize = 0) {
        struct MemberFunc {
            void *param;
            _Func func;
        };
        MemberFunc *mem_f = new MemberFunc();
        mem_f->param = lpParameter;
        mem_f->func = func;
        auto f = [](void *lpData) {
            MemberFunc *caller = (MemberFunc *)lpData;
            LPVOID ret = (void *)nullptr;
            if (caller != nullptr) {
                ret = (void *)caller->func(caller->param);
                delete caller;
            }
            return ret;
        };
        void *(*_fn)(void *) = f;
        create(hHandle, _fn, mem_f, mode, uintStackSize);
    }

    /**
     * @brief Creates a new thread with a member function pointer.
     *
     * @tparam _Func The type of the member function.
     * @tparam _CallerArg The type of the object containing the member function.
     * @param hHandle Pointer to the thread handle that will be set by this function.
     * @param func The member function to be executed by the thread.
     * @param lpParameter A pointer to the object containing the member function.
     * @param mode The real-time mode (default is USE_DEFAULT).
     * @param uintStackSize The stack size for the thread (default is 0, which means system default).
     */
    template <typename _Func, typename _CallerArg,
              typename std::enable_if<std::is_same<_Func, void (_CallerArg::*)()>::value, bool>::type = false>
    static void createEx(OS_THREAD_HANDLE *hHandle, _Func func, _CallerArg *lpParameter, RT_MODE mode = USE_DEFAULT,
                         size_t uintStackSize = 0) {
        struct MemberFunc {
            _CallerArg *obj;
            _Func func;
        };
        MemberFunc *mem_f = new MemberFunc();
        mem_f->obj = lpParameter;
        mem_f->func = func;
        auto f = [](void *lpData) {
            MemberFunc *caller = (MemberFunc *)lpData;
            if (caller != nullptr) {
                ((caller->obj)->*(caller->func))();
                delete caller;
            }
            return (void *)nullptr;
        };
        void *(*_fn)(void *) = f;
        create(hHandle, _fn, mem_f, mode, uintStackSize);
    }
#endif

    /**
     * @brief Waits for the thread to terminate.
     *
     * @param hHandle The handle of the thread to wait for.
     * @param mode The real-time mode (default is USE_DEFAULT).
     */
    static void join(OS_THREAD_HANDLE hHandle, RT_MODE mode = USE_DEFAULT) { return os_thread_join(hHandle, mode); }

    /**
     * @brief Sets the CPU affinity for a specific thread.
     *
     * @param hHandle The handle of the thread.
     * @param iCpuIndex The index of the CPU to which the thread should be bound.
     * @return True if successful, false otherwise.
     */
    static bool schedSetThreadAffinity(OS_THREAD_HANDLE hHandle, int iCpuIndex) {
        return os_thread_sched_set_thread_affinity(hHandle, iCpuIndex);
    }

    /**
     * @brief Sets the CPU affinity for the current thread.
     *
     * @param iCpuIndex The index of the CPU to which the thread should be bound.
     * @return True if successful, false otherwise.
     */
    static bool schedSetAffinity(int iCpuIndex) { return os_thread_sched_set_affinity(iCpuIndex); }

    /**
     * @brief Atomically adds a value to a variable and returns the original value.
     *
     * @param Addend Pointer to the variable to be modified.
     * @return The original value of the variable before the addition.
     */
    static unsigned long syncFetchAdd(unsigned long volatile *Addend) { return os_thread_sync_fetch_add(Addend); }

    /**
     * @brief Atomically subtracts a value from a variable and returns the original value.
     *
     * @param Addend Pointer to the variable to be modified.
     * @return The original value of the variable before the subtraction.
     */
    static unsigned long syncFetchSub(unsigned long volatile *Addend) { return os_thread_sync_fetch_sub(Addend); }

    /**
     * @brief Atomically compares the value of a variable and swaps it with a new value if the comparison succeeds.
     *
     * @param var Pointer to the variable to be compared and swapped.
     * @param compare_var The value to compare against.
     * @param new_var The new value to set if the comparison succeeds.
     * @return True if the swap was successful, false otherwise.
     */
    static bool syncCompareAndSwap(long volatile *var, long compare_var, long new_var) {
        return os_thread_sync_compare_and_swap(var, compare_var, new_var);
    }

    /**
     * @brief Atomically compares the value of a 64-bit variable and swaps it with a new value if the comparison
     * succeeds.
     *
     * @param var Pointer to the 64-bit variable to be compared and swapped.
     * @param compare_var The value to compare against.
     * @param new_var The new value to set if the comparison succeeds.
     * @return True if the swap was successful, false otherwise.
     */
    static bool syncCompareAndSwap64(long long volatile *var, long long compare_var, long long new_var) {
        return os_thread_sync_compare_and_swap_64(var, compare_var, new_var);
    }

    /**
     * @brief Initializes real-time scheduling for the current thread.
     *
     * @param iPriority The priority of the thread (ignored if mode is real-time).
     * @param mode The real-time mode (default is USE_DEFAULT).
     * @return True if successful, false otherwise.
     */
    static bool initRTSchedule(int iPriority, RT_MODE mode = USE_DEFAULT) {
        // 当该函数工作在实时模式时， iPriority会被忽略
        return os_thread_init_rt_schedule(iPriority, mode);
    }

    /**
     * @brief Exits real-time scheduling for the current thread.
     *
     * @param mode The real-time mode (default is USE_DEFAULT).
     */
    static void exitRTSchedule(RT_MODE mode = USE_DEFAULT) { return os_thread_exit_rt_schedule(mode); }

    /**
     * @brief Installs latency target for the current thread.
     *
     * @param intLatencyTargetFd File descriptor for the latency target.
     * @return True if successful, false otherwise.
     */
    static bool installLatency(int &intLatencyTargetFd) { return os_thread_install_latency(intLatencyTargetFd); }

    /**
     * @brief Uninstalls latency target for the current thread.
     *
     * @param intLatencyTargetFd File descriptor for the latency target.
     */
    static void uninstallLatency(int intLatencyTargetFd) { return os_thread_uninstall_latency(intLatencyTargetFd); }

    /**
     * @brief Suspends the execution of the current thread for a specified number of milliseconds.
     *
     * @param dwMilliseconds The number of milliseconds to sleep.
     * @param mode The real-time mode (default is USE_DEFAULT).
     */
    static void sleepMilliseconds(DWORD dwMilliseconds, RT_MODE mode = USE_DEFAULT) {
        return os_thread_sleep_milliseconds(dwMilliseconds, mode);
    }

    /**
     * @brief Yields the processor to another thread.
     *
     * @param mode The real-time mode (default is USE_DEFAULT).
     */
    static void yield(RT_MODE mode = USE_DEFAULT) { os_thread_yield(mode); }

    /**
     * @brief Synchronizes threads at a barrier.
     */
    static void barrier() { os_thread_barrier(); }

    /**
     * @brief Returns the ID of the current thread.
     *
     * @return The ID of the current thread.
     */
    static unsigned long current() { return os_thread_current(); }
};
}  // namespace OS

#endif