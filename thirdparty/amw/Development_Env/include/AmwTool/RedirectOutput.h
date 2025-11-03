#pragma once

#include "functional"
#include "AmwToolDef.h"
#include "OS/OS_Sync.h"
#include "Common.h"

namespace amw {
class AMWTOOL_CLASS OutputRedirector {
public:
    OutputRedirector(std::function<void(const std::string&)> logFunc) : m_logFunc(logFunc) {
        m_outputThread = OS_THREAD_HANDLE(NULL);
        open();
    }

    void open();

    ~OutputRedirector() {}

    // 连接UI后启动，连接断开后停止，数据又需要程序开始即开始收集，外部又要控制是否重定向功能，如果不重定向，网络模块也不用启动了
    void run() {
        m_running = true;
        OS::Thread::create(&m_outputThread, sendOutToNetwork, this);
    }

    void stop() {
        m_running = false;
        if (m_outputThread != OS_THREAD_HANDLE(NULL)) {
            OS::Thread::join(m_outputThread);
            m_outputThread = OS_THREAD_HANDLE(NULL);
        }
    }

    bool redirectStdoutToPipe(fd_type stdoutWdPipe);

private:
    fd_type m_outputPipe;
    bool m_redirecting;
    bool m_running;
    std::function<void(const std::string&)> m_logFunc;
    OS_THREAD_HANDLE m_outputThread;

    static void* sendOutToNetwork(void* arg);
};

}  // namespace amw
