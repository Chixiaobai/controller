
#pragma once

#include "atomic"
#include "functional"
#include "OS/OS_Sync.h"
#include "Socket/Socket.h"
#include "../AmwToolDef.h"

/*
    thread+非阻塞IO
*/
class AMWTOOL_CLASS LogService {
public:
    void startServer(const std::string& logServiceIp, int16_t logServicePort);
    void stopServer();

    ~LogService() { stopServer(); }
    void setWriteHandler(std::function<void(const char* buffer, int len)> handler) { m_writeHandler = handler; }

private:
    void mainThread();
    void handleClient(std::shared_ptr<CSocket> connectedSocket);

private:
    std::atomic<bool> m_bRequestTerminate;
    OS_THREAD_HANDLE m_hServerThreadHandle;
    std::vector<OS_THREAD_HANDLE> m_hClientThreads;
    std::function<void(const char* buffer, int len)> m_writeHandler;
    CSocket m_socket;
};