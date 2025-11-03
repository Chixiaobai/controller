
#pragma once

#include "Reference/Reference.h"
#include "Socket/Socket.h"
#include "../AmwToolDef.h"

/*  网络协议
        bodylen(4)+bodyType(1)+bodyTarget(1)+BodyName(不定长）+body(不定长）
        bodylen:4字节整数，表示body的长度
        bodyType:1字节整数，表示body的类型:STDOUT\STDERR
        bodyTarget:1字节整数，表示消息的目标:Center\Host\Node
        BodyName:表示消息的名称,根据NodeName(后续+HostIp)
        body:表示具体的消息内容，来源由bodyType决定
*/

class AMWTOOL_CLASS LogServiceClient {
public:
    LogServiceClient();
    ~LogServiceClient() = default;

    int connect(const std::string& ip, int port);
    int send(const char* buffer, size_t size);
    bool close();

private:
    CReference<CSocket> m_socket;
};
