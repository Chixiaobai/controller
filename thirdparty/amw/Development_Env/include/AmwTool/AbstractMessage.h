
#pragma once
#include "stdafx.h"
#include <BoundedStream/BoundedStream.h>

struct Message {
    virtual void serialize(CAbstractStream& reqStream) const=0;
    virtual void deserialize(CAbstractStream& respStream)=0;
    virtual ~Message()=default;
    void serializeToArray(void*& data, size_t& size) const {
        CBoundedStream reqStream;
        serialize(reqStream);
        int len = (int)reqStream.getLength();
        void* temp = malloc(len);
        reqStream.readObject(temp, len);
        data = temp;
        size = len;
    }

    void deserializeFromArray(const void* data, size_t size) {
        CBoundedStream reqStream;
        reqStream.writeObject(data, size);
        deserialize(reqStream);
    }
};

struct BaseServicePacket {
    BaseServicePacket(Message* req, Message* resp) : m_req(req), m_resp(resp) {}
    Message* m_req;
    Message* m_resp;
};