// 该文件用于辅助应用层的程序编写，本模块中不使用
#pragma once

#include "ApartmentApi.h"
#include "BoundedStream/BoundedStream.h"

DECLARE_STUB_INTERFACE(IApartmentNodeCallbackStub, IApartmentNodeCallback);

class CApartmentNodeCallback : public IApartmentNodeCallbackStub {
public:
    CApartmentNodeCallback() {
        m_uintRequestBufferSize = 0;
        m_pRequestBuffer = nullptr;
        m_uintResponseBufferSize = 0;
        m_pResponseBuffer = nullptr;
    }
    ~CApartmentNodeCallback() {
        if (m_pResponseBuffer != nullptr) {
            delete[] m_pResponseBuffer;
            m_pResponseBuffer = nullptr;
        }
        m_uintResponseBufferSize = 0;
        if (m_pRequestBuffer != nullptr) {
            delete[] m_pRequestBuffer;
            m_pRequestBuffer = nullptr;
        }
        m_uintRequestBufferSize = 0;
    }

public:
    void ensureRequestBuffer(unsigned int uintBufLen) {
        if (m_uintRequestBufferSize < uintBufLen) {
            if (m_pRequestBuffer != nullptr) {
                delete[] m_pRequestBuffer;
                m_pRequestBuffer = nullptr;
            }
            m_uintRequestBufferSize = uintBufLen;
            m_pRequestBuffer = new char[m_uintRequestBufferSize];
        }
    }
    void ensureResponseBuffer(unsigned int uintBufLen) {
        if (m_uintResponseBufferSize < uintBufLen) {
            if (m_pResponseBuffer != nullptr) {
                delete[] m_pResponseBuffer;
                m_pResponseBuffer = nullptr;
            }
            m_uintResponseBufferSize = uintBufLen;
            m_pResponseBuffer = new char[m_uintResponseBufferSize];
        }
    }

protected:
    virtual void onTimer(unsigned int uintTimerId) {}
    virtual void onRequestReceived(const char *strSourceApartmentUrl, const char *strSourceNodeUrl,
                                   const char *strTargetApartmentUrl, const char *strTargetNodeUrl,
                                   unsigned int uintRequestId, const void *pRequestBuffer, unsigned int uintBufLen,
                                   bool bRequireResponse) {
        m_strRequestApartmentUrl = strSourceApartmentUrl;
        m_strRequestNodeUrl = strSourceNodeUrl;
        CCacheStream requestStream;
        if (uintBufLen > 0) {
            // 先确保m_pRequestBuffer的大小足够容纳pRequestBuffer的内容
            ensureRequestBuffer(uintBufLen);
            // 将requestStream与m_pRequestBuffer绑定，避免内存分配
            requestStream.attach(m_pRequestBuffer, m_uintRequestBufferSize);
            requestStream.writeObject(pRequestBuffer, uintBufLen);
        }
        CBoundedStream responseStream;
        int ret = onNodeRequest(requestStream, responseStream);
        if (bRequireResponse == false) {
            return;
        }
        if (ret == REQUEST_OK) {
            size_t szRespSize = responseStream.getLength();
            if (szRespSize > 0) {
                ensureResponseBuffer(szRespSize);
                responseStream.readObject(m_pResponseBuffer, szRespSize);
            }
            apartment_node_write_response(strTargetApartmentUrl, strTargetNodeUrl, strSourceApartmentUrl,
                                          strSourceNodeUrl, uintRequestId, REQUEST_OK, m_pResponseBuffer, szRespSize);
            return;
        } else if (ret == REQUEST_METHOD_NOT_FOUND) {
            apartment_node_write_response(strTargetApartmentUrl, strTargetNodeUrl, strSourceApartmentUrl,
                                          strSourceNodeUrl, uintRequestId, REQUEST_METHOD_NOT_FOUND, nullptr, 0);
            return;
        } else if (ret == REQUEST_OPERATE_FAILED) {
            apartment_node_write_response(strTargetApartmentUrl, strTargetNodeUrl, strSourceApartmentUrl,
                                          strSourceNodeUrl, uintRequestId, REQUEST_OPERATE_FAILED, nullptr, 0);
            return;
        } else {
            apartment_node_write_response(strTargetApartmentUrl, strTargetNodeUrl, strSourceApartmentUrl,
                                          strSourceNodeUrl, uintRequestId, REQUEST_PARAMETER_ERROR, nullptr, 0);
            return;
        }
    }
    virtual void onApartmentOffline(const char *strSourceApartmentUrl) override {
        onNodeOffline(strSourceApartmentUrl);
    }
    virtual int onNodeRequest(CAbstractStream &requestStream, CAbstractStream &responseStream) = 0;
    // node所在的apartment断线，strRemoteUrl：127.0.0.1:7500
    virtual void onNodeOffline(const char *strRemoteUrl) {}

protected:
    unsigned int m_uintRequestBufferSize;
    char *m_pRequestBuffer;
    unsigned int m_uintResponseBufferSize;
    char *m_pResponseBuffer;
    string m_strRequestApartmentUrl;  // 当前请求方所属的Apartment
    string m_strRequestNodeUrl;       // 当前请求方所属的Node
};
