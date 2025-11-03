#pragma once

// This file is just designed for simplify driver implementation,you should bind it with FrameworkCoreStub and
// LogSupportStub.

#include "HardwareManager/IHardwareCore.h"
#include "HardwareManager/IHardwareLink.h"
#include "HardwareManager/IHardwareEditor.h"
#include "HardwareManager/IHardwareListenerManager.h"
#include "HardwareManager/IRTHardwareCore.h"
#include "Cache/Cache.h"
#include "Environment/EndianType.h"
#include "Reference/Reference.h"
#include "StringAssistant/StringAssistant.h"
#include "HardwareManager/IHardwareImpl.h"

// you can define a macro named DEBUG_HARDWARE_TIMER for diagnosis.

DECLARE_STUB_INTERFACE(IHardwareImplStub, IHardwareImpl);

class CAbstractHardwareImpl : public IHardwareImplStub {
public:
    bool open() { return openImpl(); }

public:
    virtual bool openImpl() = 0;
    virtual bool getPropertyImpl(const char *name, char *value, DWORD &valueLen) = 0;
    virtual int writeImpl(const unsigned char *buf, int len) = 0;
    virtual int readImpl(unsigned char *buf, int len) = 0;
    virtual void closeImpl() = 0;
    virtual bool setPropertyImpl(const char *name, const char *value) = 0;
    virtual bool selectReadableImpl(int dwMillionTimeout) = 0;
    virtual bool selectWritableImpl(int dwMillionTimeout) = 0;
    virtual void clearImpl() = 0;
    virtual bool isLinkBrokenImpl() = 0;
    virtual void brokenLinkImpl() = 0;

protected:
    virtual void onFinalRelease() { clearImpl(); }
};

DECLARE_STUB_INTERFACE(IHardwareCoreStub, IHardwareCore);
DECLARE_STUB_CLASS(IHardwareLinkStub, IHardwareCoreStub, IHardwareLink);
DECLARE_STUB_CLASS(IHardwareEditorStub, IHardwareLinkStub, IHardwareEditor);
DECLARE_STUB_CLASS(IHardwareListenerManagerStub, IHardwareEditorStub, IHardwareListenerManager);
DECLARE_STUB_CLASS(IRTHardwareCoreStub, IHardwareListenerManagerStub, IRTHardwareCore);

DECLARE_STUB_INTERFACE(IBaseStub, IBase);

class RTHardwareTask : public IBaseStub {
public:
    struct RTBlock {
        RTBlock() {
            lpRef = new unsigned long;
            *lpRef = 1;
            m_pBlock = nullptr;
            m_ulBlockSize = 0;
        }
        RTBlock(ULONG ulBlockSize) {
            lpRef = new unsigned long;
            *lpRef = 1;
            if (ulBlockSize > 0)
                m_pBlock = new unsigned char[ulBlockSize];
            else
                m_pBlock = nullptr;
            m_ulBlockSize = ulBlockSize;
        }
        RTBlock(const RTHardwareTask::RTBlock &sndBlock) {
            lpRef = sndBlock.lpRef;
            m_pBlock = sndBlock.m_pBlock;
            m_ulBlockSize = sndBlock.m_ulBlockSize;
            OS::Thread::syncFetchAdd(lpRef);
            (*lpRef)++;
        }
        ~RTBlock() {
            m_ulBlockSize = 0;
            if (OS::Thread::syncFetchSub(lpRef) == 0) {
                if (m_pBlock != nullptr) {
                    delete[] m_pBlock;
                    m_pBlock = nullptr;
                }
            }
        }
        const RTHardwareTask::RTBlock &operator=(const RTHardwareTask::RTBlock &sndBlock) {
            if (OS::Thread::syncFetchSub(lpRef) == 0) {
                if (m_pBlock != nullptr) {
                    delete[] m_pBlock;
                    m_pBlock = nullptr;
                }
            }
            lpRef = sndBlock.lpRef;
            m_pBlock = sndBlock.m_pBlock;
            m_ulBlockSize = sndBlock.m_ulBlockSize;
            OS::Thread::syncFetchAdd(lpRef);
            return (*this);
        }
        unsigned char *m_pBlock;
        ULONG m_ulBlockSize;
        unsigned long *lpRef;
    };

public:
    RTHardwareTask(ULONG ulInterval) : m_SendBlockCache(2), m_RecvBlockCache(1) {
        m_ulTaskId = 0;
        m_ulInterval = ulInterval;
        m_ulSndBlockSize = 0;
        m_ulRecvBlockSize = 0;
        m_uTimerId = (DWORD_PTR) nullptr;
    }
    virtual ~RTHardwareTask() {}
    void setSndBlockSize(ULONG ulSndBlockSize) { m_ulSndBlockSize = ulSndBlockSize; }
    void setRecvBlockSize(ULONG ulRecvBlockSize) { m_ulRecvBlockSize = ulRecvBlockSize; }
    RTHardwareTask::RTBlock allocSndBlock() {
        RTHardwareTask::RTBlock sndBlock(m_ulSndBlockSize);
        return sndBlock;
    }
    RTHardwareTask::RTBlock allocRecvBlock() {
        RTHardwareTask::RTBlock recvBlock(m_ulRecvBlockSize);
        return recvBlock;
    }
    inline ULONG getSndBlockSize() { return m_ulSndBlockSize; }
    inline ULONG getRecvBlockSize() { return m_ulRecvBlockSize; }
    inline CCache<RTHardwareTask::RTBlock> *getSndBlockCache() { return &m_SendBlockCache; }
    inline CCache<RTHardwareTask::RTBlock> *getRecvBlockCache() { return &m_RecvBlockCache; }
    void attachTimer(UINT_PTR uTimerId) { m_uTimerId = uTimerId; }
    void detachTimer() {
        m_uTimerId = (DWORD_PTR) nullptr;
        m_SendBlockCache.reset();
        m_RecvBlockCache.reset();
    }
    inline UINT_PTR getTimerId() { return m_uTimerId; }

public:
    ULONG m_ulTaskId;
    ULONG m_ulInterval;

protected:
    UINT_PTR m_uTimerId;
    ULONG m_ulSndBlockSize;
    ULONG m_ulRecvBlockSize;
    CCache<RTHardwareTask::RTBlock> m_SendBlockCache;
    CCache<RTHardwareTask::RTBlock> m_RecvBlockCache;
};

DECLARE_STUB_INTERFACE(IAppTimerThreadInfoStub, IAppTimerThreadInfo)
DECLARE_STUB_CLASS(IRtHardwareHolderStub, IAppTimerThreadInfoStub, IAppTimerThreadCallback)

template <class hardware>
class CRtHardwareHolder : public IRtHardwareHolderStub {
protected:
    CRtHardwareHolder() {
#ifdef DEBUG_HARDWARE_TIMER
        m_lTimerCnt = 0;
        m_lMinInterval = 0;
        m_lMaxInterval = 0;
        m_bGetInitData = false;
#endif
    }
    virtual ~CRtHardwareHolder() {
#ifdef DEBUG_HARDWARE_TIMER
        LOG_DEBUG("Hardware Timer Delta:Min=%lu,Max=%lu\n", m_lMinInterval, m_lMaxInterval);
#endif
    }

public:
    static void createUniqueRtHardwareHolder() {
        if (m_pUniqueRtHardwareHolder == nullptr) {
            m_pUniqueRtHardwareHolder = new CRtHardwareHolder<hardware>();
            CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
            if (frameworkCore.nil() == false) {
                CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
                if (appTimerManager.nil() == false) {
                    appTimerManager->joinApplication(m_pUniqueRtHardwareHolder);
                }
            } else
                LOG_WARNING("Can't found 'FrameworkCore' component, IRTHardwareCore will not be realized");
        }
    }
    static CRtHardwareHolder<hardware> *getUniqueRtHardwareHolder() { return m_pUniqueRtHardwareHolder; }
    static void destroyUniqueRtHardwareHolder() {
        if (m_pUniqueRtHardwareHolder != nullptr) {
            CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
            if (frameworkCore.nil() == false) {
                CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
                if (appTimerManager.nil() == false) {
                    appTimerManager->exit(m_pUniqueRtHardwareHolder);
                }
            }
            m_pUniqueRtHardwareHolder->releaseRef();
            m_pUniqueRtHardwareHolder = nullptr;
        }
    }

public:
    void resetHolder() {
        LOG_DEBUG("CRtHardwareHolder is reset!\n");
#ifdef DEBUG_HARDWARE_TIMER
        m_lTimerCnt = 0;
        m_lMinInterval = 0;
        m_lMaxInterval = 0;
        m_bGetInitData = false;
#endif
    }

public:
    virtual UINT_PTR getHashCode() override { return (UINT_PTR)this; }
    virtual IAppTimerThreadCallback *getCallback() override { return this; }
    virtual void onTimer(HANDLE hOwnerObject, UINT_PTR timerId, LPVOID lpTimerData) override;

protected:
#ifdef DEBUG_HARDWARE_TIMER
    timespec m_tNextTime;
    long m_lTimerCnt;
    bool m_bGetInitData;
    ULONG m_lMinInterval, m_lMaxInterval;
#endif
protected:
    static CRtHardwareHolder<hardware> *m_pUniqueRtHardwareHolder;
};

template <class impl, class hardware>
class CAbstractHardware : public IRTHardwareCoreStub, public impl {
    typedef CReference<IHardwareListener> IHardwareListenerRef;

protected:
    enum INTERFACE_MSG {
        OPEN_MESSAGE = 1,
        CLOSE_MESSAGE = 2,
    };

public:
    CAbstractHardware() {
        m_bOpen = false;
        m_ulNextHardwareTaskId = 1;
    }
    ~CAbstractHardware() { destroyAllTask_P(); }

public:
    bool initHardware(const char *strHardwareName, const char *strHardwareConfiguration) {
        if (strHardwareName != nullptr && strHardwareConfiguration != nullptr) {
            m_strHardwareName = strHardwareName;
            m_strHardwareConfiguration = strHardwareConfiguration;
            return true;
        } else
            return false;
    }
    void update(RTHardwareTask *pRTHardwareTask) {
        if (impl::selectReadableImpl(0) == true) {
            RTHardwareTask::RTBlock recvBlock = pRTHardwareTask->allocRecvBlock();
            int iLen = impl::readImpl(recvBlock.m_pBlock, (int)recvBlock.m_ulBlockSize);
            if (iLen == (int)recvBlock.m_ulBlockSize) {
                if (pRTHardwareTask->getRecvBlockCache()->push(recvBlock) == false) {
                    LOG_DEBUG("Receive-cache is full，ignore frame!\n");
                }
            } else
                LOG_DEBUG("Receive error，%d bytes is received!\n", iLen);
        } else {
            LOG_DEBUG("Don't get any bytes!\n");
        }
        RTHardwareTask::RTBlock sndBlock;
        if (pRTHardwareTask->getSndBlockCache()->tryPop(sndBlock) == true) {
            impl::writeImpl(sndBlock.m_pBlock, (int)pRTHardwareTask->getSndBlockSize());
        } else {
            LOG_DEBUG("No data will be send!\n");
        }
    }

public:
    virtual bool queryInterface(const char *name, void **pInterface) override {
        bool ret = IRTHardwareCoreStub::queryInterface(name, pInterface);
        if (ret == false) ret = impl::queryInterface(name, pInterface);
        return ret;
    }
    virtual ULONG addRef() override { return IRTHardwareCoreStub::addRef(); }
    virtual ULONG releaseRef() override { return IRTHardwareCoreStub::releaseRef(); }
    virtual void onFinalRelease() override {
        IRTHardwareCoreStub::onFinalRelease();
        impl::onFinalRelease();
    }
    virtual bool select(HARDWARE_SELECT_PROPERTY iProperty, long dwMilliSeconds) override {
        if (iProperty == HARDWARE_READABLE)
            return impl::selectReadableImpl((int)dwMilliSeconds);
        else if (iProperty == HARDWARE_WRITABLE)
            return impl::selectWritableImpl((int)dwMilliSeconds);
        else
            return false;
    }
    virtual bool open() override {
        m_bOpen = impl::open();
        notifyInterfaceListeners(CAbstractHardware::OPEN_MESSAGE, m_bOpen == true ? 1 : 0);
        return m_bOpen;
    }
    virtual bool getProperty(const char *name, char *value, DWORD &valueLen) override {
        long ret = impl::getPropertyImpl(name, value, valueLen);
        return ret;
    }
    virtual long write(const unsigned char *pBuffer, long iBufLen) override {
        long ret = impl::writeImpl(pBuffer, (int)iBufLen);
        return ret;
    }
    virtual long read(unsigned char *pBuffer, long iBufLen) override {
        long ret = impl::readImpl(pBuffer, (int)iBufLen);
        return ret;
    }
    virtual bool exchange(const unsigned char *inputBuf, long lInLen, unsigned char *outputBuf,
                          long lOutBufLen) override {
        bool ret = false;
        if (impl::writeImpl(inputBuf, (int)lInLen) == lInLen &&
            impl::readImpl(outputBuf, (int)lOutBufLen) == lOutBufLen)
            ret = true;
        return ret;
    }
    virtual void clearAllCache() override { impl::clearImpl(); }
    virtual void close() override {
        impl::closeImpl();
        m_bOpen = false;
        notifyInterfaceListeners(CLOSE_MESSAGE, 0);
    }
    virtual bool isOpen() override { return m_bOpen; }
    virtual bool getHardwareConfiguration(char *strHardwareConfiguration, DWORD &dwLen) override {
        return CStringAssistant::getBaseStringFromSTLString(m_strHardwareConfiguration, strHardwareConfiguration,
                                                            dwLen);
    }
    virtual bool isLinkBroken() { return impl::isLinkBrokenImpl(); }
    virtual void brokenLink() { return impl::brokenLinkImpl(); }
    virtual unsigned long createTask(unsigned long ulInterval, TASK_FLAGS uTaskFlags) override {
        ULONG uTaskId = INVALID_TASK_ID;
        CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
        if (frameworkCore.nil() == true)
            return uTaskId;
        else {
            CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
            if (appTimerManager.nil() == true)
                return uTaskId;
            else {
                RTHardwareTask *pRTHardwareTask = new RTHardwareTask(ulInterval);
                if (pRTHardwareTask != nullptr) {
                    uTaskId = allocHardwareTaskId();
                    if (uTaskId == INVALID_TASK_ID)
                        return uTaskId;
                    else {
                        m_TaskMapMutex.lock();
                        if (m_RTHardwareTaskMap.empty() == true) {
                            CRtHardwareHolder<hardware> *pUniqueRtHardwareHolder =
                                CRtHardwareHolder<hardware>::getUniqueRtHardwareHolder();
                            if (pUniqueRtHardwareHolder != nullptr) pUniqueRtHardwareHolder->resetHolder();
                        }
                        pRTHardwareTask->m_ulTaskId = uTaskId;
                        m_RTHardwareTaskMap.insert(pair<ULONG, RTHardwareTask *>(uTaskId, pRTHardwareTask));
                        m_TaskMapMutex.unlock();
                        if ((uTaskFlags & CREATE_SUSPENDED_TASK) == 0) resumeTask(uTaskId);
                        return uTaskId;
                    }
                } else
                    return uTaskId;
            }
        }
    }
    virtual bool suspendTask(unsigned long ulTaskId) override {
        CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
        if (frameworkCore.nil() == false) {
            CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
            CRtHardwareHolder<hardware> *pUniqueRtHardwareHolder =
                CRtHardwareHolder<hardware>::getUniqueRtHardwareHolder();
            RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
            if (pRTHardwareTask != nullptr) {
                if (appTimerManager.nil() == false && pUniqueRtHardwareHolder != nullptr) {
                    appTimerManager->killTimer(pRTHardwareTask->getTimerId(), pUniqueRtHardwareHolder);
                    pRTHardwareTask->detachTimer();
                }
                pRTHardwareTask->releaseRef();
                return true;
            } else
                return false;
        } else
            return false;
    }
    virtual bool resumeTask(unsigned long ulTaskId) override {
        CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
        if (frameworkCore.nil() == false) {
            CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
            CRtHardwareHolder<hardware> *pUniqueRtHardwareHolder =
                CRtHardwareHolder<hardware>::getUniqueRtHardwareHolder();
            RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
            if (pRTHardwareTask != nullptr) {
                if (appTimerManager.nil() == false && pUniqueRtHardwareHolder != nullptr) {
                    UINT_PTR uTimerId =
                        appTimerManager->setTimer(this, pRTHardwareTask->m_ulInterval, 0, HIGH_PRECISION_TIMER,
                                                  pRTHardwareTask, pUniqueRtHardwareHolder);
                    pRTHardwareTask->attachTimer(uTimerId);
                }
                pRTHardwareTask->releaseRef();
                return true;
            } else
                return false;
        } else
            return false;
    }
    virtual bool destroyTask(unsigned long ulTaskId) override {
        CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
        if (frameworkCore.nil() == false) {
            CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
            CRtHardwareHolder<hardware> *pUniqueRtHardwareHolder =
                CRtHardwareHolder<hardware>::getUniqueRtHardwareHolder();
            RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
            if (pRTHardwareTask != nullptr) {
                if (appTimerManager.nil() == false && pUniqueRtHardwareHolder != nullptr) {
                    appTimerManager->killTimer(pRTHardwareTask->getTimerId(), pUniqueRtHardwareHolder);
                    pRTHardwareTask->detachTimer();
                }
                pRTHardwareTask->releaseRef();
            } else
                return false;
        } else
            return false;
        m_TaskMapMutex.lock();
        map<ULONG, RTHardwareTask *>::iterator it = m_RTHardwareTaskMap.find(ulTaskId);
        if (it != m_RTHardwareTaskMap.end()) {
            it->second->releaseRef();
            m_RTHardwareTaskMap.erase(it);
        }
        m_TaskMapMutex.unlock();
        return true;
    }
    virtual void destroyAllTask() override { destroyAllTask_P(); }
    virtual bool getTaskOpt(unsigned long ulTaskId, unsigned long optname, void *value, long valuesize) override {
        bool ret = false;
        RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
        if (pRTHardwareTask != nullptr && value != nullptr) {
            if (SNDBLOCKSIZE == optname && sizeof(unsigned long) == valuesize) {
                unsigned long lSndBlockSize = pRTHardwareTask->getSndBlockSize();
                *((unsigned long *)value) = lSndBlockSize;
                ret = true;
            } else if (RECVBLOCKSIZE == optname && sizeof(unsigned long) == valuesize) {
                unsigned long lRecvBlockSize = pRTHardwareTask->getRecvBlockSize();
                *((unsigned long *)value) = lRecvBlockSize;
                ret = true;
            } else
                ret = false;
            pRTHardwareTask->releaseRef();
        }
        return ret;
    }
    virtual bool setTaskOpt(unsigned long ulTaskId, unsigned long optname, const void *value, long valuesize) override {
        bool ret = false;
        RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
        if (pRTHardwareTask != nullptr && value != nullptr) {
            if (SNDBLOCKSIZE == optname && sizeof(unsigned long) == valuesize) {
                long lSndBlockSize = *((unsigned long *)value);
                if (lSndBlockSize >= 0) pRTHardwareTask->setSndBlockSize(lSndBlockSize);
                ret = true;
            } else if (RECVBLOCKSIZE == optname && sizeof(unsigned long) == valuesize) {
                long lRecvBlockSize = *((unsigned long *)value);
                if (lRecvBlockSize >= 0) pRTHardwareTask->setRecvBlockSize(lRecvBlockSize);
                ret = true;
            } else
                ret = false;
            pRTHardwareTask->releaseRef();
        }
        return ret;
    }
    virtual bool exchange(unsigned long ulTaskId, const unsigned char *inputBuf, long lInLen, unsigned char *outputBuf,
                          long lOutBufLen) override {
        bool ret = true;
        RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
        if (pRTHardwareTask != nullptr) {
            if (pRTHardwareTask->getSndBlockSize() > 0) {
                if (lInLen == (long)pRTHardwareTask->getSndBlockSize()) {
                    RTHardwareTask::RTBlock sndBlock = pRTHardwareTask->allocSndBlock();
                    memcpy(sndBlock.m_pBlock, inputBuf, lInLen);
                    if (pRTHardwareTask->getSndBlockCache()->push(sndBlock) == false) ret = false;
                } else
                    ret = false;
            }
            if (pRTHardwareTask->getRecvBlockSize() > 0) {
                if (lOutBufLen == (long)pRTHardwareTask->getRecvBlockSize()) {
                    RTHardwareTask::RTBlock recvBlock;
                    if (pRTHardwareTask->getRecvBlockCache()->tryPop(recvBlock) == true) {
                        memcpy(outputBuf, recvBlock.m_pBlock, lOutBufLen);
                    } else
                        ret = false;
                } else
                    ret = false;
            }
            pRTHardwareTask->releaseRef();
        } else
            ret = false;
        return ret;
    }
    virtual bool exchangeInput(unsigned long ulTaskId, const unsigned char *inputBuf, long lInLen) override {
        bool ret = true;
        RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
        if (pRTHardwareTask != nullptr) {
            if (pRTHardwareTask->getSndBlockSize() > 0) {
                if (lInLen == (long)pRTHardwareTask->getSndBlockSize()) {
                    RTHardwareTask::RTBlock sndBlock = pRTHardwareTask->allocSndBlock();
                    memcpy(sndBlock.m_pBlock, inputBuf, lInLen);
                    if (pRTHardwareTask->getSndBlockCache()->push(sndBlock) == false) ret = false;
                } else
                    ret = false;
            }
            pRTHardwareTask->releaseRef();
        } else
            ret = false;
        return ret;
    }
    virtual bool exchangeOutput(unsigned long ulTaskId, unsigned char *outputBuf, long lOutBufLen) override {
        bool ret = true;
        RTHardwareTask *pRTHardwareTask = getRTHardwareTask(ulTaskId);
        if (pRTHardwareTask != nullptr) {
            if (pRTHardwareTask->getRecvBlockSize() > 0) {
                if (lOutBufLen == (long)pRTHardwareTask->getRecvBlockSize()) {
                    RTHardwareTask::RTBlock recvBlock;
                    if (pRTHardwareTask->getRecvBlockCache()->tryPop(recvBlock) == true) {
                        memcpy(outputBuf, recvBlock.m_pBlock, lOutBufLen);
                    } else
                        ret = false;
                } else
                    ret = false;
            }
            pRTHardwareTask->releaseRef();
        } else
            ret = false;
        return ret;
    }
    virtual bool setProperty(const char *name, const char *value) override {
        return impl::setPropertyImpl(name, value);
    }
    virtual void addListener(IHardwareListener *ptListener) override {
        TAutoMutex<OS::TRecursiveMutex> m_AutoMutex(&m_OperateMutex);
        if (ptListener != nullptr) {
            list<IHardwareListenerRef>::iterator it = m_InterfaceListenerList.begin();
            while (it != m_InterfaceListenerList.end()) {
                if (ptListener == (IHardwareListener *)(*it)) return;
                it++;
            }
            ptListener->addRef();
            m_InterfaceListenerList.push_back(ptListener);
        }
    }
    virtual void removeListener(IHardwareListener *ptListener) override {
        TAutoMutex<OS::TRecursiveMutex> m_AutoMutex(&m_OperateMutex);
        if (ptListener != nullptr) {
            list<IHardwareListenerRef>::iterator it = m_InterfaceListenerList.begin();
            while (it != m_InterfaceListenerList.end()) {
                if (ptListener == (IHardwareListener *)(*it)) {
                    m_InterfaceListenerList.erase(it);
                    return;
                }
                it++;
            }
        }
    }
    virtual void removeAllListener() override {
        TAutoMutex<OS::TRecursiveMutex> m_AutoMutex(&m_OperateMutex);
        m_InterfaceListenerList.clear();
    }

protected:
    void notifyInterfaceListeners(INTERFACE_MSG msg,
                                  int iParam)  //该函数在加锁状态下访问了IHardwareListener，需要格外小心死锁
    {
        TAutoMutex<OS::TRecursiveMutex> m_AutoMutex(&m_OperateMutex);
        list<IHardwareListenerRef>::iterator it = m_InterfaceListenerList.begin();
        while (it != m_InterfaceListenerList.end()) {
            if (msg == CLOSE_MESSAGE) {
                m_bOpen = false;
                (*it)->onInterfaceClose();
            } else {
                m_bOpen = true;
                (*it)->onInterfaceOpen(iParam == 0 ? false : true);
            }
            it++;
        }
    }
    ULONG allocHardwareTaskId() {
        ULONG ulTaskId = INVALID_TASK_ID;
        m_TaskMapMutex.lock();
        ULONG uSearchFromTaskId = m_ulNextHardwareTaskId;
        while (true) {
            if (m_ulNextHardwareTaskId == INVALID_TASK_ID) {
                m_ulNextHardwareTaskId++;
                continue;
            } else {
                map<ULONG, RTHardwareTask *>::iterator it = m_RTHardwareTaskMap.find(m_ulNextHardwareTaskId);
                if (it == m_RTHardwareTaskMap.end()) {
                    ulTaskId = m_ulNextHardwareTaskId;
                    m_ulNextHardwareTaskId++;
                    break;
                } else {
                    m_ulNextHardwareTaskId++;
                    if (uSearchFromTaskId == m_ulNextHardwareTaskId) break;
                }
            }
        }
        m_TaskMapMutex.unlock();
        return ulTaskId;
    }
    RTHardwareTask *getRTHardwareTask(ULONG ulTaskId) {
        RTHardwareTask *pRTHardwareTask = nullptr;
        m_TaskMapMutex.lock();
        map<ULONG, RTHardwareTask *>::iterator it = m_RTHardwareTaskMap.find(ulTaskId);
        if (it != m_RTHardwareTaskMap.end()) {
            pRTHardwareTask = it->second;
            pRTHardwareTask->addRef();
        }
        m_TaskMapMutex.unlock();
        return pRTHardwareTask;
    }
    string getHardwareConfiguration() { return m_strHardwareConfiguration; }

private:
    void destroyAllTask_P() {
        list<RTHardwareTask *> taskList;
        m_TaskMapMutex.lock();
        map<ULONG, RTHardwareTask *>::iterator it = m_RTHardwareTaskMap.begin();
        while (it != m_RTHardwareTaskMap.end()) {
            taskList.push_back(it->second);
            it++;
        }
        m_RTHardwareTaskMap.clear();
        m_TaskMapMutex.unlock();
        CReference<IFrameworkCoreInner> frameworkCore = CFrameworkCore::getResource();
        if (frameworkCore.nil() == false) {
            CReference<IAppTimerManager> appTimerManager = frameworkCore->getAppTimerManager();
            CRtHardwareHolder<hardware> *pUniqueRtHardwareHolder =
                CRtHardwareHolder<hardware>::getUniqueRtHardwareHolder();
            while (taskList.empty() == false) {
                RTHardwareTask *pRTHardwareTask = taskList.front();
                taskList.pop_front();
                if (appTimerManager.nil() == false && pUniqueRtHardwareHolder != nullptr) {
                    appTimerManager->killTimer(pRTHardwareTask->getTimerId(), pUniqueRtHardwareHolder);
                    pRTHardwareTask->detachTimer();
                }
                pRTHardwareTask->releaseRef();
                pRTHardwareTask = nullptr;
            }
        }
    }

protected:
    OS::TRecursiveMutex m_OperateMutex;
    string m_strHardwareName;
    string m_strHardwareConfiguration;
    list<IHardwareListenerRef> m_InterfaceListenerList;
    map<ULONG, RTHardwareTask *> m_RTHardwareTaskMap;
    OS::TRecursiveMutex m_TaskMapMutex;
    ULONG m_ulNextHardwareTaskId;
    bool m_bOpen;
};

template <class hardware>
CRtHardwareHolder<hardware> *CRtHardwareHolder<hardware>::m_pUniqueRtHardwareHolder = nullptr;

template <class hardware>
void CRtHardwareHolder<hardware>::onTimer(HANDLE hOwnerObject, UINT_PTR timerId, LPVOID lpTimerData) {
#ifdef DEBUG_HARDWARE_TIMER
    int tv_sub_sec = 0, tv_sub_nsec = 0;
    long delay_nsec = 0;
    unsigned long ulVibrate = 0;
    timespec currentTime;
    if (m_lTimerCnt == 0) {
        OS::Clock::getTime(CLOCK_MONOTONIC, &m_tNextTime);
        m_lMinInterval = 0;
        m_lMaxInterval = 0;
        m_bGetInitData = false;
    } else {
        OS::Clock::getTime(CLOCK_MONOTONIC, &currentTime);
        tv_sub_sec = (int)(currentTime.tv_sec - m_tNextTime.tv_sec);
        tv_sub_nsec = (int)(currentTime.tv_nsec - m_tNextTime.tv_nsec);
        delay_nsec = tv_sub_nsec + tv_sub_sec * NSEC_PER_SEC;
        ulVibrate = (unsigned long)ABS(delay_nsec);
        // if(delay_nsec>0)printf("Timer Overflow:%ldns!!!\n", delay_nsec);
        if (ulVibrate < m_lMinInterval || m_bGetInitData == false) m_lMinInterval = ulVibrate;
        if (ulVibrate > m_lMaxInterval || m_bGetInitData == false) m_lMaxInterval = ulVibrate;
        m_bGetInitData = true;
    }
    m_lTimerCnt++;
    m_tNextTime.tv_nsec += 5000000;
    while (m_tNextTime.tv_nsec >= NSEC_PER_SEC) {
        m_tNextTime.tv_nsec -= NSEC_PER_SEC;
        m_tNextTime.tv_sec++;
    }
#endif
    hardware *pHardware = (hardware *)hOwnerObject;
    RTHardwareTask *pRtHardwareTask = (RTHardwareTask *)lpTimerData;
    if (pHardware != nullptr && pRtHardwareTask != nullptr) pHardware->update(pRtHardwareTask);
}
