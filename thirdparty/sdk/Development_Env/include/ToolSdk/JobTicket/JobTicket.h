/**
 * @file JobTicket.h
 * @brief Header file for job ticket management.
 *
 * This file defines interfaces and a template class for managing jobs with resources.
 */

#pragma once

#include "Base/IBase.h"
#include "Reference/Reference.h"
#include "OS/OS_Sync.h"

/**
 * @interface IJobTicketResource
 * @brief Interface for job ticket resources.
 *
 * This interface defines methods for installing and uninstalling resources associated with a job ticket.
 */
interface IJobTicketResource : public IBase {
    /**
     * @brief Installs the resource.
     *
     * Installs the resource required for the job.
     *
     * @return true if the resource was installed successfully, false otherwise.
     */
    virtual bool onInstallResource() = 0;

    /**
     * @brief Uninstalls the resource.
     *
     * Uninstalls the resource associated with the job.
     */
    virtual void onUninstallResource() = 0;
};

/**
 * @brief Macro to enable recognition of IJobTicketResource interface.
 */
ENABLE_RECOGNIZE_MACRO(IJobTicketResource);

/**
 * @brief Stub interface for IJobTicketResource.
 */
DECLARE_STUB_INTERFACE(IJobTicketResourceStub, IJobTicketResource);

/**
 * @class CJobTicket
 * @brief Template class for managing jobs with resources.
 *
 * This template class manages jobs that can be either single-shot or periodic, and can have associated resources.
 * It handles job creation, destruction, and resource management.
 *
 * @tparam T The type of the object that contains the job function.
 */
template <typename T>
class CJobTicket {
    /**
     * @typedef MEMBLER_FUNCTION
     * @brief Typedef for member function pointers.
     */
    typedef void (T::*MEMBLER_FUNCTION)();

public:
    /**
     * @enum JOB_TYPE
     * @brief Enumerates the types of jobs.
     */
    enum JOB_TYPE {
        SINGLE_SHOT_JOB = 0, /**< Single-shot job type. */
        PERIODIC_JOB = 1     /**< Periodic job type. */
    };

public:
    /**
     * @brief Constructor for CJobTicket.
     *
     * Initializes the job ticket with default values.
     */
    CJobTicket()
        : m_pObject(nullptr),
          m_pFunc(nullptr),
          m_intJobType(SINGLE_SHOT_JOB),
          m_uintInterval(0),
          m_bResourceEnable(false),
          m_hIdleEvent(true, true),
          m_hDoNextJobEvent(false, false),
          m_hJobThread((OS_THREAD_HANDLE)NULL),
          m_bRequestStop(false) {}

    /**
     * @brief Destructor for CJobTicket.
     *
     * Cleans up the job ticket by destroying the job and removing all resources.
     */
    virtual ~CJobTicket() {
        destroyJob();
        removeAllResource();
    }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CJobTicket objects to ensure proper management of job resources.
     */
    CJobTicket(const CJobTicket &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CJobTicket objects to ensure proper management of job resources.
     */
    CJobTicket &operator=(const CJobTicket &) = delete;
#endif

public:
    /**
     * @brief Creates a job.
     *
     * Creates a job with the specified object, function, type, and interval.
     *
     * @param pObject Pointer to the object containing the job function.
     * @param func Pointer to the member function to be executed as the job.
     * @param type The type of the job (SINGLE_SHOT_JOB or PERIODIC_JOB).
     * @param uintInterval The interval for periodic jobs in milliseconds.
     * @return true if the job was created successfully, false otherwise.
     */
    bool createJob(T *pObject, MEMBLER_FUNCTION func, JOB_TYPE type = SINGLE_SHOT_JOB, unsigned int uintInterval = 0) {
        bool ret = false;
        m_JobMutex.lock();
        if (m_hJobThread == (OS_THREAD_HANDLE)NULL) {
            m_pObject = pObject;
            m_pFunc = func;
            m_intJobType = type;
            m_uintInterval = uintInterval;
            m_bRequestStop = false;
            OS::Thread::create(&m_hJobThread, onJobFunction, this);
            ret = true;
        }
        m_JobMutex.unlock();
        return ret;
    }

    /**
     * @brief Destroys a job.
     *
     * Destroys the job and waits for the job thread to finish.
     */
    void destroyJob() {
        OS_THREAD_HANDLE hJobThread = (OS_THREAD_HANDLE)NULL;
        m_JobMutex.lock();
        hJobThread = m_hJobThread;
        m_bRequestStop = true;
        m_hJobThread = (OS_THREAD_HANDLE)NULL;
        m_JobMutex.unlock();
        if (hJobThread != (OS_THREAD_HANDLE)NULL) {
            OS::Thread::join(hJobThread);
        }
        m_JobMutex.lock();
        m_bRequestStop = false;
        m_JobMutex.unlock();
    }

    /**
     * @brief Adds a resource to the job.
     *
     * Adds a resource to the job with the specified name.
     *
     * @param strResourceName The name of the resource.
     * @param pResource Pointer to the resource to be added.
     * @return true if the resource was added successfully, false otherwise.
     */
    bool addResource(const char *strResourceName, IJobTicketResource *pResource) {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        if (strResourceName == nullptr || pResource == nullptr)
            return false;
        else {
            map<string, CReference<IJobTicketResource>>::iterator it = m_JobTicketResourceMap.find(strResourceName);
            if (it == m_JobTicketResourceMap.end()) {
                CReference<IJobTicketResource> res = pResource;
                pResource = nullptr;
                m_JobTicketResourceMap.insert(pair<string, CReference<IJobTicketResource>>(strResourceName, res));
                return true;
            } else {
                return false;
            }
        }
    }

    /**
     * @brief Removes a resource from the job.
     *
     * Removes the resource with the specified name from the job.
     *
     * @param strResourceName The name of the resource to be removed.
     * @return true if the resource was removed successfully, false otherwise.
     */
    bool removeResource(const char *strResourceName) {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        if (strResourceName == nullptr)
            return false;
        else {
            map<string, CReference<IJobTicketResource>>::iterator it = m_JobTicketResourceMap.find(strResourceName);
            if (it != m_JobTicketResourceMap.end()) {
                m_JobTicketResourceMap.erase(it);
                return true;
            } else {
                return false;
            }
        }
    }

    /**
     * @brief Removes all resources from the job.
     *
     * Removes all resources associated with the job.
     */
    void removeAllResource() {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        m_JobTicketResourceMap.clear();
    }

    /**
     * @brief Retrieves a job resource.
     *
     * Retrieves the resource with the specified name.
     *
     * @param strResourceName The name of the resource to retrieve.
     * @return Pointer to the retrieved resource, or nullptr if the resource does not exist.
     */
    IJobTicketResource *getJobResource(const char *strResourceName) {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        if (strResourceName == nullptr)
            return nullptr;
        else {
            map<string, CReference<IJobTicketResource>>::iterator it = m_JobTicketResourceMap.find(strResourceName);
            if (it != m_JobTicketResourceMap.end()) {
                it->second->addRef();
                return it->second;
            } else {
                return nullptr;
            }
        }
    }

    /**
     * @brief Checks if a stop request has been made.
     *
     * Checks if a stop request has been made for the job.
     *
     * @return true if a stop request has been made, false otherwise.
     */
    bool isRequestStop() {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        return m_bRequestStop;
    }

public:
    /**
     * @brief Sets the resource enabled state.
     *
     * Sets whether the resources associated with the job are enabled.
     *
     * @param bEnabled true to enable the resources, false to disable them.
     * @return true if the resources were set successfully, false otherwise.
     */
    bool setResourceEnabled(bool bEnabled) {
        bool ret = true;
        if (bEnabled == false) {
            m_JobMutex.lock();
            m_bResourceEnable = bEnabled;  // Mark resources as unavailable, but they may still be in use
            m_JobMutex.unlock();
            m_hIdleEvent.wait();  // Wait for resources to be no longer in use
            // Add code to release resources
            uninstallAllJobTicketResource();
        } else {
            m_JobMutex.lock();
            if (m_bResourceEnable == false) {
                // Add code to acquire resources
                if (installAllJobTicketResource() == true) {
                    m_bResourceEnable = true;      // Mark resources as ready, resources are not ready until this point
                    m_hDoNextJobEvent.setEvent();  // Immediately start executing the job
                } else {
                    printf("Job will not start because not all resources can be installed.\n");
                    ret = false;
                }
            }
            m_JobMutex.unlock();
        }
        return ret;
    }

protected:
    /**
     * @brief Sets the job active state.
     *
     * Sets whether the job is active and can use resources.
     *
     * @param bActive true to activate the job, false to deactivate it.
     * @return true if the job was set successfully, false otherwise.
     */
    bool setJobActive(bool bActive) {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        if (bActive == true) {
            bool ret = (m_bResourceEnable == true);
            if (ret == true) {
                m_hIdleEvent.resetEvent();  // Mark job as active, will start using resources, resources must be marked
                                            // as inactive after use
            }
            return ret;
        } else {
            m_hIdleEvent.setEvent();  // Mark job as inactive, indicating resources are no longer in use
            return true;
        }
    }

    /**
     * @brief Executes the job.
     *
     * Executes the job repeatedly until a stop request is made.
     */
    void doJob() {
        while (m_bRequestStop == false) {
            if (setJobActive(true)) {
                ((T *)m_pObject->*m_pFunc)();  // Execute the job function
                m_JobMutex.lock();
                if (SINGLE_SHOT_JOB == m_intJobType) {
                    m_bRequestStop = true;  // Stop after executing a single-shot job
                }
                m_JobMutex.unlock();
                setJobActive(false);
            }
            unsigned int uintInterval = m_uintInterval;
            if (uintInterval == 0) {
                uintInterval = 10;  // Default interval if not specified
            }
            timespec timeout;
            OS::Clock::getTime(CLOCK_MONOTONIC, &timeout);
            timeout.tv_sec += (uintInterval / 1000);
            timeout.tv_nsec += (uintInterval % 1000) * 1000000;
            OS::Clock::adjustClockTime(&timeout);
            m_hDoNextJobEvent.wait(timeout);  // Wait for the next job execution
        }
    }

    /**
     * @brief Installs all job ticket resources.
     *
     * Installs all resources associated with the job.
     *
     * @return true if all resources were installed successfully, false otherwise.
     */
    bool installAllJobTicketResource() {
        bool bInstallOk = true;
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        map<string, CReference<IJobTicketResource>>::iterator it = m_JobTicketResourceMap.begin();
        while (bInstallOk && it != m_JobTicketResourceMap.end()) {
            bInstallOk &= it->second->onInstallResource();
            it++;
        }
        return bInstallOk;
    }

    /**
     * @brief Uninstalls all job ticket resources.
     *
     * Uninstalls all resources associated with the job.
     */
    void uninstallAllJobTicketResource() {
        TAutoMutex<OS::TRecursiveMutex> autoLock(&m_JobMutex);
        map<string, CReference<IJobTicketResource>>::iterator it = m_JobTicketResourceMap.begin();
        while (it != m_JobTicketResourceMap.end()) {
            it->second->onUninstallResource();
            it++;
        }
    }

protected:
    /**
     * @brief Static function for job execution.
     *
     * Static function that executes the job in a separate thread.
     *
     * @param lpParam Pointer to the CJobTicket object.
     * @return nullptr.
     */
    static LPVOID onJobFunction(void *lpParam) {
        CJobTicket *pJob = (CJobTicket *)lpParam;
        if (pJob != nullptr) {
            pJob->doJob();
        }
        return nullptr;
    }

protected:
    /**
     * @var T *m_pObject
     * @brief Pointer to the object containing the job function.
     */
    T *m_pObject;

    /**
     * @var MEMBLER_FUNCTION m_pFunc
     * @brief Pointer to the member function to be executed as the job.
     */
    MEMBLER_FUNCTION m_pFunc;

    /**
     * @var JOB_TYPE m_intJobType
     * @brief The type of the job (SINGLE_SHOT_JOB or PERIODIC_JOB).
     */
    JOB_TYPE m_intJobType;

    /**
     * @var unsigned int m_uintInterval
     * @brief The interval for periodic jobs in milliseconds.
     */
    unsigned int m_uintInterval;

    /**
     * @var bool m_bResourceEnable
     * @brief Indicates whether the resources are enabled.
     */
    bool m_bResourceEnable;

    /**
     * @var OS::TEvent m_hIdleEvent
     * @brief Event to indicate when resources are idle.
     */
    OS::TEvent m_hIdleEvent;

    /**
     * @var OS::TEvent m_hDoNextJobEvent
     * @brief Event to trigger the next job execution.
     */
    OS::TEvent m_hDoNextJobEvent;

    /**
     * @var OS::TRecursiveMutex m_JobMutex
     * @brief Mutex for synchronizing access to job data.
     */
    OS::TRecursiveMutex m_JobMutex;

    /**
     * @var OS_THREAD_HANDLE m_hJobThread
     * @brief Handle to the job thread.
     */
    OS_THREAD_HANDLE m_hJobThread;

    /**
     * @var map<string, CReference<IJobTicketResource>> m_JobTicketResourceMap
     * @brief Map of resources associated with the job.
     */
    map<string, CReference<IJobTicketResource>> m_JobTicketResourceMap;

    /**
     * @var bool m_bRequestStop
     * @brief Indicates whether a stop request has been made for the job.
     */
    bool m_bRequestStop;
};