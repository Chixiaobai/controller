#pragma once

#include "Base/IBase.h"

typedef unsigned long TASK_FLAGS;

#define INVALID_TASK_ID 0

/**
 * @enum TASK_FLAG
 * @brief Flags used when creating tasks in the real-time hardware core.
 */
enum TASK_FLAG {
    CREATE_SUSPENDED_TASK = 0x01, /**< Create task in suspended state */
};

/**
 * @enum TASK_OPT
 * @brief Task options that can be set or retrieved for a task.
 */
enum TASK_OPT {
    SNDBLOCKSIZE = 0,     /**< Option to set/get send block size */
    RECVBLOCKSIZE,        /**< Option to set/get receive block size */
};

/**
 * @interface IRTHardwareCore
 * @brief Interface for managing real-time hardware tasks and communication.
 *
 * This interface provides methods to create, control, and configure tasks,
 * as well as perform data exchange operations with hardware.
 */
interface IRTHardwareCore : public IBase {
    /**
     * @brief Creates a new hardware task.
     *
     * @param ulInterval Interval for periodic task execution (in milliseconds).
     * @param uTaskFlags Flags controlling task creation behavior.
     * @return Unique task ID if successful; INVALID_TASK_ID otherwise.
     */
    virtual unsigned long createTask(unsigned long ulInterval, TASK_FLAGS uTaskFlags) = 0;

    /**
     * @brief Suspends the specified task.
     *
     * @param ulTaskId ID of the task to suspend.
     * @return true if the task was successfully suspended; false otherwise.
     */
    virtual bool suspendTask(unsigned long ulTaskId) = 0;

    /**
     * @brief Resumes the specified suspended task.
     *
     * @param ulTaskId ID of the task to resume.
     * @return true if the task was successfully resumed; false otherwise.
     */
    virtual bool resumeTask(unsigned long ulTaskId) = 0;

    /**
     * @brief Destroys the specified task.
     *
     * @param ulTaskId ID of the task to destroy.
     * @return true if the task was successfully destroyed; false otherwise.
     */
    virtual bool destroyTask(unsigned long ulTaskId) = 0;

    /**
     * @brief Destroys all active tasks.
     */
    virtual void destroyAllTask() = 0;

    /**
     * @brief Retrieves an option value for a specific task.
     *
     * @param ulTaskId ID of the task.
     * @param optname Option name to retrieve.
     * @param value Buffer to receive the option value.
     * @param valuesize Size of the buffer.
     * @return true if successful; false otherwise.
     */
    virtual bool getTaskOpt(unsigned long ulTaskId, unsigned long optname, void *value, long valuesize) = 0;

    /**
     * @brief Sets an option value for a specific task.
     *
     * @param ulTaskId ID of the task.
     * @param optname Option name to set.
     * @param value Pointer to the value to set.
     * @param valuesize Size of the value.
     * @return true if successful; false otherwise.
     */
    virtual bool setTaskOpt(unsigned long ulTaskId, unsigned long optname, const void *value, long valuesize) = 0;

    /**
     * @brief Performs a full data exchange operation with hardware using a specific task.
     *
     * @param ulTaskId ID of the associated task.
     * @param inputBuf Data to send.
     * @param lInLen Number of bytes to send.
     * @param outputBuf Buffer to receive the response.
     * @param lOutBufLen Maximum number of bytes to receive.
     * @return true if the exchange completed successfully; false otherwise.
     */
    virtual bool exchange(unsigned long ulTaskId, const unsigned char *inputBuf, long lInLen, unsigned char *outputBuf,
                          long lOutBufLen) = 0;

    /**
     * @brief Sends data to hardware using a specific task.
     *
     * @param ulTaskId ID of the associated task.
     * @param inputBuf Data to send.
     * @param lInLen Number of bytes to send.
     * @return true if the data was sent successfully; false otherwise.
     */
    virtual bool exchangeInput(unsigned long ulTaskId, const unsigned char *inputBuf, long lInLen) = 0;

    /**
     * @brief Receives data from hardware using a specific task.
     *
     * @param ulTaskId ID of the associated task.
     * @param outputBuf Buffer to receive the data.
     * @param lOutBufLen Maximum number of bytes to receive.
     * @return true if data was received successfully; false otherwise.
     */
    virtual bool exchangeOutput(unsigned long ulTaskId, unsigned char *outputBuf, long lOutBufLen) = 0;
};

ENABLE_RECOGNIZE_MACRO(IRTHardwareCore)
