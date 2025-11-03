/**
 * @file ICallbackPointEx.h
 * @brief Defines the ICallbackPointEx interface for extended callback point management.
 *
 * This header file defines the `ICallbackPointEx` interface, which provides methods to manage callback sinks.
 * It extends the `IBase` interface and includes methods to get the count of sinks and retrieve specific sinks.
 */

#ifndef __TOOLSDK_I_CALLBACK_POINT_EX_H__
#define __TOOLSDK_I_CALLBACK_POINT_EX_H__

#include "Base/IBase.h"

/**
 * @brief Interface for extended callback point management.
 *
 * This interface provides methods to manage callback sinks, allowing objects to get the count of sinks
 * and retrieve specific sinks by index. It extends the `IBase` interface.
 */
interface ICallbackPointEx : public IBase {
    /**
     * @brief Gets the count of callback sinks.
     *
     * This method returns the number of callback sinks currently registered.
     *
     * @return The count of callback sinks.
     */
    virtual unsigned int getSinkCount() = 0;

    /**
     * @brief Gets a callback sink by index.
     *
     * This method retrieves a callback sink by its index.
     *
     * @param uintIndex The index of the callback sink to retrieve.
     * @return Pointer to the `IBase` object representing the callback sink if found, otherwise nullptr.
     */
    virtual IBase *getSink(unsigned int uintIndex) = 0;
};

/**
 * @brief Macro to enable recognition of the ICallbackPointEx interface.
 */
ENABLE_RECOGNIZE_MACRO(ICallbackPointEx)

#endif