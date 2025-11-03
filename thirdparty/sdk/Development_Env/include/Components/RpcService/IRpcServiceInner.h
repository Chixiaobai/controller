#pragma once

#define UNIQUE_RPCSERVICE_NAME "RpcService"

#include "IRpcServant.h"
#include "IRpcObjectCluster.h"

/**
 * @interface IRpcExceptionTranslator
 * @brief Interface for translating UUID-based exceptions into human-readable strings.
 */
interface IRpcExceptionTranslator : public IBase {
    /**
     * @brief Translates a UUID exception into a descriptive string.
     *
     * @param strExceptionContent Buffer to receive the translated exception message.
     * @param dwSize On input, specifies buffer size; on output, receives actual length.
     * @return true if translation succeeded; false otherwise.
     */
    virtual bool translateUuidException(char *strExceptionContent, DWORD &dwSize) = 0;
};

/**
 * @interface IRpcServiceInner
 * @brief Core interface for managing RPC services, including receivers, servants, and clusters.
 *
 * This interface provides methods to configure the RPC service environment, manage RPC receivers,
 * register and unregister servants, handle notifications, and control authentication status.
 */
interface IRpcServiceInner : public IBase {
    /**
     * @brief Sets the product UUID for the RPC service.
     *
     * @param strProductUuid Product UUID string.
     * @return true if the UUID was successfully set; false otherwise.
     */
    virtual bool setRpcProductUuid(const char *strProductUuid) = 0;

    /**
     * @brief Sets the workspace directory for RPC operations.
     *
     * @param strWorkspace Path to the workspace directory.
     */
    virtual void setWorkspace(const char *strWorkspace) = 0;

    /**
     * @brief Sets the configuration directory for storing RPC-related data.
     *
     * @param strConfigurationDir Path to the configuration directory.
     */
    virtual void setConfigurationDir(const char *strConfigurationDir) = 0;

    /**
     * @brief Loads all RPC receivers from the specified content file.
     *
     * @param strContentName Name of the content file to load.
     * @return true if loading succeeded; false otherwise.
     */
    virtual bool loadAllReceiver(const char *strContentName) = 0;

    /**
     * @brief Saves all registered interfaces to persistent storage.
     *
     * @return true if saving succeeded; false otherwise.
     */
    virtual bool saveAllInterface() = 0;

    /**
     * @brief Creates an RPC receiver for the specified hardware factory and name.
     *
     * @param strHardwareFactory Name of the hardware factory.
     * @param strHardwareName Name of the hardware instance.
     * @return true if the receiver was successfully created; false otherwise.
     */
    virtual bool createRpcReceiver(const char *strHardwareFactory, const char *strHardwareName) = 0;

    /**
     * @brief Destroys an existing RPC receiver for the specified hardware factory and name.
     *
     * @param strHardwareFactory Name of the hardware factory.
     * @param strHardwareName Name of the hardware instance.
     * @return true if the receiver was successfully destroyed; false otherwise.
     */
    virtual bool destroyRpcReceiver(const char *strHardwareFactory, const char *strHardwareName) = 0;

    /**
     * @brief Destroys all registered RPC receivers.
     *
     * @return true if all receivers were successfully destroyed; false otherwise.
     */
    virtual bool destroyAllRpcReceiver() = 0;

    /**
     * @brief Retrieves the number of currently registered RPC receivers.
     *
     * @return The count of RPC receivers.
     */
    virtual size_t getReceiverCount() = 0;

    /**
     * @brief Retrieves the name of the receiver at the specified index.
     *
     * @param uIndex Zero-based index of the receiver name to retrieve.
     * @param strReceiverName Buffer to receive the receiver name string.
     * @param dwSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getReceiverName(size_t uIndex, char *strReceiverName, DWORD &dwSize) = 0;

    /**
     * @brief Registers an RPC servant with the service.
     *
     * @param pRpcServant Pointer to the IRpcServant interface of the servant.
     * @return true if registration succeeded; false otherwise.
     */
    virtual bool registServant(IRpcServant * pRpcServant) = 0;

    /**
     * @brief Unregisters an RPC servant by its ID.
     *
     * @param uiServantId ID of the servant to unregister.
     * @return true if unregistration succeeded; false otherwise.
     */
    virtual bool unregistServant(unsigned short uiServantId) = 0;

    /**
     * @brief Finds the library path for a given servant name.
     *
     * @param strServantName Name of the servant.
     * @param strServantLibrary Buffer to receive the library path string.
     * @param dwSize On input, specifies buffer size; on output, receives actual length.
     * @param strPostFix Optional postfix to append to the servant name when searching.
     * @return true if the library path was found; false otherwise.
     */
    virtual bool findServantLibrary(const char *strServantName, char *strServantLibrary, DWORD &dwSize,
                                    const char *strPostFix = nullptr) = 0;

    /**
     * @brief Retrieves the object cluster associated with the specified servant ID.
     *
     * @param uiServantId ID of the servant.
     * @return Pointer to the IRpcObjectCluster instance if found; nullptr otherwise.
     */
    virtual IRpcObjectCluster *getObjectCluster(unsigned short uiServantId) = 0;

    /**
     * @brief Posts a notification to a specific interface of a servant.
     *
     * @param uiServantId ID of the target servant.
     * @param uintObjectId ID of the target object.
     * @param uiInterfaceId ID of the target interface.
     * @param uiNotifyCode Notification code to send.
     * @param pNotifyContent Pointer to the notification content data.
     * @param uiNotifySize Size of the notification content in bytes.
     * @return true if the notification was successfully posted; false otherwise.
     */
    virtual bool postNotify(unsigned short uiServantId, unsigned int uintObjectId, unsigned short uiInterfaceId,
                            unsigned short uiNotifyCode, const unsigned char *pNotifyContent,
                            unsigned short uiNotifySize) = 0;

    /**
     * @brief Sets the exception translator used for handling UUID-based exceptions.
     *
     * @param pExceptionTranslator Pointer to the IRpcExceptionTranslator implementation.
     */
    virtual void setExceptionTranslator(IRpcExceptionTranslator * pExceptionTranslator) = 0;

    /**
     * @brief Sets the authentication status for a specific hardware link.
     *
     * @param strHardwareFactory Name of the hardware factory.
     * @param strHardwareName Name of the hardware instance.
     * @param bAuthenticated true to mark the link as authenticated; false otherwise.
     * @return true if the operation succeeded; false otherwise.
     */
    virtual bool setLinkAuthenticated(const char *strHardwareFactory, const char *strHardwareName,
                                      bool bAuthenticated) = 0;

    /**
     * @brief Checks whether a specific hardware link is authenticated.
     *
     * @param strHardwareFactory Name of the hardware factory.
     * @param strHardwareName Name of the hardware instance.
     * @return true if the link is authenticated; false otherwise.
     */
    virtual bool isLinkAuthenticated(const char *strHardwareFactory, const char *strHardwareName) = 0;

    /**
     * @brief Checks whether any hardware link is currently authenticated.
     *
     * @return true if at least one authenticated link exists; false otherwise.
     */
    virtual bool hasAnyAuthenticatedLink() = 0;

    /**
     * @brief Retrieves the current RPC time in milliseconds.
     *
     * @return Current RPC time value.
     */
    virtual unsigned long long getRpcTime() = 0;
};
class CRpcExceptionTranslatorHelper {
public:
    static bool translateUuidException(IRpcExceptionTranslator *pTranslator, string &strExceptionContent) {
        if (pTranslator != nullptr) {
            DWORD dwLen = 0;
            if (pTranslator->translateUuidException(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pTranslator->translateUuidException(pValue, dwLen);
                strExceptionContent = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
};

class CRpcServiceHelper {
public:
    static bool getReceiverName(IRpcServiceInner *pRPCService, size_t uIndex, string &strReceiverName) {
        if (pRPCService != nullptr) {
            DWORD dwLen = 0;
            if (pRPCService->getReceiverName(uIndex, nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pRPCService->getReceiverName(uIndex, pValue, dwLen);
                strReceiverName = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
    static bool findServantLibrary(IRpcServiceInner *pRPCService, string strServantName, string &strServantLibrary,
                                   string strPostFix = "") {
        if (pRPCService != nullptr) {
            DWORD dwLen = 0;
            if (pRPCService->findServantLibrary(strServantName.c_str(), nullptr, dwLen, strPostFix.c_str()) == true &&
                dwLen > 0) {
                char *pValue = new char[dwLen];
                pRPCService->findServantLibrary(strServantName.c_str(), pValue, dwLen, strPostFix.c_str());
                strServantLibrary = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        }
        return false;
    }
};

ENABLE_RECOGNIZE_MACRO(IRpcServiceInner)
