#pragma once

#include "ModuleDef/IModuleVersion.h"
#include "IHardwareCore.h"

/**
 * @interface IHardwareFactory
 * @brief Factory interface for creating and managing hardware instances.
 *
 * This interface provides methods to create hardware objects, retrieve factory descriptions,
 * and obtain version information for the hardware implementation.
 */
interface IHardwareFactory : public IBase {
    /**
     * @brief Creates a hardware instance with the specified name and workspace.
     *
     * @param strHardwareName Name of the hardware to create.
     * @param strWorksapce Workspace path for the hardware instance.
     * @return Pointer to the created IHardwareCore instance if successful; nullptr otherwise.
     */
    virtual IHardwareCore *createHardware(const char *strHardwareName, const char *strWorksapce) = 0;

    /**
     * @brief Retrieves a description of the hardware factory.
     *
     * @param strFactoryDesc Buffer to receive the factory description string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getFactoryDesc(char *strFactoryDesc, unsigned long &ulSize) = 0;

    /**
     * @brief Retrieves the version information of the hardware implementation.
     *
     * @return Pointer to an IModuleVersion object containing version details.
     */
    virtual IModuleVersion *getHardwareVersion() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareFactory)

class CHardwareFactoryHelper {
public:
    static bool getFactoryDesc(IHardwareFactory *pHardwareFactory, string &strFactoryDesc) {
        if (pHardwareFactory != nullptr) {
            DWORD dwLen = 0;
            if (pHardwareFactory->getFactoryDesc(nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pHardwareFactory->getFactoryDesc(pValue, dwLen);
                strFactoryDesc = pValue;
                delete[] pValue;
                pValue = nullptr;
                return true;
            } else
                return false;
        } else
            return false;
    }
};
