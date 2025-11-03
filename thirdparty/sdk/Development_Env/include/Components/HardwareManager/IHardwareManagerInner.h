#pragma once

#include "Base/IBase.h"

#define UNIQUE_HARDWAREMANAGER_NAME "HardwareManager"

interface INameEnum;
interface IHardwareCore;
/**
 * @interface IHardwareManagerInner
 * @brief Core interface for managing hardware components and factories.
 *
 * This interface provides comprehensive methods to configure workspace settings,
 * install/uninstall hardware factories, manage hardware instances, and query system state.
 */
interface IHardwareManagerInner : public IBase {
    /**
     * @brief Sets the working directory for hardware operations.
     *
     * @param strWorkspace Path to the workspace directory.
     */
    virtual void setWorkspace(const char *strWorkspace) = 0;

    /**
     * @brief Sets the directory containing hardware configuration files.
     *
     * @param strConfigurationDir Path to the configuration directory.
     */
    virtual void setConfigurationDir(const char *strConfigurationDir) = 0;

    /**
     * @brief Installs a hardware factory by name.
     *
     * @param strFactoryName Name of the factory to install.
     * @param strPostFix Optional postfix used in naming or path resolution.
     * @return true if installation succeeded; false otherwise.
     */
    virtual bool installHardwareFactoryByName(const char *strFactoryName, const char *strPostFix = nullptr) = 0;

    /**
     * @brief Installs a hardware factory using its library name.
     *
     * @param strFactoryName Name of the factory to install.
     * @param strFactoryLibrary Path to the factory's implementation library.
     * @param strPostFix Optional postfix used in naming or path resolution.
     * @return true if installation succeeded; false otherwise.
     */
    virtual bool installHardwareFactoryByLibrary(const char *strFactoryName, const char *strFactoryLibrary,
                                                 const char *strPostFix = nullptr) = 0;

    /**
     * @brief Installs a hardware factory with absolute paths.
     *
     * @param strFactoryName Name of the factory to install.
     * @param strAbsoluteFactoryLibrary Absolute path to the factory's implementation library.
     * @param strAbsoluteFactoryTemplate Absolute path to the factory's template/configuration file.
     * @return true if installation succeeded; false otherwise.
     */
    virtual bool installHardwareFactory(const char *strFactoryName, const char *strAbsoluteFactoryLibrary,
                                        const char *strAbsoluteFactoryTemplate) = 0;

    /**
     * @brief Uninstalls a previously installed hardware factory.
     *
     * @param strFactoryName Name of the factory to uninstall.
     * @return true if uninstallation succeeded; false otherwise.
     */
    virtual bool uninstallHardwareFactory(const char *strFactoryName) = 0;

    /**
     * @brief Uninstalls all currently installed hardware factories.
     *
     * @return true if all factories were uninstalled successfully; false otherwise.
     */
    virtual bool uninstallAllHardwareFactory() = 0;

    /**
     * @brief Loads all hardware configurations from persistent storage.
     *
     * @param strContentName Name of the content section to load.
     * @return true if loading succeeded; false otherwise.
     */
    virtual bool loadAllHardware(const char *strContentName) = 0;

    /**
     * @brief Saves all current hardware configurations to persistent storage.
     *
     * @return true if saving succeeded; false otherwise.
     */
    virtual bool saveAllHardware() = 0;

    /**
     * @brief Creates a hardware instance using the specified factory.
     *
     * @param strFactoryName Name of the factory to use.
     * @param strHardwareName Desired name for the new hardware instance.
     * @return true if creation succeeded; false otherwise.
     */
    virtual bool createHardware(const char *strFactoryName, const char *strHardwareName) = 0;

    /**
     * @brief Destroys a hardware instance.
     *
     * @param strFactoryName Name of the factory that created the hardware.
     * @param strHardwareName Name of the hardware instance to destroy.
     * @param bRemoveConfiguration true to remove associated configuration files; false otherwise.
     * @return true if destruction succeeded; false otherwise.
     */
    virtual bool destroyHardware(const char *strFactoryName, const char *strHardwareName,
                                 bool bRemoveConfiguration) = 0;

    /**
     * @brief Destroys all hardware instances created by a specific factory.
     *
     * @param strFactoryName Name of the factory whose instances should be destroyed.
     * @param bRemoveConfiguration true to remove associated configuration files; false otherwise.
     * @return true if all instances were destroyed successfully; false otherwise.
     */
    virtual bool destroyAllHardwareCreateByFactory(const char *strFactoryName, bool bRemoveConfiguration) = 0;

    /**
     * @brief Destroys all hardware instances managed by this manager.
     *
     * @param bRemoveConfiguration true to remove associated configuration files; false otherwise.
     * @return true if all instances were destroyed successfully; false otherwise.
     */
    virtual bool destroyAllHardware(bool bRemoveConfiguration) = 0;

    /**
     * @brief Retrieves the number of installed hardware factories.
     *
     * @return The count of hardware factories.
     */
    virtual unsigned long getHardwareFactoryCount() = 0;

    /**
     * @brief Retrieves the name of a hardware factory at the specified index.
     *
     * @param ulIndex Index of the factory name to retrieve (0-based).
     * @param strHardwareFactoryName Buffer to receive the factory name.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getHardwareFactoryName(unsigned long ulIndex, char *strHardwareFactoryName, unsigned long &ulSize) = 0;

    /**
     * @brief Retrieves the number of hardware instances managed by a specific factory.
     *
     * @param strFactoryName Name of the factory.
     * @return The count of hardware instances.
     */
    virtual unsigned long getHardwareCount(const char *strFactoryName) = 0;

    /**
     * @brief Retrieves the name of a hardware instance at the specified index under a factory.
     *
     * @param strFactoryName Name of the factory.
     * @param ulIndex Index of the hardware name to retrieve (0-based).
     * @param strHardwareName Buffer to receive the hardware name.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getHardwareName(const char *strFactoryName, unsigned long ulIndex, char *strHardwareName,
                                 unsigned long &ulSize) = 0;

    /**
     * @brief Enumerates all installed hardware factory names.
     *
     * @return Pointer to an INameEnum object containing factory names.
     */
    virtual INameEnum *getAllHardwareFactoryName() = 0;

    /**
     * @brief Enumerates all hardware instance names managed by a specific factory.
     *
     * @param strFactoryName Name of the factory.
     * @return Pointer to an INameEnum object containing hardware names.
     */
    virtual INameEnum *getAllHardwareName(const char *strFactoryName) = 0;

    /**
     * @brief Retrieves a hardware core instance by factory and hardware name.
     *
     * @param strFactoryName Name of the factory.
     * @param strHardwareName Name of the hardware instance.
     * @return Pointer to the IHardwareCore instance if found; nullptr otherwise.
     */
    virtual IHardwareCore *getHardware(const char *strFactoryName, const char *strHardwareName) = 0;

    /**
     * @brief Clears all internal caches related to hardware instances.
     */
    virtual void clearAllHardwareCache() = 0;
};

ENABLE_RECOGNIZE_MACRO(IHardwareManagerInner)

class CHardwareManagerHelper {
public:
    static bool parserAbsoluteHardwareName(const string &strAbsoluteHardwareName, string &strHardwareFactory,
                                           string &strHardwareName) {
        size_t first_split_pos = strAbsoluteHardwareName.find("::");
        if (first_split_pos == strAbsoluteHardwareName.npos)
            return false;
        else {
            strHardwareFactory = strAbsoluteHardwareName.substr(0, first_split_pos);
            strHardwareName = strAbsoluteHardwareName.substr(first_split_pos + 2,
                                                             strAbsoluteHardwareName.size() - first_split_pos - 2);
            return true;
        }
    }
    static bool getAbsoluteHardwareName(const string &strHardwareFactory, const string &strHardwareName,
                                        string &strAbsoluteHardwareName) {
        if (strHardwareFactory.empty() == true || strHardwareName.empty() == true) return false;
        strAbsoluteHardwareName = strHardwareFactory + "::" + strHardwareName;
        return true;
    }
};
