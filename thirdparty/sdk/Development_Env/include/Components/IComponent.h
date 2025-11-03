#pragma once

#include "Base/IBase.h"

#define DECLARE_COMPONENT_STUB(classname)
#define IMPLEMENT_COMPONENT_STUB(classname)

/**
 * @interface IComponentProperties
 * @brief Interface for managing component properties.
 * @extends IBase
 */
interface IComponentProperties : public IBase {
    /**
     * @brief Gets the count of component properties.
     * @return The number of component properties.
     */
    virtual size_t getComponentPropertyCount() = 0;

    /**
     * @brief Gets the name of a component property by index.
     * @param index Index of the property.
     * @param pName Buffer to store the property name.
     * @param dwNameLen Pointer to the size of the buffer for the property name.
     * @return True if the property name is retrieved successfully, false otherwise.
     */
    virtual bool getComponentPropertyName(size_t index, char *pName, LPDWORD dwNameLen) = 0;

    /**
     * @brief Gets the value of a component property by name.
     * @param pName Name of the property.
     * @param pValue Buffer to store the property value.
     * @param dwValueLen Pointer to the size of the buffer for the property value.
     * @return True if the property value is retrieved successfully, false otherwise.
     */
    virtual bool getComponentPropertyValue(const char *pName, char *pValue, LPDWORD dwValueLen) = 0;

    /**
     * @brief Sets the value of a component property by name.
     * @param pName Name of the property.
     * @param pValue New value for the property.
     * @return True if the property value is set successfully, false otherwise.
     */
    virtual bool setComponentPropertyValue(const char *pName, const char *pValue) = 0;
};

/**
 * @interface IComponent
 * @brief Interface for managing components.
 * @extends IBase
 */
interface IComponent : public IBase {
    /**
     * @brief Gets the class name of the component.
     * @param pName Buffer to store the class name.
     * @param dwNameLen Pointer to the size of the buffer for the class name.
     * @return True if the class name is retrieved successfully, false otherwise.
     */
    virtual bool getComponentClassName(char *pName, LPDWORD dwNameLen) = 0;

    /**
     * @brief Gets the name of the component.
     * @param pName Buffer to store the component name.
     * @param dwNameLen Pointer to the size of the buffer for the component name.
     * @return True if the component name is retrieved successfully, false otherwise.
     */
    virtual bool getComponentName(char *pName, LPDWORD dwNameLen) = 0;

    /**
     * @brief Sets the name of the component.
     * @param pName New name for the component.
     */
    virtual void setComponentName(const char *pName) = 0;

    /**
     * @brief Gets the properties of the component.
     * @return Pointer to the IComponentProperties object.
     */
    virtual IComponentProperties *getComponentProperties() = 0;
};

/**
 * @interface IComponentSync
 * @brief Interface for synchronizing component access.
 * @extends IBase
 */
interface IComponentSync : public IBase {
    /**
     * @brief Locks the component for exclusive access.
     */
    virtual void lock() = 0;

    /**
     * @brief Unlocks the component to allow other accesses.
     */
    virtual void unlock() = 0;
};

ENABLE_RECOGNIZE_MACRO(IComponentProperties)
ENABLE_RECOGNIZE_MACRO(IComponent)
ENABLE_RECOGNIZE_MACRO(IComponentSync)

interface IResourceContext;
typedef void (*LOAD_COMPONENT_FUNC)();                                        //导出函数名称:OnComponentLoaded
typedef void (*UNLOAD_COMPONENT_FUNC)();                                      //导出函数名称:OnComponentUnloaded
typedef void (*UPDATE_COMPONENT_RESOURCE)(IComponent *, IResourceContext *);  //导出函数名称:OnUpdateComponentResource
typedef bool (*IS_SINGTONE_COMPONENT_FUNC)();                                 //导出函数名称:IsSingtoneComponent
typedef IComponent *(*CREATE_COMPONENT_FUNC)();                               //导出函数名称:CreateComponent
typedef bool (*GET_COMPONENT_CLASS_NAME)(char *pName, LPDWORD dwNameLen);  //导出函数名称:GetComponentClassName
