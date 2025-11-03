/**
 * @file IModuleVariants.h
 * @brief Header file for module variants management.
 *
 * This file defines an interface and a helper class for managing module variants.
 */

#ifndef __TOOLSDK_I_MODULE_VARIANTS_H__
#define __TOOLSDK_I_MODULE_VARIANTS_H__

#include "DataType/IVariant.h"

/**
 * @interface IModuleVariants
 * @brief Interface for managing module variants.
 *
 * This interface defines methods for loading, saving, and accessing module variants.
 */
interface IModuleVariants : public IBase {
    /**
     * @brief Loads module variants from a file.
     *
     * Loads module variants from the specified file.
     *
     * @param strFilePath The path to the file containing the module variants.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool loadFromFile(const char *strFilePath) = 0;

    /**
     * @brief Saves module variants to a file.
     *
     * Saves the current module variants to the specified file.
     *
     * @param strFilePath The path to the file where the module variants will be saved.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool saveToFile(const char *strFilePath) = 0;

    /**
     * @brief Retrieves the count of module variants.
     *
     * Retrieves the number of module variants.
     *
     * @return The count of module variants.
     */
    virtual size_t getModuleVariantCount() = 0;

    /**
     * @brief Retrieves the name of a module variant.
     *
     * Retrieves the name of the module variant at the specified index.
     *
     * @param uIndex The index of the module variant.
     * @param pName Pointer to the buffer to store the name.
     * @param dwLen Reference to the length of the buffer.
     * @return true if the operation was successful, false otherwise.
     */
    virtual bool getModuleVariantName(size_t uIndex, char *pName, DWORD &dwLen) = 0;

    /**
     * @brief Retrieves a variant by name.
     *
     * Retrieves the variant with the specified name.
     *
     * @param pName The name of the variant.
     * @return Pointer to the variant, or nullptr if the variant does not exist.
     */
    virtual IVariant *getVariant(const char *pName) = 0;

    /**
     * @brief Creates a clone of the module variants.
     *
     * Creates a new instance of the module variants with the same data.
     *
     * @return Pointer to the cloned module variants.
     */
    virtual IModuleVariants *clone() = 0;
};

/**
 * @class CModuleVariantsHelper
 * @brief Helper class for managing module variants.
 *
 * This class provides static methods to assist with managing module variants.
 */
class CModuleVariantsHelper {
public:
    /**
     * @brief Retrieves the name of a module variant.
     *
     * Retrieves the name of the module variant at the specified index and stores it in the provided string.
     *
     * @param pModuleVariants Pointer to the module variants.
     * @param uIndex The index of the module variant.
     * @param strVariantName Reference to the string to store the name.
     * @return true if the operation was successful, false otherwise.
     */
    static bool getModuleVariantName(IModuleVariants *pModuleVariants, size_t uIndex, string &strVariantName) {
        bool ret = false;
        if (pModuleVariants != nullptr) {
            DWORD dwLen = 0;
            if (pModuleVariants->getModuleVariantName(uIndex, nullptr, dwLen) == true && dwLen > 0) {
                char *pValue = new char[dwLen];
                pModuleVariants->getModuleVariantName(uIndex, pValue, dwLen);
                strVariantName = pValue;
                ret = true;
                delete[] pValue;
                pValue = nullptr;
            }
        }
        return ret;
    }
};

#endif  // __TOOLSDK_I_MODULE_VARIANTS_H__