#pragma once

#include "Base/IBase.h"

/**
 * @interface ILanguageTranslator
 * @brief Interface for language translation operations.
 *
 * This interface provides methods to access translator information and perform translation tasks.
 */
interface ILanguageTranslator : public IBase {
    /**
     * @brief Retrieves the parent translator in the translation hierarchy.
     *
     * @return Pointer to the parent ILanguageTranslator instance, or nullptr if none.
     */
    virtual ILanguageTranslator *getParentTranslator() = 0;

    /**
     * @brief Retrieves the name of this translator.
     *
     * @param strName Buffer to receive the translator name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getTranslatorName(char *strName, unsigned long &ulSize) = 0;

    /**
     * @brief Checks whether a translation item exists for the specified ID.
     *
     * @param strId Identifier of the item to check.
     * @return true if the item exists; false otherwise.
     */
    virtual bool existItem(const char *strId) = 0;

    /**
     * @brief Translates a string identified by its ID into the target language.
     *
     * @param strId Identifier of the string to translate.
     * @param strItem Buffer to receive the translated string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if translation succeeded; false otherwise.
     */
    virtual bool translate(const char *strId, char *strItem, unsigned long &ulSize) = 0;
};

ENABLE_RECOGNIZE_MACRO(ILanguageTranslator)

class CLanguageTranslatorHelper {
public:
    static string getTranslatorName(ILanguageTranslator *pLanguageTranslator) {
        string strName;
        if (pLanguageTranslator != nullptr) {
            unsigned long ulSize = 0;
            if (pLanguageTranslator->getTranslatorName(nullptr, ulSize) == true && ulSize > 0) {
                char *strNameA = new char[ulSize + 1];
                bzero(strNameA, (ulSize + 1) * sizeof(char));
                pLanguageTranslator->getTranslatorName(strNameA, ulSize);
                strName = strNameA;
                delete[] strNameA;
                strNameA = nullptr;
            }
        }
        return strName;
    }
    static string translate(ILanguageTranslator *pLanguageTranslator, const char *strId) {
        string strItem;
        if (pLanguageTranslator != nullptr) {
            unsigned long ulSize = 0;
            if (pLanguageTranslator->translate(strId, nullptr, ulSize) == true && ulSize > 0) {
                char *strItemA = new char[ulSize + 1];
                bzero(strItemA, (ulSize + 1) * sizeof(char));
                pLanguageTranslator->translate(strId, strItemA, ulSize);
                strItem = strItemA;
                delete[] strItemA;
                strItemA = nullptr;
            }
        }
        return strItem;
    }
};
