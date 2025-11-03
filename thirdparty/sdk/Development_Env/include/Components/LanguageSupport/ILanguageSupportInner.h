#pragma once

#define UNIQUE_LANGUAGESUPPORT_NAME "LanguageSupport"

#include "ILanguageTranslator.h"

/**
 * @interface ILanguageSupportInner
 * @brief Core interface for language support and translation management.
 *
 * This interface provides methods to manage language settings, create translators,
 * and perform translation operations.
 */
interface ILanguageSupportInner : public IBase {
    /**
     * @brief Retrieves the directory path where language resources are stored.
     *
     * @param strLanguageDirectory Buffer to receive the directory path string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getLanagueDirectory(char *strLanguageDirectory, unsigned long &ulSize) = 0;

    /**
     * @brief Retrieves the currently set language name.
     *
     * @param strLanguage Buffer to receive the language name string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if successful; false otherwise.
     */
    virtual bool getLanague(char *strLanguage, unsigned long &ulSize) = 0;

    /**
     * @brief Sets the current language to be used for translation.
     *
     * @param strLanguage Name of the language to set.
     */
    virtual void setLanague(const char *strLanguage) = 0;

    /**
     * @brief Creates a new language translator instance.
     *
     * @param strTranslatorFolder Directory containing translator resources.
     * @param strTranslatorName Name of the translator to create.
     * @param strParentAbsoluteTranslatorName Optional parent translator name for inheritance.
     * @return true if the translator was successfully created; false otherwise.
     */
    virtual bool createLanguageTranslator(const char *strTranslatorFolder, const char *strTranslatorName,
                                          const char *strParentAbsoluteTranslatorName) = 0;

    /**
     * @brief Translates a string identified by its ID using the specified translator.
     *
     * @param pLanguageTranslator Pointer to the ILanguageTranslator instance.
     * @param strId Identifier of the string to translate.
     * @param strItem Buffer to receive the translated string.
     * @param ulSize On input, specifies buffer size; on output, receives actual length.
     * @return true if translation succeeded; false otherwise.
     */
    virtual bool translate(ILanguageTranslator * pLanguageTranslator, const char *strId, char *strItem,
                           unsigned long &ulSize) = 0;

    /**
     * @brief Finds a language translator by its absolute name.
     *
     * @param strAbsoluteTranslatorName Absolute name of the translator to find.
     * @return Pointer to the ILanguageTranslator if found; nullptr otherwise.
     */
    virtual ILanguageTranslator *findLanguageTranslator(const char *strAbsoluteTranslatorName) = 0;
};

ENABLE_RECOGNIZE_MACRO(ILanguageSupportInner)

class CLanguageSupportHelper {
public:
    static string getLanagueDirectory(ILanguageSupportInner *pLanguageSupport) {
        string strLanguageDirectory;
        if (pLanguageSupport != nullptr) {
            unsigned long ulSize = 0;
            if (pLanguageSupport->getLanagueDirectory(nullptr, ulSize) == true && ulSize > 0) {
                char *strLanguageDirectoryA = new char[ulSize + 1];
                bzero(strLanguageDirectoryA, (ulSize + 1) * sizeof(char));
                pLanguageSupport->getLanagueDirectory(strLanguageDirectoryA, ulSize);
                strLanguageDirectory = strLanguageDirectoryA;
                delete[] strLanguageDirectoryA;
                strLanguageDirectoryA = nullptr;
            }
        }
        return strLanguageDirectory;
    }
    static string GetLanague(ILanguageSupportInner *pLanguageSupport) {
        string strLanguage;
        if (pLanguageSupport != nullptr) {
            unsigned long ulSize = 0;
            if (pLanguageSupport->getLanague(nullptr, ulSize) == true && ulSize > 0) {
                char *strLanguageA = new char[ulSize + 1];
                bzero(strLanguageA, (ulSize + 1) * sizeof(char));
                pLanguageSupport->getLanague(strLanguageA, ulSize);
                strLanguage = strLanguageA;
                delete[] strLanguageA;
                strLanguageA = nullptr;
            }
        }
        return strLanguage;
    }
    static string Translate(ILanguageSupportInner *pLanguageSupport, ILanguageTranslator *pLanguageTranslator,
                            const char *strId) {
        string strItem;
        if (pLanguageSupport != nullptr) {
            unsigned long ulSize = 0;
            if (pLanguageSupport->translate(pLanguageTranslator, strId, nullptr, ulSize) == true && ulSize > 0) {
                char *strItemA = new char[ulSize + 1];
                bzero(strItemA, (ulSize + 1) * sizeof(char));
                pLanguageSupport->translate(pLanguageTranslator, strId, strItemA, ulSize);
                strItem = strItemA;
                delete[] strItemA;
                strItemA = nullptr;
            }
        }
        return strItem;
    }
};
