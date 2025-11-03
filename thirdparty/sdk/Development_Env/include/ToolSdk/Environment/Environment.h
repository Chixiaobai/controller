/**
 * @file Environment.h
 * @brief Header file for environment-related functionalities.
 *
 * This file defines macros, functions, and a class for managing environment-specific paths and information.
 */

#ifndef __TOOLSDK_ENVIRONMENT_H__
#define __TOOLSDK_ENVIRONMENT_H__

#include "ToolSdkDef.h"

/**
 * @def APPLICATION_DIR_ENV
 * @brief Macro for the application directory environment variable.
 *
 * Represents the environment variable for the application directory.
 */
#define APPLICATION_DIR_ENV "${Application}"

/**
 * @def WORKSPACE_DIR_ENV
 * @brief Macro for the workspace directory environment variable.
 *
 * Represents the environment variable for the workspace directory.
 */
#define WORKSPACE_DIR_ENV "${Workspace}"

/**
 * @def APPLICATION_DIR_PATH_STRING
 * @brief Macro for the application directory path string.
 *
 * Defines the application directory path using the APPLICATION_DIR_ENV macro.
 */
#define APPLICATION_DIR_PATH_STRING APPLICATION_DIR_ENV "/"

/**
 * @def WORKSPACE_DIR_PATH_STRING
 * @brief Macro for the workspace directory path string.
 *
 * Defines the workspace directory path using the WORKSPACE_DIR_ENV macro.
 */
#define WORKSPACE_DIR_PATH_STRING WORKSPACE_DIR_ENV "/"

/**
 * @fn bool cenvironment_get_application_dir(char *strApplicationDir, DWORD &dwLen)
 * @brief Retrieves the application directory path.
 *
 * Retrieves the path of the application directory and stores it in the provided buffer.
 *
 * @param strApplicationDir Pointer to the buffer to store the application directory path.
 * @param dwLen Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool cenvironment_get_application_dir(char *strApplicationDir, DWORD &dwLen);

/**
 * @fn bool cenvironment_get_application_name(char *strApplicationName, DWORD &dwLen)
 * @brief Retrieves the application name.
 *
 * Retrieves the name of the application and stores it in the provided buffer.
 *
 * @param strApplicationName Pointer to the buffer to store the application name.
 * @param dwLen Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool cenvironment_get_application_name(char *strApplicationName, DWORD &dwLen);

/**
 * @fn bool cenvironment_encoder_path(const char *strPath, const char *strWorkspace, char *strEncoderPath, DWORD &dwLen)
 * @brief Encodes a path relative to the workspace.
 *
 * Encodes the provided path relative to the specified workspace and stores the result in the provided buffer.
 *
 * @param strPath Pointer to the buffer containing the original path.
 * @param strWorkspace Pointer to the buffer containing the workspace path.
 * @param strEncoderPath Pointer to the buffer to store the encoded path.
 * @param dwLen Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool cenvironment_encoder_path(const char *strPath, const char *strWorkspace, char *strEncoderPath,
                                           DWORD &dwLen);

/**
 * @fn bool cenvironment_decoder_path(const char *strPath, const char *strWorkspace, char *strDecoderPath, DWORD &dwLen)
 * @brief Decodes a path relative to the workspace.
 *
 * Decodes the provided path relative to the specified workspace and stores the result in the provided buffer.
 *
 * @param strPath Pointer to the buffer containing the encoded path.
 * @param strWorkspace Pointer to the buffer containing the workspace path.
 * @param strDecoderPath Pointer to the buffer to store the decoded path.
 * @param dwLen Reference to the length of the buffer.
 * @return true if the operation was successful, false otherwise.
 */
TOOLSDK_API bool cenvironment_decoder_path(const char *strPath, const char *strWorkspace, char *strDecoderPath,
                                           DWORD &dwLen);

/**
 * @class CEnvironment
 * @brief Class for managing environment-specific paths and information.
 *
 * This class provides static methods to retrieve application directory and name, and to encode/decode paths relative to
 * the workspace.
 */
class CEnvironment {
public:
    /**
     * @brief Maximum size for path buffers.
     *
     * Defines the maximum size for path buffers used in the class methods.
     */
    static const SSIZE_T MAX_SIZE = 1024;

public:
    /**
     * @brief Retrieves the application directory path.
     *
     * Retrieves the path of the application directory and returns it as a string.
     *
     * @return The application directory path as a string.
     */
    static string getApplicationDir() {
        string strSTLApplicationDir;
        DWORD dwLen = 0;
        if (cenvironment_get_application_dir(nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            cenvironment_get_application_dir(pValue, dwLen);
            strSTLApplicationDir = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strSTLApplicationDir;
    }

    /**
     * @brief Retrieves the application name.
     *
     * Retrieves the name of the application and returns it as a string.
     *
     * @return The application name as a string.
     */
    static string getApplicationName() {
        string strApplicationName;
        DWORD dwLen = 0;
        if (cenvironment_get_application_name(nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            cenvironment_get_application_name(pValue, dwLen);
            strApplicationName = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strApplicationName;
    }

    /**
     * @brief Encodes a path relative to the workspace.
     *
     * Encodes the provided path relative to the specified workspace and returns it as a string.
     *
     * @param strPath The original path to encode.
     * @param strWorkspace The workspace path.
     * @return The encoded path as a string.
     */
    static string encoderPath(string strPath, string strWorkspace) {
        string strEncoderPath;
        DWORD dwLen = 0;
        if (cenvironment_encoder_path(strPath.c_str(), strWorkspace.c_str(), nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            cenvironment_encoder_path(strPath.c_str(), strWorkspace.c_str(), pValue, dwLen);
            strEncoderPath = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strEncoderPath;
    }

    /**
     * @brief Decodes a path relative to the workspace.
     *
     * Decodes the provided path relative to the specified workspace and returns it as a string.
     *
     * @param strPath The encoded path to decode.
     * @param strWorkspace The workspace path.
     * @return The decoded path as a string.
     */
    static string decoderPath(string strPath, string strWorkspace) {
        string strDecoderPath;
        DWORD dwLen = 0;
        if (cenvironment_decoder_path(strPath.c_str(), strWorkspace.c_str(), nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            cenvironment_decoder_path(strPath.c_str(), strWorkspace.c_str(), pValue, dwLen);
            strDecoderPath = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strDecoderPath;
    }
};

#endif  // __TOOLSDK_ENVIRONMENT_H__