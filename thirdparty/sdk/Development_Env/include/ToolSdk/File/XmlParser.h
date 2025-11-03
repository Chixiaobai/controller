#ifndef __TOOLSDK_XML_PARSER_H__
#define __TOOLSDK_XML_PARSER_H__

#include "ToolSdkDef.h"
#include "File/File.h"
#include "libxml/parser.h"

/**
 * @brief Opaque structure representing an XML node.
 */
struct _XmlNode;

/**
 * @brief Pointer to an XML node.
 */
typedef _XmlNode *LPXmlNode;

/**
 * @brief Opaque structure representing an XML file handle.
 */
struct xml_file_handle;

/**
 * @brief Initializes the XML module.
 * @note This function is not thread-safe.
 */
TOOLSDK_API void xml_module_init();

/**
 * @brief Uninitializes the XML module.
 * @note This function is not thread-safe.
 */
TOOLSDK_API void xml_module_uninit();

/**
 * @brief Creates a new XML file handle.
 * @return A pointer to the created XML file handle.
 */
TOOLSDK_API xml_file_handle *xml_file_create();

/**
 * @brief Destroys an XML file handle.
 * @param handle Reference to the XML file handle to be destroyed.
 */
TOOLSDK_API void xml_file_destroy(xml_file_handle *&handle);

/**
 * @brief Removes all content from the XML file.
 * @param handle The XML file handle.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_remove_all(xml_file_handle *handle);

/**
 * @brief Closes the XML file.
 * @param handle The XML file handle.
 */
TOOLSDK_API void xml_file_close(xml_file_handle *handle);

/**
 * @brief Retrieves the root node of the XML file.
 * @param handle The XML file handle.
 * @return A pointer to the root node.
 */
TOOLSDK_API LPXmlNode xml_file_get_root(xml_file_handle *handle);

/**
 * @brief Retrieves the number of attributes of an XML node.
 * @param hNodeHandle The XML node handle.
 * @return The number of attributes.
 */
TOOLSDK_API UINT xml_file_get_attribute_count(LPXmlNode hNodeHandle);

/**
 * @brief Removes an attribute from an XML node by index.
 * @param index The index of the attribute to remove.
 * @param hNodeHandle The XML node handle.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_remove_attribute_by_index(int index, LPXmlNode hNodeHandle);

/**
 * @brief Opens an XML file.
 * @param handle The XML file handle.
 * @param strXmlPath The path to the XML file.
 * @param uFlag The flags for opening the file.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_open(xml_file_handle *handle, const char *strXmlPath, int uFlag);

/**
 * @brief Retrieves the number of child nodes with a specific name.
 * @param hParentNode The parent XML node handle.
 * @param strChildName The name of the child nodes to count.
 * @return The number of child nodes.
 */
TOOLSDK_API UINT xml_file_get_child_count(LPXmlNode hParentNode, const char *strChildName);

/**
 * @brief Retrieves a child node by index and name.
 * @param index The index of the child node.
 * @param hParentNode The parent XML node handle.
 * @param strChildName The name of the child node.
 * @return A pointer to the child node.
 */
TOOLSDK_API LPXmlNode xml_file_get_child_node(int index, LPXmlNode hParentNode, const char *strChildName);

/**
 * @brief Retrieves the name of an attribute by index.
 * @param index The index of the attribute.
 * @param hNodeHandle The XML node handle.
 * @param strName Buffer to store the attribute name.
 * @param dwLen Length of the buffer.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_get_attribute_name(int index, LPXmlNode hNodeHandle, char *strName, DWORD &dwLen);

/**
 * @brief Retrieves the value of an attribute by index.
 * @param index The index of the attribute.
 * @param hNodeHandle The XML node handle.
 * @param strValue Buffer to store the attribute value.
 * @param dwLen Length of the buffer.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_get_attribute_value_by_index(int index, LPXmlNode hNodeHandle, char *strValue, DWORD &dwLen);

/**
 * @brief Retrieves the value of an attribute by name.
 * @param strAttributeName The name of the attribute.
 * @param hNodeHandle The XML node handle.
 * @param strValue Buffer to store the attribute value.
 * @param dwLen Length of the buffer.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_get_attribute_value(const char *strAttributeName, LPXmlNode hNodeHandle, char *strValue,
                                              DWORD &dwLen);

/**
 * @brief Retrieves the value of an XML node.
 * @param hNodeHandle The XML node handle.
 * @param strValue Buffer to store the node value.
 * @param dwLen Length of the buffer.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_get_node_value(LPXmlNode hNodeHandle, char *strValue, DWORD &dwLen);

/**
 * @brief Retrieves the name of an XML node.
 * @param hNodeHandle The XML node handle.
 * @param strName Buffer to store the node name.
 * @param dwLen Length of the buffer.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_get_node_name(LPXmlNode hNodeHandle, char *strName, DWORD &dwLen);

/**
 * @brief Inserts a new node after a specified node.
 * @param handle The XML file handle.
 * @param index The index at which to insert the new node.
 * @param hParentNode The parent XML node handle.
 * @param strNodeName The name of the new node.
 * @param strNodeValue The value of the new node.
 * @param strChildName The name of the child node.
 * @return A pointer to the new node.
 */
TOOLSDK_API LPXmlNode xml_file_insert_after_node(xml_file_handle *handle, int index, LPXmlNode hParentNode,
                                                 const char *strNodeName, const char *strNodeValue,
                                                 const char *strChildName);

/**
 * @brief Inserts a new node before a specified node.
 * @param handle The XML file handle.
 * @param index The index at which to insert the new node.
 * @param hParentNode The parent XML node handle.
 * @param strNodeName The name of the new node.
 * @param strNodeValue The value of the new node.
 * @param strChildName The name of the child node.
 * @return A pointer to the new node.
 */
TOOLSDK_API LPXmlNode xml_file_insert_before_node(xml_file_handle *handle, int index, LPXmlNode hParentNode,
                                                  const char *strNodeName, const char *strNodeValue,
                                                  const char *strChildName);

/**
 * @brief Appends a new node to a parent node.
 * @param handle The XML file handle.
 * @param hParentNode The parent XML node handle.
 * @param strNodeName The name of the new node.
 * @param strNodeValue The value of the new node.
 * @param strChildName The name of the child node.
 * @return A pointer to the new node.
 */
TOOLSDK_API LPXmlNode xml_file_append_node(xml_file_handle *handle, LPXmlNode hParentNode, const char *strNodeName,
                                           const char *strNodeValue, const char *strChildName);

/**
 * @brief Inserts a new attribute after a specified attribute.
 * @param index The index at which to insert the new attribute.
 * @param hNodeHandle The XML node handle.
 * @param strAttributeName The name of the new attribute.
 * @param strAttributeValue The value of the new attribute.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_insert_after_attribute(int index, LPXmlNode hNodeHandle, const char *strAttributeName,
                                                 const char *strAttributeValue);

/**
 * @brief Inserts a new attribute before a specified attribute.
 * @param index The index at which to insert the new attribute.
 * @param hNodeHandle The XML node handle.
 * @param strAttributeName The name of the new attribute.
 * @param strAttributeValue The value of the new attribute.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_insert_before_attribute(int index, LPXmlNode hNodeHandle, const char *strAttributeName,
                                                  const char *strAttributeValue);

/**
 * @brief Appends a new attribute to a node.
 * @param hNodeHandle The XML node handle.
 * @param strAttributeName The name of the new attribute.
 * @param strAttributeValue The value of the new attribute.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_append_attribute(LPXmlNode hNodeHandle, const char *strAttributeName,
                                           const char *strAttributeValue);

/**
 * @brief Removes an attribute from a node by name.
 * @param strAttributeName The name of the attribute to remove.
 * @param hNodeHandle The XML node handle.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_remove_attribute(const char *strAttributeName, LPXmlNode hNodeHandle);

/**
 * @brief Removes a node from a parent node.
 * @param handle The XML file handle.
 * @param index The index of the node to remove.
 * @param hParentNode The parent XML node handle.
 * @param strChildName The name of the child node.
 * @return True if successful, false otherwise.
 */
TOOLSDK_API bool xml_file_remove_node(xml_file_handle *handle, int index, LPXmlNode hParentNode,
                                      const char *strChildName);

/**
 * @brief Removes all child nodes with a specific name from a parent node.
 * @param handle The XML file handle.
 * @param hParentNode The parent XML node handle.
 * @param strChildName The name of the child nodes to remove.
 */
TOOLSDK_API void xml_file_remove_all_child(xml_file_handle *handle, LPXmlNode hParentNode, const char *strChildName);

class CXmlParser {
public:
    /**
     * @brief Enumeration of flags for opening an XML file.
     */
    enum OpenFlags {
        modeRead = (int)0x01,       /**< Open for reading. */
        modeWrite = (int)0x02,      /**< Open for writing. */
        modeReadWrite = (int)0x04,  /**< Open for reading and writing. */
        modeCreate = (int)0x08,     /**< Create a new file if it does not exist. */
        modeNoTruncate = (int)0x10, /**< Do not truncate the file when opening for writing. */
        modeSync = (int)0x20        /**< Synchronous mode, write operations wait for data to be written to disk. */
    };

public:
    /**
     * @brief Constructor for CXmlParser.
     * Initializes the XML file handle.
     */
    CXmlParser(void) { m_pXmlFileHandle = xml_file_create(); }

    /**
     * @brief Destructor for CXmlParser.
     * Destroys the XML file handle.
     */
    virtual ~CXmlParser(void) { xml_file_destroy(m_pXmlFileHandle); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     * Prevents copying of CXmlParser objects.
     */
    CXmlParser(const CXmlParser &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     * Prevents assignment of CXmlParser objects.
     */
    CXmlParser &operator=(const CXmlParser &) = delete;
#endif

public:
    /**
     * @brief Removes all content from the XML file.
     * @return True if successful, false otherwise.
     */
    bool removaAll() { return xml_file_remove_all(m_pXmlFileHandle); }

    /**
     * @brief Closes the XML file.
     */
    void close() { xml_file_close(m_pXmlFileHandle); }

    /**
     * @brief Retrieves the root node of the XML file.
     * @return A pointer to the root node.
     */
    LPXmlNode getRoot() { return xml_file_get_root(m_pXmlFileHandle); }

    /**
     * @brief Retrieves the number of attributes of an XML node.
     * @param hNodeHandle The XML node handle.
     * @return The number of attributes.
     */
    UINT getAttributeCount(LPXmlNode hNodeHandle) { return xml_file_get_attribute_count(hNodeHandle); }

    /**
     * @brief Removes an attribute from an XML node by index.
     * @param index The index of the attribute to remove.
     * @param hNodeHandle The XML node handle.
     * @return True if successful, false otherwise.
     */
    bool removeAttribute(int index, LPXmlNode hNodeHandle) {
        return xml_file_remove_attribute_by_index(index, hNodeHandle);
    }

    /**
     * @brief Opens an XML file.
     * @param strXmlPath The path to the XML file.
     * @param uFlag The flags for opening the file.
     * @return True if successful, false otherwise.
     */
    bool open(const string &strXmlPath, int uFlag = CXmlParser::modeRead) {
        return xml_file_open(m_pXmlFileHandle, strXmlPath.c_str(), uFlag);
    }

    /**
     * @brief Retrieves the number of child nodes with a specific name.
     * @param hParentNode The parent XML node handle.
     * @param strChildName The name of the child nodes to count.
     * @return The number of child nodes.
     */
    UINT getChildCount(LPXmlNode hParentNode, const string &strChildName = "") {
        return xml_file_get_child_count(hParentNode, strChildName.c_str());
    }

    /**
     * @brief Retrieves a child node by index and name.
     * @param index The index of the child node.
     * @param hParentNode The parent XML node handle.
     * @param strChildName The name of the child node.
     * @return A pointer to the child node.
     */
    LPXmlNode getChildNode(int index, LPXmlNode hParentNode, const string &strChildName = "") {
        return xml_file_get_child_node(index, hParentNode, strChildName.c_str());
    }

    /**
     * @brief Retrieves the name of an attribute by index.
     * @param index The index of the attribute.
     * @param hNodeHandle The XML node handle.
     * @param strName Buffer to store the attribute name.
     * @return True if successful, false otherwise.
     */
    bool getAttributeName(int index, LPXmlNode hNodeHandle, string &strName) {
        DWORD dwLen = 0;
        strName.clear();
        if (xml_file_get_attribute_name(index, hNodeHandle, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            xml_file_get_attribute_name(index, hNodeHandle, pValue, dwLen);
            strName = pValue;
            delete[] pValue;
            pValue = nullptr;
            return true;
        } else
            return false;
    }

    /**
     * @brief Retrieves the value of an attribute by index.
     * @param index The index of the attribute.
     * @param hNodeHandle The XML node handle.
     * @param strValue Buffer to store the attribute value.
     * @return True if successful, false otherwise.
     */
    bool getAttributeValue(int index, LPXmlNode hNodeHandle, string &strValue) {
        DWORD dwLen = 0;
        strValue.clear();
        if (xml_file_get_attribute_value_by_index(index, hNodeHandle, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            xml_file_get_attribute_value_by_index(index, hNodeHandle, pValue, dwLen);
            strValue = pValue;
            delete[] pValue;
            pValue = nullptr;
            return true;
        } else
            return false;
    }

    /**
     * @brief Retrieves the value of an attribute by name.
     * @param strAttributeName The name of the attribute.
     * @param hNodeHandle The XML node handle.
     * @param strValue Buffer to store the attribute value.
     * @return True if successful, false otherwise.
     */
    bool getAttributeValue(const string &strAttributeName, LPXmlNode hNodeHandle, string &strValue) {
        DWORD dwLen = 0;
        strValue.clear();
        if (xml_file_get_attribute_value(strAttributeName.c_str(), hNodeHandle, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            xml_file_get_attribute_value(strAttributeName.c_str(), hNodeHandle, pValue, dwLen);
            strValue = pValue;
            delete[] pValue;
            pValue = nullptr;
            return true;
        } else
            return false;
    }

    /**
     * @brief Retrieves the value of an XML node.
     * @param hNodeHandle The XML node handle.
     * @return The value of the node.
     */
    string getNodeValue(LPXmlNode hNodeHandle) {
        string strValue;
        DWORD dwLen = 0;
        if (xml_file_get_node_value(hNodeHandle, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            xml_file_get_node_value(hNodeHandle, pValue, dwLen);
            strValue = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strValue;
    }

    /**
     * @brief Retrieves the name of an XML node.
     * @param hNodeHandle The XML node handle.
     * @return The name of the node.
     */
    string getNodeName(LPXmlNode hNodeHandle) {
        string strName;
        DWORD dwLen = 0;
        if (xml_file_get_node_name(hNodeHandle, nullptr, dwLen) == true && dwLen > 0) {
            char *pValue = new char[dwLen];
            xml_file_get_node_name(hNodeHandle, pValue, dwLen);
            strName = pValue;
            delete[] pValue;
            pValue = nullptr;
        }
        return strName;
    }

    /**
     * @brief Inserts a new node after a specified node.
     * @param index The index at which to insert the new node.
     * @param hParentNode The parent XML node handle.
     * @param strNodeName The name of the new node.
     * @param strNodeValue The value of the new node.
     * @param strChildName The name of the child node.
     * @return A pointer to the new node.
     */
    LPXmlNode insertAfterNode(int index, LPXmlNode hParentNode, const string &strNodeName,
                              const string &strNodeValue = "", const string &strChildName = "") {
        return xml_file_insert_after_node(m_pXmlFileHandle, index, hParentNode, strNodeName.c_str(),
                                          strNodeValue.c_str(), strChildName.c_str());
    }

    /**
     * @brief Inserts a new node before a specified node.
     * @param index The index at which to insert the new node.
     * @param hParentNode The parent XML node handle.
     * @param strNodeName The name of the new node.
     * @param strNodeValue The value of the new node.
     * @param strChildName The name of the child node.
     * @return A pointer to the new node.
     */
    LPXmlNode insertBeforeNode(int index, LPXmlNode hParentNode, const string &strNodeName,
                               const string &strNodeValue = "", const string &strChildName = "") {
        return xml_file_insert_before_node(m_pXmlFileHandle, index, hParentNode, strNodeName.c_str(),
                                           strNodeValue.c_str(), strChildName.c_str());
    }

    /**
     * @brief Appends a new node to a parent node.
     * @param hParentNode The parent XML node handle.
     * @param strNodeName The name of the new node.
     * @param strNodeValue The value of the new node.
     * @param strChildName The name of the child node.
     * @return A pointer to the new node.
     */
    LPXmlNode appendNode(LPXmlNode hParentNode, const string &strNodeName, const string &strNodeValue = "",
                         const string &strChildName = "") {
        return xml_file_append_node(m_pXmlFileHandle, hParentNode, strNodeName.c_str(), strNodeValue.c_str(),
                                    strChildName.c_str());
    }

    /**
     * @brief Inserts a new attribute after a specified attribute.
     * @param index The index at which to insert the new attribute.
     * @param hNodeHandle The XML node handle.
     * @param strAttributeName The name of the new attribute.
     * @param strAttributeValue The value of the new attribute.
     * @return True if successful, false otherwise.
     */
    bool insertAfterAttribute(int index, LPXmlNode hNodeHandle, const string &strAttributeName,
                              const string &strAttributeValue) {
        return xml_file_insert_after_attribute(index, hNodeHandle, strAttributeName.c_str(), strAttributeValue.c_str());
    }

    /**
     * @brief Inserts a new attribute before a specified attribute.
     * @param index The index at which to insert the new attribute.
     * @param hNodeHandle The XML node handle.
     * @param strAttributeName The name of the new attribute.
     * @param strAttributeValue The value of the new attribute.
     * @return True if successful, false otherwise.
     */
    bool insertBeforeAttribute(int index, LPXmlNode hNodeHandle, const string &strAttributeName,
                               const string &strAttributeValue) {
        return xml_file_insert_before_attribute(index, hNodeHandle, strAttributeName.c_str(),
                                                strAttributeValue.c_str());
    }

    /**
     * @brief Appends a new attribute to a node.
     * @param hNodeHandle The XML node handle.
     * @param strAttributeName The name of the new attribute.
     * @param strAttributeValue The value of the new attribute.
     * @return True if successful, false otherwise.
     */
    bool appendAttribute(LPXmlNode hNodeHandle, const string &strAttributeName, const string &strAttributeValue) {
        return xml_file_append_attribute(hNodeHandle, strAttributeName.c_str(), strAttributeValue.c_str());
    }

    /**
     * @brief Removes an attribute from a node by name.
     * @param strAttributeName The name of the attribute to remove.
     * @param hNodeHandle The XML node handle.
     * @return True if successful, false otherwise.
     */
    bool removeAttribute(const string &strAttributeName, LPXmlNode hNodeHandle) {
        return xml_file_remove_attribute(strAttributeName.c_str(), hNodeHandle);
    }

    /**
     * @brief Removes a node from a parent node.
     * @param index The index of the node to remove.
     * @param hParentNode The parent XML node handle.
     * @param strChildName The name of the child node.
     * @return True if successful, false otherwise.
     */
    bool removeNode(int index, LPXmlNode hParentNode, const string &strChildName = "") {
        return xml_file_remove_node(m_pXmlFileHandle, index, hParentNode, strChildName.c_str());
    }

    /**
     * @brief Removes all child nodes with a specific name from a parent node.
     * @param hParentNode The parent XML node handle.
     * @param strChildName The name of the child nodes to remove.
     */
    void removeAllChild(LPXmlNode hParentNode, const string &strChildName = "") {
        xml_file_remove_all_child(m_pXmlFileHandle, hParentNode, strChildName.c_str());
    }

protected:
    /**
     * @brief The XML file handle.
     */
    xml_file_handle *m_pXmlFileHandle;
};

#endif
