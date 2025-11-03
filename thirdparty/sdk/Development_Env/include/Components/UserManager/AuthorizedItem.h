#pragma once

#include "IUserManagerInner.h"
#include "Reference/Reference.h"
#include "StringAssistant/StringAssistant.h"

#define ACCESS_PROPERTY_NAME "access"
#define OWNER_PROPERTY_NAME "owner"

class CAuthorizedItem : public IAuthorizedItemStub {
public:
    class CAuthorizedItemAccessMode {
    public:
        static DWORD toNumerical(string strAccessMode) {
            DWORD dwAccessMode = 0;
            size_t iLen = strAccessMode.length();
            if (iLen > 0 && strAccessMode[0] == 'r') dwAccessMode |= USER_READ_ACCESS;
            if (iLen > 1 && strAccessMode[1] == 'w') dwAccessMode |= USER_WRITE_ACCESS;
            if (iLen > 3 && strAccessMode[3] == 'r') dwAccessMode |= GROUP_READ_ACCESS;
            if (iLen > 4 && strAccessMode[4] == 'w') dwAccessMode |= GROUP_WRITE_ACCESS;
            if (iLen > 6 && strAccessMode[6] == 'r') dwAccessMode |= OTHER_READ_ACCESS;
            if (iLen > 7 && strAccessMode[7] == 'w') dwAccessMode |= OTHER_WRITE_ACCESS;
            return dwAccessMode;
        }
        static string toString(DWORD dwAccessMode) {
            char strAccessMode[10] = {0};
            if (dwAccessMode & USER_READ_ACCESS)
                strAccessMode[0] = 'r';
            else
                strAccessMode[0] = '-';
            if (dwAccessMode & USER_WRITE_ACCESS)
                strAccessMode[1] = 'w';
            else
                strAccessMode[1] = '-';
            strAccessMode[2] = '-';
            if (dwAccessMode & GROUP_READ_ACCESS)
                strAccessMode[3] = 'r';
            else
                strAccessMode[3] = '-';
            if (dwAccessMode & GROUP_WRITE_ACCESS)
                strAccessMode[4] = 'w';
            else
                strAccessMode[4] = '-';
            strAccessMode[5] = '-';
            if (dwAccessMode & OTHER_READ_ACCESS)
                strAccessMode[6] = 'r';
            else
                strAccessMode[6] = '-';
            if (dwAccessMode & OTHER_WRITE_ACCESS)
                strAccessMode[7] = 'w';
            else
                strAccessMode[7] = '-';
            strAccessMode[8] = '-';
            return strAccessMode;
        }
    };
    class CAuthorizedItemOwner {
    public:
        static bool toOwner(string strUserGroup, string strUserName, string &strOwner) {
            if (strUserGroup.empty() == false && strUserName.empty() == false) {
                strOwner = strUserGroup + ":" + strUserName;
                return true;
            } else
                return false;
        }
        static bool toUser(string strOwner, string &strUserGroup, string &strUserName) {
            size_t split_pos = strOwner.find(':');
            if (split_pos != strOwner.npos) {
                strUserGroup = strOwner.substr(0, split_pos);
                strUserName = strOwner.substr(split_pos + 1);
                if (strUserGroup.empty() == false && strUserName.empty() == false)
                    return true;
                else
                    return false;
            } else
                return false;
        }
    };

public:
    CAuthorizedItem() { m_dwAccessMode = USER_ALL_ACCESS; }
    virtual ~CAuthorizedItem() {}
    virtual bool initAuthorizedItem(IUserAuthorized *pUserAuthorized) {
        if (pUserAuthorized != nullptr) {
            pUserAuthorized->addRef();
            m_UserAuthorized = pUserAuthorized;
            if (m_UserAuthorized.nil() == false) {
                getUserAuthorizedInfo(pUserAuthorized, m_strUserGroup, m_strUserName);
            }
            return true;
        } else
            return false;
    }
    virtual bool setUser(const char *strUserGroup, const char *strUserName, IUserAuthorized *pUserAuthorized)
        override  // set object owner,anybody can access this object if it do not have groupname or username
    {
        if (access(CONTROL_ACCESS, pUserAuthorized) == true) {
            m_strUserGroup = strUserGroup;
            m_strUserName = strUserName;
            return true;
        } else
            return false;
    }
    virtual bool getUserGroup(char *strUserGroup, unsigned long &ulSize) override {
        return CStringAssistant::getBaseStringFromSTLString(m_strUserGroup, strUserGroup, ulSize);
    }
    virtual bool getUserName(char *strUserName, unsigned long &ulSize) override {
        return CStringAssistant::getBaseStringFromSTLString(m_strUserName, strUserName, ulSize);
    }
    virtual DWORD getAccessMode() override { return m_dwAccessMode; }
    virtual bool setAccessMode(DWORD dwAccessMode, IUserAuthorized *pUserAuthorized) override {
        if (access(CONTROL_ACCESS, pUserAuthorized) == true) {
            m_dwAccessMode = dwAccessMode;
            return true;
        } else
            return false;
    }
    virtual bool access(ACCESS_TYPE type, IUserAuthorized *pUserAuthorized) override {
        if (m_strUserGroup.empty() == true ||
            m_strUserName.empty() == true)  // anybody can access this object if it do not have groupname or username
        {
            return true;
        } else {
            if (pUserAuthorized == nullptr) {
                pUserAuthorized = m_UserAuthorized;  // Use current user authorized
            }
            if (pUserAuthorized != nullptr) {
                string strUserGroup, strUserName;
                getUserAuthorizedInfo(pUserAuthorized, strUserGroup, strUserName);
                if (SYSTEM_GROUP_NAME == strUserGroup)  // system group can access any resource
                {
                    return true;
                } else if (ADMINISTRATOR_GROUP_NAME == strUserGroup) {
                    if (m_strUserGroup == SYSTEM_GROUP_NAME)
                        return false;  // administrator can not access system group
                    else
                        return true;
                } else {
                    if (type == READ_ACCESS) {
                        if (m_dwAccessMode & USER_READ_ACCESS) {
                            if (strUserName == m_strUserName)
                                return true;
                            else
                                return false;
                        } else if (m_dwAccessMode & GROUP_READ_ACCESS) {
                            if (strUserGroup == m_strUserGroup)
                                return true;
                            else
                                return false;
                        } else if (m_dwAccessMode & OTHER_READ_ACCESS) {
                            return true;
                        } else
                            return false;
                    } else if (type == WRITE_ACCESS) {
                        if (m_dwAccessMode & USER_WRITE_ACCESS) {
                            if (strUserName == m_strUserName)
                                return true;
                            else
                                return false;
                        } else if (m_dwAccessMode & GROUP_WRITE_ACCESS) {
                            if (strUserGroup == m_strUserGroup)
                                return true;
                            else
                                return false;
                        } else if (m_dwAccessMode & OTHER_WRITE_ACCESS) {
                            return true;
                        } else
                            return false;
                    } else if (type == CONTROL_ACCESS) {
                        if (strUserName == m_strUserName)  // user can change access-flag for it's own resource
                        {
                            return true;
                        } else if (m_dwAccessMode & GROUP_WRITE_ACCESS) {
                            if (strUserGroup == m_strUserGroup)
                                return true;
                            else
                                return false;
                        } else if (m_dwAccessMode & OTHER_WRITE_ACCESS) {
                            return true;
                        } else
                            return false;
                    } else
                        return false;
                }
            } else
                return false;
        }
    }
    static DWORD getAccessMode(IBase *pAccessObject) {
        CReference<IAuthorizedItem> accessItem = AS_INTERFACE(pAccessObject, IAuthorizedItem);
        if (accessItem.nil() == false)
            return accessItem->getAccessMode();
        else
            return 0;
    }
    static bool access(IBase *pAccessObject, ACCESS_TYPE type, IUserAuthorized *pUserAuthorized) {
        CReference<IAuthorizedItem> accessItem = AS_INTERFACE(pAccessObject, IAuthorizedItem);
        if (accessItem.nil() == false)
            return accessItem->access(type, pUserAuthorized);
        else
            return false;
    }
    static bool setAccessMode(IBase *pAccessObject, DWORD dwAccessMode, IUserAuthorized *pUserAuthorized) {
        CReference<IAuthorizedItem> accessItem = AS_INTERFACE(pAccessObject, IAuthorizedItem);
        if (accessItem.nil() == false)
            return accessItem->setAccessMode(dwAccessMode, pUserAuthorized);
        else
            return false;
    }
    static bool setUser(IBase *pAccessObject, const char *strUserGroup, const char *strUserName,
                        IUserAuthorized *pUserAuthorized) {
        CReference<IAuthorizedItem> accessItem = AS_INTERFACE(pAccessObject, IAuthorizedItem);
        if (accessItem.nil() == false)
            return accessItem->setUser(strUserGroup, strUserName, pUserAuthorized);
        else
            return false;
    }
    static bool getUserAuthorizedInfo(IUserAuthorized *pUserAuthorized, string &strUserGroup, string &strUserName) {
        if (pUserAuthorized == nullptr)
            return false;
        else {
            unsigned long ulSize = 0;
            if (pUserAuthorized->getUserGroup(nullptr, ulSize) == true) {
                char *strUserGroupA = nullptr;
                strUserGroupA = new char[ulSize + 1];
                bzero(strUserGroupA, ulSize + 1);
                pUserAuthorized->getUserGroup(strUserGroupA, ulSize);
                strUserGroup = strUserGroupA;
                delete[] strUserGroupA;
                strUserGroupA = nullptr;
            } else
                return false;
            ulSize = 0;
            if (pUserAuthorized->getUserName(nullptr, ulSize) == true) {
                char *strUserNameA = nullptr;
                strUserNameA = new char[ulSize + 1];
                bzero(strUserNameA, ulSize + 1);
                pUserAuthorized->getUserName(strUserNameA, ulSize);
                strUserName = strUserNameA;
                delete[] strUserNameA;
                strUserNameA = nullptr;
            } else
                return false;
            return true;
        }
    }
    static bool getAuthorizedItemInfo(IBase *pAccessObject, string &strUserGroup, string &strUserName) {
        CReference<IAuthorizedItem> accessItem = AS_INTERFACE(pAccessObject, IAuthorizedItem);
        if (accessItem.nil() == false) {
            bool ret = true;
            unsigned long ulSize = 0;
            if (accessItem->getUserGroup(nullptr, ulSize) == true) {
                char *strUserGroupA = nullptr;
                strUserGroupA = new char[ulSize + 1];
                bzero(strUserGroupA, ulSize + 1);
                accessItem->getUserGroup(strUserGroupA, ulSize);
                strUserGroup = strUserGroupA;
                delete[] strUserGroupA;
                strUserGroupA = nullptr;
            } else
                ret = false;
            ulSize = 0;
            if (accessItem->getUserName(nullptr, ulSize) == true) {
                char *strUserNameA = nullptr;
                strUserNameA = new char[ulSize + 1];
                bzero(strUserNameA, ulSize + 1);
                accessItem->getUserName(strUserNameA, ulSize);
                strUserName = strUserNameA;
                delete[] strUserNameA;
                strUserNameA = nullptr;
            } else
                ret = false;
            return ret;
        } else
            return false;
    }

protected:
    CReference<IUserAuthorized> m_UserAuthorized;
    string m_strUserGroup;
    string m_strUserName;
    DWORD m_dwAccessMode;
};
