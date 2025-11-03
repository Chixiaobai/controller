#ifndef __TOOLSDK_ENDIAN_TYPE_H__
#define __TOOLSDK_ENDIAN_TYPE_H__

enum CPU_ENDIAN_TYPE {
    INVALID_ENDIAN_TYPE = -1,
    LITTLE_ENDIAN_TYPE = 0,
    BIG_ENDIAN_TYPE = 1,
};

class CEndianType {
public:
    static CPU_ENDIAN_TYPE getCPUEndianType() {
        union EndianTest {
            unsigned short usValue;
            struct {
                unsigned char byteValue[2];
            };
        };
        static CPU_ENDIAN_TYPE currentCPUEndian = INVALID_ENDIAN_TYPE;
        if (currentCPUEndian == INVALID_ENDIAN_TYPE) {
            EndianTest test;
            test.byteValue[0] = 0x12;
            test.byteValue[1] = 0x34;
            if (test.usValue == 0x1234)
                currentCPUEndian = BIG_ENDIAN_TYPE;
            else
                currentCPUEndian = LITTLE_ENDIAN_TYPE;
        }
        return currentCPUEndian;
    }
    template <class T>
    static T hostToNetwork(T &value) {
        if (getCPUEndianType() == LITTLE_ENDIAN_TYPE) {
            unsigned char *pt = (unsigned char *)&value;
            size_t len = sizeof(T);
            for (size_t i = 0; i < len / 2; i++) {
                unsigned char tmp = pt[i];
                pt[i] = pt[len - i - 1];
                pt[len - i - 1] = tmp;
            }
        }
        return value;
    }
    template <class T>
    static T networkToHost(T &value) {
        if (getCPUEndianType() == LITTLE_ENDIAN_TYPE) {
            unsigned char *pt = (unsigned char *)&value;
            size_t len = sizeof(T);
            for (size_t i = 0; i < len / 2; i++) {
                unsigned char tmp = pt[i];
                pt[i] = pt[len - i - 1];
                pt[len - i - 1] = tmp;
            }
        }
        return value;
    }
    template <class T>
    static T hostToLittleEndian(T &value) {
        if (getCPUEndianType() == BIG_ENDIAN_TYPE) {
            unsigned char *pt = (unsigned char *)&value;
            size_t len = sizeof(T);
            for (size_t i = 0; i < len / 2; i++) {
                unsigned char tmp = pt[i];
                pt[i] = pt[len - i - 1];
                pt[len - i - 1] = tmp;
            }
        }
        return value;
    }
    template <class T>
    static T littleEndianToHost(T &value) {
        if (getCPUEndianType() == BIG_ENDIAN_TYPE) {
            unsigned char *pt = (unsigned char *)&value;
            size_t len = sizeof(T);
            for (size_t i = 0; i < len / 2; i++) {
                unsigned char tmp = pt[i];
                pt[i] = pt[len - i - 1];
                pt[len - i - 1] = tmp;
            }
        }
        return value;
    }
};

#endif
