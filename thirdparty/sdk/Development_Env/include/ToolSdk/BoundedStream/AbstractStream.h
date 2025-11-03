#ifndef __TOOLSDK_ABSTRACT_STREAM_H__
#define __TOOLSDK_ABSTRACT_STREAM_H__

#include "ToolSdkDef.h"
#include "Environment/EndianType.h"

class CAbstractStream {
public:
    /**
     * @brief Exception class for stream operations.
     *
     * This class is used to represent exceptions that may occur during stream operations.
     */
    class CStreamException {
    public:
        /**
         * @brief Constructor for CStreamException.
         *
         * Initializes the exception with a specific error code.
         *
         * @param iExceptionCode The error code representing the exception.
         */
        CStreamException(int iExceptionCode) { m_iExceptionCode = iExceptionCode; }

    public:
        /// Operation succeeded.
        static const int OPERATE_SUCCEED = 0;
        /// Operation error ignored.
        static const int IGNORE_OPERATE_ERROR = -1;
        /// Inner critical error.
        static const int INNER_CRITICAL_ERROR = -2;
        /// Not enough space in the stream.
        static const int NOT_ENOUGH_SPACE = -3;
        /// Unexpected value encountered.
        static const int UNEXPECTED_VALUE_ERROR = -4;

    protected:
        /// The error code stored in the exception.
        int m_iExceptionCode;
    };

public:
    /**
     * @brief Reads an object from the stream into a buffer.
     *
     * This method is a pure virtual function that must be implemented by derived classes.
     * It reads a specified number of bytes from the stream into the provided buffer.
     *
     * @param pBuf Pointer to the buffer where the data will be stored.
     * @param uReadLen Number of bytes to read from the stream.
     */
    virtual void readObject(void *pBuf, size_t uReadLen) = 0;

    /**
     * @brief Writes an object from a buffer into the stream.
     *
     * This method is a pure virtual function that must be implemented by derived classes.
     * It writes a specified number of bytes from the provided buffer into the stream.
     *
     * @param pBuf Pointer to the buffer containing the data to be written.
     * @param uWriteLen Number of bytes to write to the stream.
     */
    virtual void writeObject(const void *pBuf, const size_t uWriteLen) = 0;

    /**
     * @brief Gets the length of the stream.
     *
     * This method is a pure virtual function that must be implemented by derived classes.
     * It returns the current length of the stream.
     *
     * @return The length of the stream.
     */
    virtual size_t getLength() = 0;

public:
    /**
     * @brief Writes a CHAR value to the stream.
     *
     * This operator writes a CHAR value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it writes the value to the stream.
     *
     * @param var The CHAR value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(CHAR var) {
        if (enoughWritable(sizeof(CHAR)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            writeObject((const void *)&var, sizeof(CHAR));
            return *this;
        }
    }

    /**
     * @brief Writes a BYTE value to the stream.
     *
     * This operator writes a BYTE value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it writes the value to the stream.
     *
     * @param var The BYTE value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(BYTE var) {
        if (enoughWritable(sizeof(BYTE)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            writeObject((const void *)&var, sizeof(BYTE));
            return *this;
        }
    }

    /**
     * @brief Writes a SHORT value to the stream.
     *
     * This operator writes a SHORT value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The SHORT value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(SHORT var) {
        if (enoughWritable(sizeof(SHORT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(SHORT));
            return *this;
        }
    }

    /**
     * @brief Writes a USHORT value to the stream.
     *
     * This operator writes a USHORT value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The USHORT value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(USHORT var) {
        if (enoughWritable(sizeof(USHORT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(USHORT));
            return *this;
        }
    }
    /**
     * @brief Writes an INT value to the stream.
     *
     * This operator writes an INT value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The INT value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(INT var) {
        if (enoughWritable(sizeof(INT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(INT));
            return *this;
        }
    }

    /**
     * @brief Writes a UINT value to the stream.
     *
     * This operator writes a UINT value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The UINT value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(UINT var) {
        if (enoughWritable(sizeof(UINT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(UINT));
            return *this;
        }
    }

    /**
     * @brief Writes a LONG value to the stream.
     *
     * This operator writes a LONG value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The LONG value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(LONG var) {
        if (enoughWritable(sizeof(LONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(LONG));
            return *this;
        }
    }

    /**
     * @brief Writes a ULONG value to the stream.
     *
     * This operator writes a ULONG value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The ULONG value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(ULONG var) {
        if (enoughWritable(sizeof(ULONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(ULONG));
            return *this;
        }
    }

    /**
     * @brief Writes a LONGLONG value to the stream.
     *
     * This operator writes a LONGLONG value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The LONGLONG value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(LONGLONG var) {
        if (enoughWritable(sizeof(LONGLONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(LONGLONG));
            return *this;
        }
    }

    /**
     * @brief Writes a ULONGLONG value to the stream.
     *
     * This operator writes a ULONGLONG value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The ULONGLONG value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(ULONGLONG var) {
        if (enoughWritable(sizeof(ULONGLONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(ULONGLONG));
            return *this;
        }
    }

    /**
     * @brief Writes a FLOAT value to the stream.
     *
     * This operator writes a FLOAT value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The FLOAT value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(FLOAT var) {
        if (enoughWritable(sizeof(FLOAT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(FLOAT));
            return *this;
        }
    }

    /**
     * @brief Writes a DOUBLE value to the stream.
     *
     * This operator writes a DOUBLE value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it converts the value to network byte order and writes it to the stream.
     *
     * @param var The DOUBLE value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(DOUBLE var) {
        if (enoughWritable(sizeof(DOUBLE)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            CEndianType::hostToNetwork(var);
            writeObject((const void *)&var, sizeof(DOUBLE));
            return *this;
        }
    }
    /**
     * @brief Writes a boolean value to the stream.
     *
     * This operator writes a boolean value to the stream. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     * Otherwise, it writes the value to the stream.
     *
     * @param var The boolean value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(bool var) {
        if (enoughWritable(sizeof(bool)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            writeObject((const void *)&var, sizeof(bool));
            return *this;
        }
    }

    /**
     * @brief Writes a string to the stream.
     *
     * This operator writes a string to the stream. It first converts the string from OEM encoding to UTF-8,
     * then writes the length of the string followed by the string itself. It checks if there is enough space
     * available in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     *
     * @param var The string value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(const string &var) {
        string new_var = OS::UTF8Helper::oemToUTF8(var);
        UINT uLen = (UINT)new_var.size();
        CEndianType::hostToNetwork(uLen);
        if (enoughWritable(sizeof(UINT) + new_var.size()) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            writeObject((const void *)&uLen, sizeof(UINT));
            writeObject((const void *)new_var.c_str(), new_var.size());
            return *this;
        }
    }

    /**
     * @brief Writes an ASCII string to the stream.
     *
     * This operator writes an ASCII string to the stream. It first checks if the string is in ASCII format.
     * If not, it throws a CStreamException with UNEXPECTED_VALUE_ERROR. Then, it writes the length of the
     * string followed by the string itself. It checks if there is enough space available in the stream.
     * If not, it throws a CStreamException with NOT_ENOUGH_SPACE.
     *
     * @param asciiVar The ASCII string value to be written.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator<<(const char *asciiVar) {
        if (OS::CLibrary::String::isAscii(asciiVar) == false) {
            throw new CStreamException(CStreamException::UNEXPECTED_VALUE_ERROR);
        } else {
            size_t len = strlen(asciiVar);
            UINT uLen = (UINT)len;
            CEndianType::hostToNetwork(uLen);
            if (enoughWritable(sizeof(UINT) + len) == false)
                throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
            else {
                writeObject((const void *)&uLen, sizeof(UINT));
                writeObject((const void *)asciiVar, len);
                return *this;
            }
        }
    }

    /**
     * @brief Reads a CHAR value from the stream.
     *
     * This operator reads a CHAR value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream.
     *
     * @param var Reference to the CHAR variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(CHAR &var) {
        if (enoughReadable(sizeof(CHAR)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(CHAR));
            return *this;
        }
    }

    /**
     * @brief Reads a BYTE value from the stream.
     *
     * This operator reads a BYTE value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream.
     *
     * @param var Reference to the BYTE variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(BYTE &var) {
        if (enoughReadable(sizeof(BYTE)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(BYTE));
            return *this;
        }
    }

    /**
     * @brief Reads a SHORT value from the stream.
     *
     * This operator reads a SHORT value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the SHORT variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(SHORT &var) {
        if (enoughReadable(sizeof(SHORT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(SHORT));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a USHORT value from the stream.
     *
     * This operator reads a USHORT value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the USHORT variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(USHORT &var) {
        if (enoughReadable(sizeof(USHORT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(USHORT));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads an INT value from the stream.
     *
     * This operator reads an INT value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the INT variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(INT &var) {
        if (enoughReadable(sizeof(INT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(INT));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a UINT value from the stream.
     *
     * This operator reads a UINT value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the UINT variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(UINT &var) {
        if (enoughReadable(sizeof(UINT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(UINT));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a LONG value from the stream.
     *
     * This operator reads a LONG value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the LONG variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(LONG &var) {
        if (enoughReadable(sizeof(LONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(LONG));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a ULONG value from the stream.
     *
     * This operator reads a ULONG value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the ULONG variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(ULONG &var) {
        if (enoughReadable(sizeof(ULONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(ULONG));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a LONGLONG value from the stream.
     *
     * This operator reads a LONGLONG value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the LONGLONG variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(LONGLONG &var) {
        if (enoughReadable(sizeof(LONGLONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(LONGLONG));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a ULONGLONG value from the stream.
     *
     * This operator reads a ULONGLONG value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the ULONGLONG variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(ULONGLONG &var) {
        if (enoughReadable(sizeof(ULONGLONG)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(ULONGLONG));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a FLOAT value from the stream.
     *
     * This operator reads a FLOAT value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the FLOAT variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(FLOAT &var) {
        if (enoughReadable(sizeof(FLOAT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(FLOAT));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a DOUBLE value from the stream.
     *
     * This operator reads a DOUBLE value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream and converts it from network byte order to host byte order.
     *
     * @param var Reference to the DOUBLE variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(DOUBLE &var) {
        if (enoughReadable(sizeof(DOUBLE)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(DOUBLE));
            CEndianType::networkToHost(var);
            return *this;
        }
    }

    /**
     * @brief Reads a boolean value from the stream.
     *
     * This operator reads a boolean value from the stream. It checks if there is enough data available
     * in the stream. If not, it throws a CStreamException with NOT_ENOUGH_SPACE. Otherwise, it reads
     * the value from the stream.
     *
     * @param var Reference to the boolean variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(bool &var) {
        if (enoughReadable(sizeof(bool)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            readObject(&var, sizeof(bool));
            return *this;
        }
    }

    /**
     * @brief Reads a string from the stream.
     *
     * This operator reads a string from the stream. It first reads the length of the string, then reads
     * the string itself. It checks if there is enough data available in the stream. If not, it throws a
     * CStreamException with NOT_ENOUGH_SPACE. The string is then converted from UTF-8 to OEM encoding.
     *
     * @param var Reference to the string variable where the read value will be stored.
     * @return Reference to the current CAbstractStream object.
     */
    CAbstractStream &operator>>(string &var) {
        if (enoughReadable(sizeof(UINT)) == false)
            throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
        else {
            UINT uLen = 0;
            readObject(&uLen, sizeof(UINT));
            CEndianType::networkToHost(uLen);
            if (enoughReadable(uLen) == false)
                throw new CStreamException(CStreamException::NOT_ENOUGH_SPACE);
            else {
                char *pBuffer = new char[uLen + 1];
                readObject(pBuffer, uLen);
                pBuffer[uLen] = '\0';
                var = OS::UTF8Helper::oemFromUTF8(pBuffer);
                delete[] pBuffer;
                pBuffer = nullptr;
                return *this;
            }
        }
    }

    /**
     * @brief Checks if there is enough readable space in the stream.
     *
     * This pure virtual function must be implemented by derived classes to determine if the stream
     * has sufficient space to read the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for reading.
     * @return true if there is enough space to read the required number of bytes, false otherwise.
     */
protected:
    virtual bool enoughReadable(size_t uRequireSpace) = 0;

    /**
     * @brief Checks if there is enough writable space in the stream.
     *
     * This pure virtual function must be implemented by derived classes to determine if the stream
     * has sufficient space to write the specified number of bytes.
     *
     * @param uRequireSpace The number of bytes required for writing.
     * @return true if there is enough space to write the required number of bytes, false otherwise.
     */
    virtual bool enoughWritable(size_t uRequireSpace) = 0;
};

#endif
