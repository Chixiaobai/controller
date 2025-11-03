#ifndef __BASICSDK_SOCKET_H__
#define __BASICSDK_SOCKET_H__

#include "OS/Socket/OS_Socket.h"
#include "ISocket.h"

class CSockAddr : public sockaddr_in {
public:
    /**
     * @brief Default constructor for CSockAddr.
     *
     * Initializes a CSockAddr object with default values: family set to AF_INET, port and address set to 0.
     */
    CSockAddr() {
        bzero(this, sizeof(sockaddr));
        sin_family = AF_INET;
        sin_port = 0;
        sin_addr.s_addr = 0;
    }

    /**
     * @brief Constructor for CSockAddr from a sockaddr.
     *
     * Initializes a CSockAddr object from a given sockaddr structure.
     *
     * @param sa The sockaddr structure to initialize from.
     */
    CSockAddr(const sockaddr &sa) { memcpy(this, &sa, sizeof(sockaddr)); }

    /**
     * @brief Constructor for CSockAddr from a sockaddr_in.
     *
     * Initializes a CSockAddr object from a given sockaddr_in structure.
     *
     * @param sin The sockaddr_in structure to initialize from.
     */
    CSockAddr(const sockaddr_in &sin) { memcpy(this, &sin, sizeof(sockaddr_in)); }

    /**
     * @brief Constructor for CSockAddr from an IP address and port.
     *
     * Initializes a CSockAddr object with the specified IP address and port.
     *
     * @param ulAddr The IP address in network byte order.
     * @param ushPort The port number in host byte order (default is 0).
     */
    CSockAddr(const ULONG ulAddr, const USHORT ushPort = 0) {
        bzero(this, sizeof(sockaddr));
        sin_family = AF_INET;
        sin_port = htons(ushPort);
        sin_addr.s_addr = htonl((unsigned int)ulAddr);
    }

    /**
     * @brief Constructor for CSockAddr from a string representation of an IP address and port.
     *
     * Initializes a CSockAddr object with the specified string representation of an IP address and port.
     *
     * @param str The string representation of the IP address.
     * @param ushPort The port number in host byte order (default is 0).
     */
    CSockAddr(const string &str, const USHORT ushPort = 0) {
        bzero(this, sizeof(sockaddr));
        sin_family = AF_INET;
        sin_port = htons(ushPort);
        ULONG s_address;
        s_address = inet_addr(str.c_str());
        if (s_address != INADDR_NONE) {
            sin_addr.s_addr = (unsigned int)s_address;
        } else
            sin_addr.s_addr = 0;
    }

    /**
     * @brief Returns the dotted decimal representation of the IP address.
     *
     * Converts the IP address to its dotted decimal string representation.
     *
     * @return The dotted decimal string representation of the IP address.
     */
    string dottedDecimal() { return inet_ntoa(sin_addr); }

    /**
     * @brief Returns the port number in host byte order.
     *
     * Retrieves the port number stored in the CSockAddr object, converting it from network byte order to host byte
     * order.
     *
     * @return The port number in host byte order.
     */
    USHORT Port() const { return ntohs(sin_port); }

    /**
     * @brief Returns the IP address in host byte order.
     *
     * Retrieves the IP address stored in the CSockAddr object, converting it from network byte order to host byte
     * order.
     *
     * @return The IP address in host byte order.
     */
    ULONG iPAddr() const { return ntohl(sin_addr.s_addr); }

    /**
     * @brief Assignment operator from a sockaddr.
     *
     * Assigns the values from a given sockaddr structure to the CSockAddr object.
     *
     * @param sa The sockaddr structure to assign from.
     * @return Reference to the assigned CSockAddr object.
     */
    const CSockAddr &operator=(const sockaddr &sa) {
        memcpy(this, &sa, sizeof(sockaddr));
        return *this;
    }

    /**
     * @brief Assignment operator from a sockaddr_in.
     *
     * Assigns the values from a given sockaddr_in structure to the CSockAddr object.
     *
     * @param sin The sockaddr_in structure to assign from.
     * @return Reference to the assigned CSockAddr object.
     */
    const CSockAddr &operator=(const sockaddr_in &sin) {
        memcpy(this, &sin, sizeof(sockaddr_in));
        return *this;
    }

    /**
     * @brief Conversion operator to sockaddr.
     *
     * Converts the CSockAddr object to a sockaddr structure.
     *
     * @return The sockaddr structure.
     */
    operator sockaddr() { return *((sockaddr *)this); }

    /**
     * @brief Conversion operator to sockaddr*.
     *
     * Converts the CSockAddr object to a pointer to a sockaddr structure.
     *
     * @return Pointer to the sockaddr structure.
     */
    operator sockaddr *() { return (sockaddr *)this; }

    /**
     * @brief Conversion operator to sockaddr_in*.
     *
     * Converts the CSockAddr object to a pointer to a sockaddr_in structure.
     *
     * @return Pointer to the sockaddr_in structure.
     */
    operator sockaddr_in *() { return (sockaddr_in *)this; }
};

DECLARE_STUB_INTERFACE(ISocketStub, ISocket)

class CSocket : public ISocketStub {
public:
    /**
     * @brief Constructor for CSocket.
     *
     * Initializes a new CSocket object, initializes the socket library, and sets the socket file descriptor
     * to an invalid value. The real-time mode is set to USE_STD.
     */
    CSocket() {
        OS::Socket::initLibrary();
        m_iSocketFd = (OS_SOCKET)INVALID_FD_VALUE;
        m_RtMode = USE_STD;
    }

    /**
     * @brief Destructor for CSocket.
     *
     * Cleans up the socket library when the CSocket object is destroyed.
     */
    virtual ~CSocket() { OS::Socket::clearLibrary(); }

#ifdef STD_CPLUSPLUS_11
    /**
     * @brief Deleted copy constructor.
     *
     * Prevents copying of CSocket objects to ensure proper management of resources.
     */
    CSocket(const CSocket &) = delete;

public:
    /**
     * @brief Deleted assignment operator.
     *
     * Prevents assignment of CSocket objects to ensure proper management of resources.
     */
    CSocket &operator=(const CSocket &) = delete;
#endif

public:
    /**
     * @brief Returns the socket file descriptor.
     *
     * Retrieves the file descriptor of the socket.
     *
     * @return The socket file descriptor.
     */
    inline OS_SOCKET getSockFd() { return m_iSocketFd; }

    /**
     * @brief Checks if the socket is ready to read data.
     *
     * Checks if the socket is ready to read data within the specified timeout period.
     *
     * @param dwMilliSeconds The timeout period in milliseconds.
     * @param intErrorCode Pointer to an integer where the error code will be stored if an error occurs.
     * @return true if the socket is ready to read, false otherwise.
     */
    bool readyToRead(DWORD dwMilliSeconds, int *intErrorCode = nullptr) {
        return OS::Socket::readable(m_iSocketFd, dwMilliSeconds, intErrorCode, m_RtMode);
    }

    /**
     * @brief Checks if the socket is ready to write data.
     *
     * Checks if the socket is ready to write data within the specified timeout period.
     *
     * @param dwMilliSeconds The timeout period in milliseconds.
     * @param intErrorCode Pointer to an integer where the error code will be stored if an error occurs.
     * @return true if the socket is ready to write, false otherwise.
     */
    bool readyToWrite(DWORD dwMilliSeconds, int *intErrorCode = nullptr) {
        return OS::Socket::writable(m_iSocketFd, dwMilliSeconds, intErrorCode, m_RtMode);
    }

    /**
     * @brief Checks if the socket is ready to read data in microseconds.
     *
     * Checks if the socket is ready to read data within the specified timeout period in microseconds.
     *
     * @param dwMicroSecond The timeout period in microseconds.
     * @param intErrorCode Pointer to an integer where the error code will be stored if an error occurs.
     * @return true if the socket is ready to read, false otherwise.
     */
    bool readyToReadInMicroSecond(DWORD dwMicroSecond, int *intErrorCode = nullptr) {
        return OS::Socket::precision_readable(m_iSocketFd, dwMicroSecond, intErrorCode, m_RtMode);
    }

    /**
     * @brief Checks if the socket is ready to write data in microseconds.
     *
     * Checks if the socket is ready to write data within the specified timeout period in microseconds.
     *
     * @param dwMicroSecond The timeout period in microseconds.
     * @param intErrorCode Pointer to an integer where the error code will be stored if an error occurs.
     * @return true if the socket is ready to write, false otherwise.
     */
    bool readyToWriteInMicroSecond(DWORD dwMicroSecond, int *intErrorCode = nullptr) {
        return OS::Socket::precision_writable(m_iSocketFd, dwMicroSecond, intErrorCode, m_RtMode);
    }

    /**
     * @brief Checks if the link is broken.
     *
     * Checks if the link associated with the socket is broken.
     *
     * @return true if the link is broken, false otherwise.
     */
    bool isLinkBroken() { return OS::Socket::isLinkBroken(m_iSocketFd); }

public:
    /**
     * @brief Attaches an existing socket file descriptor to the socket object.
     *
     * Attaches an existing socket file descriptor to the socket object and sets the real-time mode.
     *
     * @param iSocketFd The socket file descriptor to attach.
     * @param mode The real-time mode for the socket (default is USE_DEFAULT).
     * @return true if the socket was successfully attached, false otherwise.
     */
    virtual bool attach(OS_SOCKET iSocketFd, RT_MODE mode = USE_DEFAULT) override {
        int m_RECVBufferSize = 8192, m_SNDBufferSize = 8192;
        if (m_iSocketFd != (OS_SOCKET)INVALID_FD_VALUE || iSocketFd == (OS_SOCKET)INVALID_FD_VALUE) return false;
        m_iSocketFd = iSocketFd;
        if (mode == USE_DEFAULT) {
            m_RtMode = get_default_rt_mode();
        } else {
            m_RtMode = mode;
        }
        int TimeOut = 500, iReuseAddress = 1;
        OS::Socket::setSendTimeout(m_iSocketFd, TimeOut, m_RtMode);
        OS::Socket::setRecvTimeout(m_iSocketFd, TimeOut, m_RtMode);
        OS::Socket::setSockOpt(m_iSocketFd, SOL_SOCKET, SO_RCVBUF, (const char *)&m_RECVBufferSize, sizeof(int),
                               m_RtMode);
        OS::Socket::setSockOpt(m_iSocketFd, SOL_SOCKET, SO_SNDBUF, (const char *)&m_SNDBufferSize, sizeof(int),
                               m_RtMode);
        OS::Socket::setSockOpt(m_iSocketFd, SOL_SOCKET, SO_REUSEADDR, (const char *)&iReuseAddress, sizeof(int),
                               m_RtMode);
        OS::Socket::disableUDPUnreachedReport(iSocketFd);
        unsigned long ul = 1;
        OS::Socket::ioctl(m_iSocketFd, FIONBIO, (unsigned long *)&ul, m_RtMode);
        onSocketAttach();
        return true;
    }

    /**
     * @brief Detaches the socket file descriptor from the socket object.
     *
     * Detaches the socket file descriptor from the socket object and returns it.
     *
     * @return The detached socket file descriptor.
     */
    virtual OS_SOCKET dettach() override {
        onSocketDetach();
        OS_SOCKET detatchsocket = m_iSocketFd;
        m_iSocketFd = (OS_SOCKET)INVALID_FD_VALUE;
        return detatchsocket;
    }

    /**
     * @brief Initializes a new socket.
     *
     * Initializes a new socket with the specified type, local address, and local port.
     *
     * @param nSocketType The type of the socket (e.g., SOCK_STREAM, SOCK_DGRAM).
     * @param strLocalAddress The local address to bind the socket to.
     * @param uLocalPort The local port to bind the socket to.
     * @param mode The real-time mode for the socket (default is USE_DEFAULT).
     * @return true if the socket was successfully initialized, false otherwise.
     */
    virtual bool initSocket(OS_SOCKET_TYPE nSocketType, const string &strLocalAddress, USHORT uLocalPort,
                            RT_MODE mode = USE_DEFAULT) override {
        if (mode == USE_DEFAULT) {
            m_RtMode = get_default_rt_mode();
        } else {
            m_RtMode = mode;
        }
        OS_SOCKET iSocketFd = 0;
        iSocketFd = OS::Socket::socket(AF_INET, nSocketType, 0, m_RtMode);
        if (iSocketFd == (OS_SOCKET)INVALID_FD_VALUE) return false;
        if (attach(iSocketFd, mode) == false) return false;
        return bind(strLocalAddress, uLocalPort);
    }

    /**
     * @brief Closes the socket.
     *
     * Closes the socket and releases any associated resources.
     *
     * @return true if the socket was successfully closed, false otherwise.
     */
    virtual bool close() override {
        OS_SOCKET iSocketFd = dettach();
        OS::Socket::close(iSocketFd, m_RtMode);
        return true;
    }

    /**
     * @brief Binds the socket to a local address and port.
     *
     * Binds the socket to the specified local address and port.
     *
     * @param strLocalAddress The local address to bind the socket to.
     * @param uLocalPort The local port to bind the socket to.
     * @return true if the socket was successfully bound, false otherwise.
     */
    virtual bool bind(const string &strLocalAddress, USHORT uLocalPort) override {
        CSockAddr m_SockAddr(strLocalAddress, uLocalPort);
        int ret = OS::Socket::bind(m_iSocketFd, (const sockaddr *)m_SockAddr, sizeof(CSockAddr), m_RtMode);
        if (ret == SOCKET_ERROR) {
            printf("Bind failed, error code: %d\n", OS::Socket::getErrnoCode());
            return false;
        }
        return true;
    }

    /**
     * @brief Puts the socket into listening mode.
     *
     * Puts the socket into listening mode and specifies the maximum length of the queue of pending connections.
     *
     * @param nConnectionBacklog The maximum length of the queue of pending connections.
     * @return true if the socket was successfully put into listening mode, false otherwise.
     */
    virtual bool listen(int nConnectionBacklog) override {
        int ret = OS::Socket::listen(m_iSocketFd, nConnectionBacklog, m_RtMode);
        if (ret == SOCKET_ERROR) {
            return false;
        }
        return true;
    }

    /**
     * @brief Connects the socket to a remote address and port.
     *
     * Connects the socket to the specified remote address and port.
     *
     * @param strRemoteAddress The remote address to connect to.
     * @param uRemotePort The remote port to connect to.
     * @return true if the socket was successfully connected, false otherwise.
     */
    virtual bool connect(const string &strRemoteAddress, USHORT uRemotePort) override {
        CSockAddr m_SockAddr(strRemoteAddress, uRemotePort);
        int ret = OS::Socket::connect(m_iSocketFd, (sockaddr *)m_SockAddr, sizeof(CSockAddr), m_RtMode);
        if (ret == SOCKET_ERROR) {
            if (OS::Socket::getErrnoCode() == E_SOCKET_CODE(EINPROGRESS) ||
                OS::Socket::getErrnoCode() == E_SOCKET_CODE(EWOULDBLOCK)) {
                return true;
            } else
                return false;
        }
        return true;
    }

    /**
     * @brief Accepts a new connection on the socket.
     *
     * Accepts a new connection on the socket and returns a new socket object for the connection.
     *
     * @param rConnectedSocket Reference to the new socket object for the accepted connection.
     * @param strRemoteAddress Reference to the string where the remote address will be stored.
     * @param uRemotePort Reference to the variable where the remote port will be stored.
     * @param mode The real-time mode for the accepted socket (default is USE_DEFAULT).
     * @return true if a connection was successfully accepted, false otherwise.
     */
    virtual bool accept(ISocket *&rConnectedSocket, string &strRemoteAddress, USHORT &uRemotePort,
                        RT_MODE mode = USE_DEFAULT) override {
        if (rConnectedSocket == nullptr)
            return false;
        else {
            CSockAddr acceptAddr;
            OS_SOCKET_LEN iLen = sizeof(CSockAddr);
            OS_SOCKET sock_fd = (OS_SOCKET)INVALID_FD_VALUE;
#ifdef _WIN32
            sock_fd = OS::Socket::accept(m_iSocketFd, (sockaddr *)acceptAddr, &iLen, m_RtMode);
#else
            while (true) {
                sock_fd = OS::Socket::accept(m_iSocketFd, (sockaddr *)acceptAddr, &iLen, m_RtMode);
                if (sock_fd == (OS_SOCKET)INVALID_FD_VALUE && OS::Socket::getErrnoCode() == E_SOCKET_CODE(EINTR))
                    continue;
                else
                    break;
            }
#endif
            bool ret = rConnectedSocket->attach(sock_fd, mode);
            if (ret == true) {
                strRemoteAddress = acceptAddr.dottedDecimal();
                uRemotePort = acceptAddr.Port();
            }
            return ret;
        }
    }

    /**
     * @brief Sends data over the socket.
     *
     * Sends data over the socket to the connected peer.
     *
     * @param lpBuf Pointer to the buffer containing the data to be sent.
     * @param iBufLen The length of the data to be sent.
     * @param iFlags Flags for the send operation.
     * @return The number of bytes sent, or -1 if an error occurred.
     */
    virtual int send(const void *lpBuf, int iBufLen, int iFlags) override {
        int ret = 0;
        while (true) {
            ret = OS::Socket::send(m_iSocketFd, lpBuf, iBufLen, iFlags, m_RtMode);
            if (ret == -1 && OS::Socket::getErrnoCode() == E_SOCKET_CODE(EINTR))
                continue;
            else
                break;
        }
        return ret;
    }

    /**
     * @brief Receives data from the socket.
     *
     * Receives data from the socket from the connected peer.
     *
     * @param lpBuf Pointer to the buffer where the received data will be stored.
     * @param iBufLen The maximum length of the data to be received.
     * @param iFlags Flags for the receive operation.
     * @return The number of bytes received, or -1 if an error occurred.
     */
    virtual int receive(void *lpBuf, int iBufLen, int iFlags) override {
        int ret = 0;
        while (true) {
            ret = OS::Socket::recv(m_iSocketFd, lpBuf, iBufLen, iFlags, m_RtMode);
            if (ret == -1 && OS::Socket::getErrnoCode() == E_SOCKET_CODE(EINTR))
                continue;
            else
                break;
        }
        return ret;
    }

    /**
     * @brief Sends data to a specific address and port.
     *
     * Sends data to the specified remote address and port.
     *
     * @param lpBuf Pointer to the buffer containing the data to be sent.
     * @param iBufLen The length of the data to be sent.
     * @param strRemoteAddress The remote address to send data to.
     * @param uRemotePort The remote port to send data to.
     * @param iFlags Flags for the send operation.
     * @return The number of bytes sent, or -1 if an error occurred.
     */
    virtual int sendTo(const void *lpBuf, int iBufLen, const string &strRemoteAddress, USHORT uRemotePort,
                       int iFlags) override {
        CSockAddr remoteAddr(strRemoteAddress, uRemotePort);
        int ret = 0;
        while (true) {
            ret = OS::Socket::sendTo(m_iSocketFd, lpBuf, iBufLen, iFlags, (const sockaddr *)&remoteAddr,
                                     sizeof(CSockAddr), m_RtMode);
            if (ret == -1 && OS::Socket::getErrnoCode() == E_SOCKET_CODE(EINTR))
                continue;
            else
                break;
        }
        return ret;
    }

    /**
     * @brief Receives data from a specific address and port.
     *
     * Receives data from the specified remote address and port.
    0 *
     * @param lpBuf Pointer to the buffer where the received data will be stored.
     * @param iBufLen The maximum length of the data to be received.
     * @param strRemoteAddress Reference to the string where the remote address will be stored.
     * @param uRemotePort Reference to the variable where the remote port will be stored.
     * @param iFlags Flags for the receive operation.
     * @return The number of bytes received, or -1 if an error occurred.
     */
    virtual int receiveFrom(void *lpBuf, int iBufLen, string &strRemoteAddress, USHORT &uRemotePort,
                            int iFlags) override {
        CSockAddr remoteAddr;
        OS_SOCKET_LEN iLen = sizeof(CSockAddr);
        int ret = 0;
        while (true) {
            ret = OS::Socket::recvFrom(m_iSocketFd, lpBuf, iBufLen, iFlags, (sockaddr *)&remoteAddr, &iLen, m_RtMode);
            if (ret == -1 && OS::Socket::getErrnoCode() == E_SOCKET_CODE(EINTR))
                continue;
            else
                break;
        }
        if (ret >= 0) {
            strRemoteAddress = remoteAddr.dottedDecimal();
            uRemotePort = remoteAddr.Port();
        }
        return ret;
    }

    /**
     * @brief Gets the name of the remote peer.
     *
     * Gets the name (address and port) of the remote peer connected to the socket.
     *
     * @param strRemoteAddress Reference to the string where the remote address will be stored.
     * @param uRemotePort Reference to the variable where the remote port will be stored.
     * @return true if the remote peer name was successfully retrieved, false otherwise.
     */
    virtual bool getPeerName(string &strRemoteAddress, USHORT &uRemotePort) override {
        sockaddr name;
        OS_SOCKET_LEN namelen = sizeof(sockaddr);
        int ret = OS::Socket::getPeerName(m_iSocketFd, &name, &namelen, m_RtMode);
        if (ret == SOCKET_ERROR)
            return false;
        else {
            CSockAddr m_SockAddr(name);
            strRemoteAddress = m_SockAddr.dottedDecimal();
            uRemotePort = m_SockAddr.Port();
            return true;
        }
    }
    /**
     * @brief Gets the name of the socket.
     *
     * Gets the name (address and port) of the socket.
     *
     * @param strLocalAddress Reference to the string where the local address will be stored.
     * @param uLocalPort Reference to the variable where the local port will be stored.
     * @return true if the socket name was successfully retrieved, false otherwise.
     */
    virtual bool getSockName(string &strLocalAddress, USHORT &uLocalPort) override {
        sockaddr name;
        OS_SOCKET_LEN namelen = sizeof(sockaddr);
        int ret = OS::Socket::getSockName(m_iSocketFd, &name, &namelen, m_RtMode);
        if (ret == SOCKET_ERROR)
            return false;
        else {
            CSockAddr m_SockAddr(name);
            strLocalAddress = m_SockAddr.dottedDecimal();
            uLocalPort = m_SockAddr.Port();
            return true;
        }
    }

    /**
     * @brief Gets the value of a socket option.
     *
     * Gets the value of the specified socket option.
     *
     * @param iLevel The level at which the option is defined (e.g., SOL_SOCKET).
     * @param iOptname The name of the option to get.
     * @param pOptval Pointer to the buffer where the option value will be stored.
     * @param uOptlen Pointer to the variable where the length of the option value will be stored.
     * @return 0 if the option value was successfully retrieved, or -1 if an error occurred.
     */
    virtual int getSockopt(int iLevel, int iOptname, void *pOptval, OS_SOCKET_LEN *uOptlen) override {
        return OS::Socket::getSockOpt(m_iSocketFd, iLevel, iOptname, pOptval, uOptlen, m_RtMode);
    }

    /**
     * @brief Sets the value of a socket option.
     *
     * Sets the value of the specified socket option.
     *
     * @param iLevel The level at which the option is defined (e.g., SOL_SOCKET).
     * @param iOptname The name of the option to set.
     * @param pOptval Pointer to the buffer containing the option value.
     * @param uOptlen The length of the option value.
     * @return 0 if the option value was successfully set, or -1 if an error occurred.
     */
    virtual int setSockopt(int iLevel, int iOptname, const void *pOptval, OS_SOCKET_LEN uOptlen) override {
        return OS::Socket::setSockOpt(m_iSocketFd, iLevel, iOptname, pOptval, uOptlen, m_RtMode);
    }

protected:
    /**
     * @brief Called when the final release of the socket object is performed.
     *
     * Ensures that the socket is closed when the final release of the socket object is performed.
     */
    virtual void onFinalRelease() override { close(); }

    /**
     * @brief Called when the socket is attached to a file descriptor.
     *
     * This method can be overridden to perform additional actions when the socket is attached to a file descriptor.
     */
    virtual void onSocketAttach() {}

    /**
     * @brief Called when the socket is detached from a file descriptor.
     *
     * This method can be overridden to perform additional actions when the socket is detached from a file descriptor.
     */
    virtual void onSocketDetach() {}

protected:
    /**
     * @var OS_SOCKET m_iSocketFd
     * @brief The socket file descriptor.
     *
     * Stores the file descriptor of the socket.
     */
    OS_SOCKET m_iSocketFd;

    /**
     * @var RT_MODE m_RtMode
     * @brief The real-time mode for the socket.
     *
     * Specifies the real-time mode used by the socket.
     */
    RT_MODE m_RtMode;
};

#endif
