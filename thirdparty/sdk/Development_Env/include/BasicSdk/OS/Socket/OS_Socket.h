#ifndef __BASICSDK_OS_SOCKET_H__
#define __BASICSDK_OS_SOCKET_H__

#include "BasicSdkDef.h"
#include "OS/OSType.h"
#include "OSType_Socket.h"

#ifdef _WIN32
#define E_SOCKET_CODE(code) WSA##code
#else
#define E_SOCKET_CODE(code) code
#endif

/**
 * Initializes the socket library. This function must be called before using any other socket functions.
 *
 * @return Returns true if the initialization is successful, otherwise false.
 */
BASICSDK_API bool os_socket_init_library();

/**
 * Cleans up and releases resources used by the socket library. This function should be called after all socket
 * operations are completed.
 *
 * @return Returns true if the cleanup is successful, otherwise false.
 */
BASICSDK_API bool os_socket_clear_library();

/**
 * Retrieves the error code from the last socket operation that failed.
 *
 * @return The error code associated with the last socket operation failure.
 */
BASICSDK_API int os_socket_get_errno_code();

/**
 * Creates a socket with the specified domain, type, and protocol.
 *
 * @param domain Specifies the communication domain (e.g., AF_INET for IPv4).
 * @param type Specifies the socket type (e.g., SOCK_STREAM for TCP, SOCK_DGRAM for UDP).
 * @param protocol Specifies the protocol to use (e.g., IPPROTO_TCP or IPPROTO_UDP). Use 0 for default.
 * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
 *
 * @return Returns a socket descriptor on success, or INVALID_SOCKET on failure.
 */
BASICSDK_API OS_SOCKET os_socket(int domain, int type, int protocol, RT_MODE mode = USE_DEFAULT);

/**
 * Binds the specified socket to an address and port.
 *
 * @param s The socket descriptor to bind.
 * @param addr A pointer to the sockaddr structure containing the address information.
 * @param addrlen The length of the address structure.
 * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
 *
 * @return Returns 0 on success, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_bind(OS_SOCKET s, const struct sockaddr *addr, socklen_t addrlen,
                                RT_MODE mode = USE_DEFAULT);

/**
 * Sets the socket to listen for incoming connections.
 *
 * @param s The socket descriptor to set in listening mode.
 * @param backlog The maximum number of pending connections allowed.
 * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
 *
 * @return Returns 0 on success, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_listen(OS_SOCKET s, int backlog, RT_MODE mode = USE_DEFAULT);

/**
 * Connects the specified socket to a remote address.
 *
 * @param s The socket descriptor to connect.
 * @param addr A pointer to the sockaddr structure containing the remote address information.
 * @param addrlen The length of the address structure.
 * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
 *
 * @return Returns 0 on success, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_connect(OS_SOCKET s, const struct sockaddr *addr, socklen_t addrlen,
                                   RT_MODE mode = USE_DEFAULT);
/**
 * Accepts an incoming connection on the specified socket.
 *
 * @param s The listening socket descriptor.
 * @param addr A pointer to a sockaddr structure to store the address of the connecting entity.
 * @param addrlen A pointer to the length of the sockaddr structure.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns the accepted socket descriptor if successful; otherwise, returns INVALID_SOCKET.
 */
BASICSDK_API OS_SOCKET os_socket_accept(OS_SOCKET s, struct sockaddr *addr, socklen_t *addrlen,
                                        RT_MODE mode = USE_DEFAULT);

/**
 * Sends data over the specified socket.
 *
 * @param s The socket descriptor for sending data.
 * @param buf A pointer to the buffer containing the data to send.
 * @param len The length of the data to send.
 * @param flags Flags to modify the behavior of the send operation.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns the number of bytes sent if successful; otherwise, returns SOCKET_ERROR.
 */
BASICSDK_API int os_socket_send(OS_SOCKET s, const void *buf, int len, int flags, RT_MODE mode = USE_DEFAULT);

/**
 * Receives data from the specified socket.
 *
 * @param s The socket descriptor for receiving data.
 * @param buf A pointer to the buffer to store the received data.
 * @param len The length of the buffer.
 * @param flags Flags to modify the behavior of the receive operation.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns the number of bytes received if successful; otherwise, returns SOCKET_ERROR.
 */
BASICSDK_API int os_socket_recv(OS_SOCKET s, void *buf, int len, int flags, RT_MODE mode = USE_DEFAULT);

/**
 * Sends data to the specified destination address over the socket.
 *
 * @param s The socket descriptor for sending data.
 * @param buf A pointer to the buffer containing the data to send.
 * @param len The length of the data to send.
 * @param flags Flags to modify the behavior of the send operation.
 * @param dest_addr A pointer to the sockaddr structure specifying the destination address.
 * @param addrlen The length of the destination address.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns the number of bytes sent if successful; otherwise, returns SOCKET_ERROR.
 */
BASICSDK_API int os_socket_send_to(OS_SOCKET s, const void *buf, int len, int flags, const struct sockaddr *dest_addr,
                                   socklen_t addrlen, RT_MODE mode = USE_DEFAULT);

/**
 * Receives data from the specified socket and retrieves the sender's address.
 *
 * @param s The socket descriptor for receiving data.
 * @param buf A pointer to the buffer to store the received data.
 * @param len The length of the buffer.
 * @param flags Flags to modify the behavior of the receive operation.
 * @param src_addr A pointer to the sockaddr structure to store the sender's address.
 * @param addrlen A pointer to the length of the sockaddr structure.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns the number of bytes received if successful; otherwise, returns SOCKET_ERROR.
 */
BASICSDK_API int os_socket_recv_from(OS_SOCKET s, void *buf, int len, int flags, struct sockaddr *src_addr,
                                     socklen_t *addrlen, RT_MODE mode = USE_DEFAULT);

/**
 * Retrieves the address of the peer connected to the specified socket.
 *
 * @param s The socket descriptor.
 * @param addr A pointer to the sockaddr structure to store the peer's address.
 * @param addrlen A pointer to the length of the sockaddr structure.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns 0 if successful; otherwise, returns SOCKET_ERROR.
 */
BASICSDK_API int os_socket_get_peer_name(OS_SOCKET s, struct sockaddr *addr, socklen_t *addrlen,
                                         RT_MODE mode = USE_DEFAULT);

/**
 * Retrieves the current address associated with the specified socket.
 *
 * @param s The socket descriptor.
 * @param addr A pointer to the sockaddr structure to store the socket's address.
 * @param addrlen A pointer to the length of the sockaddr structure.
 * @param mode Optional real-time mode, default is USE_DEFAULT.
 * @return Returns 0 if successful; otherwise, returns SOCKET_ERROR.
 */
BASICSDK_API int os_socket_get_sock_name(OS_SOCKET s, struct sockaddr *addr, socklen_t *addrlen,
                                         RT_MODE mode = USE_DEFAULT);
/**
 * @brief Waits for multiple sockets to change status based on the specified conditions.
 *
 * This function monitors sets of file descriptors (sockets) to see if they are ready
 * for reading, writing, or if an exceptional condition has occurred.
 *
 * @param nfds The highest-numbered file descriptor in any of the three sets, plus 1.
 * @param readfds A pointer to a set of sockets to be checked for readability.
 * @param writefds A pointer to a set of sockets to be checked for writability.
 * @param exceptfds A pointer to a set of sockets to be checked for exceptional conditions.
 * @param timeout A pointer to a timeval structure specifying the maximum time to wait.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 * @return int Returns the number of ready descriptors, 0 if timed out, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
                                  struct timeval *timeout, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Performs control operations on a socket.
 *
 * This function performs various control operations on a socket, depending on the command provided.
 *
 * @param s The socket descriptor.
 * @param cmd The control command to execute.
 * @param argp A pointer to the argument for the command.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 * @return int Returns 0 on success, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_ioctl(OS_SOCKET s, ULONG cmd, ULONG *argp, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Closes a socket.
 *
 * This function closes the specified socket and releases associated resources.
 *
 * @param s The socket descriptor to close.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 */
BASICSDK_API void os_socket_close(OS_SOCKET s, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Disables the reporting of UDP unreachable messages.
 *
 * This function disables the generation of ICMP "destination unreachable" messages for UDP sockets.
 *
 * @param s The socket descriptor.
 */
BASICSDK_API void os_socket_disable_udp_unreached_report(OS_SOCKET s);

/**
 * @brief Sets the send timeout for a socket.
 *
 * This function sets the timeout value for sending data on the specified socket.
 *
 * @param s The socket descriptor.
 * @param dwMillionSecond The timeout value in milliseconds.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 * @return int Returns 0 on success, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_set_send_timeout(OS_SOCKET s, DWORD dwMillionSecond, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Sets the receive timeout for a socket.
 *
 * This function sets the timeout value for receiving data on the specified socket.
 *
 * @param s The socket descriptor.
 * @param dwMillionSecond The timeout value in milliseconds.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 * @return int Returns 0 on success, or SOCKET_ERROR on failure.
 */
BASICSDK_API int os_socket_set_recv_timeout(OS_SOCKET s, DWORD dwMillionSecond, RT_MODE mode = USE_DEFAULT);

/**
 * @brief Checks if a socket is readable within a given timeout.
 *
 * This function checks whether the specified socket has data available to be read within the timeout period.
 *
 * @param s The socket descriptor.
 * @param dwMillionSecond The timeout value in milliseconds.
 * @param intErrorCode Optional pointer to store the error code if the function fails.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 * @return bool Returns true if the socket is readable, otherwise false.
 */
BASICSDK_API bool os_socket_readable(OS_SOCKET s, DWORD dwMillionSecond, int *intErrorCode = nullptr,
                                     RT_MODE mode = USE_DEFAULT);

/**
 * @brief Checks if a socket is writable within a given timeout.
 *
 * This function checks whether the specified socket is ready to send data within the timeout period.
 *
 * @param s The socket descriptor.
 * @param dwMillionSecond The timeout value in milliseconds.
 * @param intErrorCode Optional pointer to store the error code if the function fails.
 * @param mode The real-time operation mode. Default is USE_DEFAULT.
 * @return bool Returns true if the socket is writable, otherwise false.
 */
BASICSDK_API bool os_socket_writable(OS_SOCKET s, DWORD dwMillionSecond, int *intErrorCode = nullptr,
                                     RT_MODE mode = USE_DEFAULT);
/**
 * Checks if the socket is readable within the specified time.
 *
 * @param s Socket descriptor.
 * @param dwMicroSecond Timeout duration in microseconds.
 * @param intErrorCode Pointer to store error code if the function fails.
 * @param mode Real-time mode flag, USE_DEFAULT for default settings.
 * @return True if the socket becomes readable within the specified time.
 *
 * This function determines whether a socket becomes readable within a specified number of microseconds.
 * It is commonly used in non-blocking mode to quickly check if data is available.
 */
BASICSDK_API bool os_socket_precision_readable(OS_SOCKET s, DWORD dwMicroSecond, int *intErrorCode = nullptr,
                                               RT_MODE mode = USE_DEFAULT);

/**
 * Checks if the socket is writable within the specified time.
 *
 * @param s Socket descriptor.
 * @param dwMicroSecond Timeout duration in microseconds.
 * @param intErrorCode Pointer to store error code if the function fails.
 * @param mode Real-time mode flag, USE_DEFAULT for default settings.
 * @return True if the socket becomes writable within the specified time.
 *
 * This function determines whether a socket becomes writable within a specified number of microseconds.
 * It is commonly used in non-blocking mode to quickly check if data can be sent.
 */
BASICSDK_API bool os_socket_precision_writable(OS_SOCKET s, DWORD dwMicroSecond, int *intErrorCode = nullptr,
                                               RT_MODE mode = USE_DEFAULT);

/**
 * Checks if the socket connection is broken.
 *
 * @param s Socket descriptor.
 * @param mode Real-time mode flag, USE_DEFAULT for default settings.
 * @return True if the connection is broken.
 *
 * This function detects the connection status of a socket, particularly useful for long-running applications.
 * It ensures that the peer connection remains valid and handles disconnected cases promptly.
 */
BASICSDK_API bool os_socket_is_link_broken(OS_SOCKET s, RT_MODE mode = USE_DEFAULT);

/**
 * Gets the value of a socket option.
 *
 * @param s Socket descriptor.
 * @param iLevel Option level, such as SOL_SOCKET.
 * @param iOptname Option name, such as SO_REUSEADDR.
 * @param pOptval Pointer to buffer storing the option value.
 * @param uOptlen Pointer to buffer length.
 * @param mode Real-time mode flag, USE_DEFAULT for default settings.
 * @return True if getting the option value succeeds.
 *
 * This function retrieves a specific option value of a socket to check or modify its behavior at runtime.
 */
BASICSDK_API bool os_socket_get_sock_opt(OS_SOCKET s, int iLevel, int iOptname, void *pOptval, OS_SOCKET_LEN *uOptlen,
                                         RT_MODE mode = USE_DEFAULT);

/**
 * Sets the value of a socket option.
 *
 * @param s Socket descriptor.
 * @param iLevel Option level, such as SOL_SOCKET.
 * @param iOptname Option name, such as SO_REUSEADDR.
 * @param pOptval Pointer to constant buffer containing the option value.
 * @param uOptlen Buffer length.
 * @param mode Real-time mode flag, USE_DEFAULT for default settings.
 * @return True if setting the option value succeeds.
 *
 * This function sets a specific option value of a socket to customize its behavior at runtime.
 * For example, it can be used to set the socket to reuse address mode.
 */
BASICSDK_API bool os_socket_set_sock_opt(OS_SOCKET s, int iLevel, int iOptname, const void *pOptval,
                                         OS_SOCKET_LEN uOptlen, RT_MODE mode = USE_DEFAULT);

namespace OS {
class Socket {
public:
    /**
     * Initializes the socket library. This function must be called before using any other socket functions.
     *
     * @return Returns true if the initialization is successful, otherwise false.
     */
    static bool initLibrary() { return os_socket_init_library(); }

    /**
     * Cleans up and releases resources used by the socket library. This function should be called after all socket
     * operations are completed.
     *
     * @return Returns true if the cleanup is successful, otherwise false.
     */
    static bool clearLibrary() { return os_socket_clear_library(); }

    /**
     * Retrieves the error code from the last socket operation that failed.
     *
     * @return The error code associated with the last socket operation failure.
     */
    static int getErrnoCode() { return os_socket_get_errno_code(); }

    /**
     * Creates a socket with the specified domain, type, and protocol.
     *
     * @param domain Specifies the communication domain (e.g., AF_INET for IPv4).
     * @param type Specifies the socket type (e.g., SOCK_STREAM for TCP, SOCK_DGRAM for UDP).
     * @param protocol Specifies the protocol to use (e.g., IPPROTO_TCP or IPPROTO_UDP). Use 0 for default.
     * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
     *
     * @return Returns a socket descriptor on success, or INVALID_SOCKET on failure.
     */
    static OS_SOCKET socket(int domain, int type, int protocol, RT_MODE mode = USE_DEFAULT) {
        return os_socket(domain, type, protocol, mode);
    }

    /**
     * Binds the specified socket to an address and port.
     *
     * @param s The socket descriptor to bind.
     * @param addr A pointer to the sockaddr structure containing the address information.
     * @param addrlen The length of the address structure.
     * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
     *
     * @return Returns 0 on success, or SOCKET_ERROR on failure.
     */
    static int bind(OS_SOCKET s, const struct sockaddr *addr, socklen_t addrlen, RT_MODE mode = USE_DEFAULT) {
        return os_socket_bind(s, addr, addrlen, mode);
    }

    /**
     * Sets the socket to listen for incoming connections.
     *
     * @param s The socket descriptor to set in listening mode.
     * @param backlog The maximum number of pending connections allowed.
     * @param mode Specifies the real-time mode. USE_DEFAULT is typically used.
     *
     * @return Returns 0 on success, or SOCKET_ERROR on failure.
     */
    static int listen(OS_SOCKET s, int backlog, RT_MODE mode = USE_DEFAULT) {
        return os_socket_listen(s, backlog, mode);
    }
    /**
     * Initiates a connection on the specified socket to the given address.
     *
     * @param s The socket descriptor to connect.
     * @param addr A pointer to the sockaddr structure containing the remote address information.
     * @param addrlen The length of the address structure.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns 0 if successful; otherwise, returns SOCKET_ERROR.
     */
    static int connect(OS_SOCKET s, const struct sockaddr *addr, socklen_t addrlen, RT_MODE mode = USE_DEFAULT) {
        return os_socket_connect(s, addr, addrlen, mode);
    }

    /**
     * Accepts an incoming connection request on the specified listening socket.
     *
     * @param s The listening socket descriptor.
     * @param addr A pointer to a sockaddr structure to store the address of the connecting entity.
     * @param addrlen A pointer to the length of the sockaddr structure.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns the accepted socket descriptor if successful; otherwise, returns INVALID_SOCKET.
     */
    static OS_SOCKET accept(OS_SOCKET s, struct sockaddr *addr, socklen_t *addrlen, RT_MODE mode = USE_DEFAULT) {
        return os_socket_accept(s, addr, addrlen, mode);
    }

    /**
     * Sends data over the specified connected socket.
     *
     * @param s The socket descriptor for sending data.
     * @param buf A pointer to the buffer containing the data to send.
     * @param len The length of the data to send.
     * @param flags Flags to modify the behavior of the send operation.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns the number of bytes sent if successful; otherwise, returns SOCKET_ERROR.
     */
    static int send(OS_SOCKET s, const void *buf, int len, int flags, RT_MODE mode = USE_DEFAULT) {
        return os_socket_send(s, buf, len, flags, mode);
    }

    /**
     * Receives data from the specified connected socket.
     *
     * @param s The socket descriptor for receiving data.
     * @param buf A pointer to the buffer to store the received data.
     * @param len The length of the buffer.
     * @param flags Flags to modify the behavior of the receive operation.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns the number of bytes received if successful; otherwise, returns SOCKET_ERROR.
     */
    static int recv(OS_SOCKET s, void *buf, int len, int flags, RT_MODE mode = USE_DEFAULT) {
        return os_socket_recv(s, buf, len, flags, mode);
    }

    /**
     * Sends data to the specified destination address over the socket.
     *
     * @param s The socket descriptor for sending data.
     * @param buf A pointer to the buffer containing the data to send.
     * @param len The length of the data to send.
     * @param flags Flags to modify the behavior of the send operation.
     * @param dest_addr A pointer to the sockaddr structure specifying the destination address.
     * @param addrlen The length of the destination address.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns the number of bytes sent if successful; otherwise, returns SOCKET_ERROR.
     */
    static int sendTo(OS_SOCKET s, const void *buf, int len, int flags, const struct sockaddr *dest_addr,
                      socklen_t addrlen, RT_MODE mode = USE_DEFAULT) {
        return os_socket_send_to(s, buf, len, flags, dest_addr, addrlen, mode);
    }

    /**
     * Receives data from the specified socket and retrieves the sender's address.
     *
     * @param s The socket descriptor for receiving data.
     * @param buf A pointer to the buffer to store the received data.
     * @param len The length of the buffer.
     * @param flags Flags to modify the behavior of the receive operation.
     * @param src_addr A pointer to the sockaddr structure to store the sender's address.
     * @param addrlen A pointer to the length of the sockaddr structure.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns the number of bytes received if successful; otherwise, returns SOCKET_ERROR.
     */
    static int recvFrom(OS_SOCKET s, void *buf, int len, int flags, struct sockaddr *src_addr, socklen_t *addrlen,
                        RT_MODE mode = USE_DEFAULT) {
        return os_socket_recv_from(s, buf, len, flags, src_addr, addrlen, mode);
    }
    /**
     * Retrieves the name of the peer connected to the specified socket.
     *
     * @param s The socket descriptor.
     * @param addr A pointer to a sockaddr structure where the peer's address will be stored.
     * @param addrlen A pointer to the length of the sockaddr structure.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns 0 on success, SOCKET_ERROR on failure.
     */
    static int getPeerName(OS_SOCKET s, struct sockaddr *addr, socklen_t *addrlen, RT_MODE mode = USE_DEFAULT) {
        return os_socket_get_peer_name(s, addr, addrlen, mode);
    }

    /**
     * Retrieves the name of the local socket.
     *
     * @param s The socket descriptor.
     * @param addr A pointer to a sockaddr structure where the socket's address will be stored.
     * @param addrlen A pointer to the length of the sockaddr structure.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns 0 on success, SOCKET_ERROR on failure.
     */
    static int getSockName(OS_SOCKET s, struct sockaddr *addr, socklen_t *addrlen, RT_MODE mode = USE_DEFAULT) {
        return os_socket_get_sock_name(s, addr, addrlen, mode);
    }

    /**
     * Waits for one or more sockets to become ready to perform I/O operations.
     *
     * @param nfds The highest-numbered file descriptor in any of the three sets, plus 1.
     * @param readfds A pointer to a set of sockets to be checked for readability.
     * @param writefds A pointer to a set of sockets to be checked for writability.
     * @param exceptfds A pointer to a set of sockets to be checked for exceptional conditions.
     * @param timeout A pointer to a timeval structure specifying the maximum time to wait.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns the number of ready descriptors, 0 if timed out, SOCKET_ERROR on failure.
     */
    static int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout,
                      RT_MODE mode = USE_DEFAULT) {
        return os_socket_select(nfds, readfds, writefds, exceptfds, timeout, mode);
    }

    /**
     * Performs a control operation on the specified socket.
     *
     * @param s The socket descriptor.
     * @param cmd The control command to execute.
     * @param argp A pointer to an argument for the command.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     * @return Returns 0 on success, SOCKET_ERROR on failure.
     */
    static int ioctl(OS_SOCKET s, ULONG cmd, ULONG *argp, RT_MODE mode = USE_DEFAULT) {
        return os_socket_ioctl(s, cmd, argp, mode);
    }

    /**
     * Closes the specified socket and releases associated resources.
     *
     * @param s The socket descriptor to close.
     * @param mode Optional real-time mode, default is USE_DEFAULT.
     */
    static void close(OS_SOCKET s, RT_MODE mode = USE_DEFAULT) { return os_socket_close(s, mode); }
    /**
     * @brief Disables the reporting of UDP unreachable messages.
     *
     * This function disables the generation of ICMP "destination unreachable" messages for UDP sockets.
     *
     * @param s The socket descriptor.
     */
    static void disableUDPUnreachedReport(OS_SOCKET s) { return os_socket_disable_udp_unreached_report(s); }

    /**
     * @brief Sets the send timeout for a socket.
     *
     * This function sets the timeout value for sending data on the specified socket.
     *
     * @param s The socket descriptor.
     * @param dwMillionSecond The timeout value in milliseconds.
     * @param mode The real-time operation mode. Default is USE_DEFAULT.
     * @return int Returns 0 on success, or SOCKET_ERROR on failure.
     */
    static int setSendTimeout(OS_SOCKET s, DWORD dwMillionSecond, RT_MODE mode = USE_DEFAULT) {
        return os_socket_set_send_timeout(s, dwMillionSecond, mode);
    }

    /**
     * @brief Sets the receive timeout for a socket.
     *
     * This function sets the timeout value for receiving data on the specified socket.
     *
     * @param s The socket descriptor.
     * @param dwMillionSecond The timeout value in milliseconds.
     * @param mode The real-time operation mode. Default is USE_DEFAULT.
     * @return int Returns 0 on success, or SOCKET_ERROR on failure.
     */
    static int setRecvTimeout(OS_SOCKET s, DWORD dwMillionSecond, RT_MODE mode = USE_DEFAULT) {
        return os_socket_set_recv_timeout(s, dwMillionSecond, mode);
    }

    /**
     * @brief Checks if a socket is readable within a given timeout.
     *
     * This function checks whether the specified socket has data available to be read within the timeout period.
     *
     * @param s The socket descriptor.
     * @param dwMillionSecond The timeout value in milliseconds.
     * @param intErrorCode Optional pointer to store the error code if the function fails.
     * @param mode The real-time operation mode. Default is USE_DEFAULT.
     * @return bool Returns true if the socket is readable, otherwise false.
     */
    static bool readable(OS_SOCKET s, DWORD dwMillionSecond, int *intErrorCode = nullptr, RT_MODE mode = USE_DEFAULT) {
        return os_socket_readable(s, dwMillionSecond, intErrorCode, mode);
    }

    /**
     * @brief Checks if a socket is writable within a given timeout.
     *
     * This function checks whether the specified socket is ready to send data within the timeout period.
     *
     * @param s The socket descriptor.
     * @param dwMillionSecond The timeout value in milliseconds.
     * @param intErrorCode Optional pointer to store the error code if the function fails.
     * @param mode The real-time operation mode. Default is USE_DEFAULT.
     * @return bool Returns true if the socket is writable, otherwise false.
     */
    static bool writable(OS_SOCKET s, DWORD dwMillionSecond, int *intErrorCode = nullptr, RT_MODE mode = USE_DEFAULT) {
        return os_socket_writable(s, dwMillionSecond, intErrorCode, mode);
    }
    /**
     * @brief Checks if the socket is readable within the specified time.
     *
     * This function determines whether a socket becomes readable within a specified number of microseconds.
     * It is commonly used in non-blocking mode to quickly check if data is available.
     *
     * @param s Socket descriptor.
     * @param dwMicroSecond Timeout duration in microseconds.
     * @param intErrorCode Pointer to store error code if the function fails. Optional, default is nullptr.
     * @param mode Real-time mode flag, USE_DEFAULT for default settings. Optional, default is USE_DEFAULT.
     * @return True if the socket becomes readable within the specified time.
     */
    static bool precision_readable(OS_SOCKET s, DWORD dwMicroSecond, int *intErrorCode = nullptr,
                                   RT_MODE mode = USE_DEFAULT) {
        return os_socket_precision_readable(s, dwMicroSecond, intErrorCode, mode);
    }

    /**
     * @brief Checks if the socket is writable within the specified time.
     *
     * This function determines whether a socket becomes writable within a specified number of microseconds.
     * It is commonly used in non-blocking mode to quickly check if data can be sent.
     *
     * @param s Socket descriptor.
     * @param dwMicroSecond Timeout duration in microseconds.
     * @param intErrorCode Pointer to store error code if the function fails. Optional, default is nullptr.
     * @param mode Real-time mode flag, USE_DEFAULT for default settings. Optional, default is USE_DEFAULT.
     * @return True if the socket becomes writable within the specified time.
     */
    static bool precision_writable(OS_SOCKET s, DWORD dwMicroSecond, int *intErrorCode = nullptr,
                                   RT_MODE mode = USE_DEFAULT) {
        return os_socket_precision_writable(s, dwMicroSecond, intErrorCode, mode);
    }

    /**
     * @brief Checks if the socket connection is broken.
     *
     * This function detects the connection status of a socket, particularly useful for long-running applications.
     * It ensures that the peer connection remains valid and handles disconnected cases promptly.
     *
     * @param s Socket descriptor.
     * @param mode Real-time mode flag, USE_DEFAULT for default settings. Optional, default is USE_DEFAULT.
     * @return True if the connection is broken.
     */
    static bool isLinkBroken(OS_SOCKET s, RT_MODE mode = USE_DEFAULT) { return os_socket_is_link_broken(s, mode); }

    /**
     * @brief Gets the value of a socket option.
     *
     * This function retrieves a specific option value of a socket to check or modify its behavior at runtime.
     *
     * @param s Socket descriptor.
     * @param iLevel Option level, such as SOL_SOCKET.
     * @param iOptname Option name, such as SO_REUSEADDR.
     * @param pOptval Pointer to buffer storing the option value.
     * @param uOptlen Pointer to buffer length.
     * @param mode Real-time mode flag, USE_DEFAULT for default settings. Optional, default is USE_DEFAULT.
     * @return 0 on success, SOCKET_ERROR on failure.
     */
    static int getSockOpt(OS_SOCKET s, int iLevel, int iOptname, void *pOptval, OS_SOCKET_LEN *uOptlen,
                          RT_MODE mode = USE_DEFAULT) {
        return os_socket_get_sock_opt(s, iLevel, iOptname, pOptval, uOptlen, mode);
    }

    /**
     * @brief Sets the value of a socket option.
     *
     * This function sets a specific option value of a socket to customize its behavior at runtime.
     * For example, it can be used to set the socket to reuse address mode.
     *
     * @param s Socket descriptor.
     * @param iLevel Option level, such as SOL_SOCKET.
     * @param iOptname Option name, such as SO_REUSEADDR.
     * @param pOptval Pointer to constant buffer containing the option value.
     * @param uOptlen Buffer length.
     * @param mode Real-time mode flag, USE_DEFAULT for default settings. Optional, default is USE_DEFAULT.
     * @return 0 on success, SOCKET_ERROR on failure.
     */
    static int setSockOpt(OS_SOCKET s, int iLevel, int iOptname, const void *pOptval, OS_SOCKET_LEN uOptlen,
                          RT_MODE mode = USE_DEFAULT) {
        return os_socket_set_sock_opt(s, iLevel, iOptname, pOptval, uOptlen, mode);
    }
};
}  // namespace OS

#endif
