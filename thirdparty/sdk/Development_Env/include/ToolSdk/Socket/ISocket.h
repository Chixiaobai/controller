#pragma once

#include "Base/IBase.h"

/**
 * @interface ISocket
 * @brief Interface for socket operations.
 *
 * This interface defines the methods for socket operations, including attaching and detaching sockets,
 * initializing, binding, listening, connecting, accepting connections, sending and receiving data,
 * and getting and setting socket options.
 */
interface ISocket : public IBase {
    /**
     * @brief Attaches an existing socket file descriptor to the socket object.
     *
     * Attaches an existing socket file descriptor to the socket object and sets the real-time mode.
     *
     * @param iSocketFd The socket file descriptor to attach.
     * @param mode The real-time mode for the socket (default is USE_DEFAULT).
     * @return true if the socket was successfully attached, false otherwise.
     */
    virtual bool attach(OS_SOCKET iSocketFd, RT_MODE mode = USE_DEFAULT) = 0;

    /**
     * @brief Detaches the socket file descriptor from the socket object.
     *
     * Detaches the socket file descriptor from the socket object and returns it.
     *
     * @return The detached socket file descriptor.
     */
    virtual OS_SOCKET dettach() = 0;

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
                            RT_MODE mode = USE_DEFAULT) = 0;

    /**
     * @brief Closes the socket.
     *
     * Closes the socket and releases any associated resources.
     *
     * @return true if the socket was successfully closed, false otherwise.
     */
    virtual bool close() = 0;

    /**
     * @brief Binds the socket to a local address and port.
     *
     * Binds the socket to the specified local address and port.
     *
     * @param strLocalAddress The local address to bind the socket to.
     * @param uLocalPort The local port to bind the socket to.
     * @return true if the socket was successfully bound, false otherwise.
     */
    virtual bool bind(const string &strLocalAddress, USHORT uLocalPort) = 0;

    /**
     * @brief Puts the socket into listening mode.
     *
     * Puts the socket into listening mode and specifies the maximum length of the queue of pending connections.
     *
     * @param nConnectionBacklog The maximum length of the queue of pending connections.
     * @return true if the socket was successfully put into listening mode, false otherwise.
     */
    virtual bool listen(int nConnectionBacklog) = 0;

    /**
     * @brief Connects the socket to a remote address and port.
     *
     * Connects the socket to the specified remote address and port.
     *
     * @param strRemoteAddress The remote address to connect to.
     * @param uRemotePort The remote port to connect to.
     * @return true if the socket was successfully connected, false otherwise.
     */
    virtual bool connect(const string &strRemoteAddress, USHORT uRemotePort) = 0;

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
    virtual bool accept(ISocket * &rConnectedSocket, string & strRemoteAddress, USHORT & uRemotePort,
                        RT_MODE mode = USE_DEFAULT) = 0;

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
    virtual int send(const void *lpBuf, int iBufLen, int iFlags) = 0;

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
    virtual int receive(void *lpBuf, int iBufLen, int iFlags) = 0;

    /**
     * @brief Sends data to a specific address and port.
     *
     * Sends data to the specified remote address and port.
     *
     * @param lpBuf Pointer to the buffer containing the data to be sent.
     * @param iBufLen The length of the data to be sent.
     *0 @param strRemoteAddress The remote address to send data to.
     * @param uRemotePort The remote port to send data to.
     * @param iFlags Flags for the send operation.
     * @return The number of bytes sent, or -1 if an error occurred.
     */
    virtual int sendTo(const void *lpBuf, int iBufLen, const string &strRemoteAddress, USHORT uRemotePort,
                       int iFlags) = 0;

    /**
     * @brief Receives data from a specific address and port.
     *
     * Receives data from the specified remote address and port.
     *
     * @param lpBuf Pointer to the buffer where the received data will be stored.
     * @param iBufLen The maximum length of the data to be received.
     * @param strRemoteAddress Reference to the string where the remote address will be stored.
     * @param uRemotePort Reference to the variable where the remote port will be stored.
     * @param iFlags Flags for the receive operation.
     * @return The number of bytes received, or -1 if an error occurred.
     */
    virtual int receiveFrom(void *lpBuf, int iBufLen, string &strRemoteAddress, USHORT &uRemotePort, int iFlags) = 0;

    /**
     * @brief Gets the name of the remote peer.
     *
     * Gets the name (address and port) of the remote peer connected to the socket.
     *
     * @param strRemoteAddress Reference to the string where the remote address will be stored.
     * @param uRemotePort Reference to the variable where the remote port will be stored.
     * @return true if the remote peer name was successfully retrieved, false otherwise.
     */
    virtual bool getPeerName(string & strRemoteAddress, USHORT & uRemotePort) = 0;

    /**
     * @brief Gets the name of the socket.
     *
     * Gets the name (address and port) of the socket.
     *
     * @param strLocalAddress Reference to the string where the local address will be stored.
     * @param uLocalPort Reference to the variable where the local port will be stored.
     * @return true if the socket name was successfully retrieved, false otherwise.
     */
    virtual bool getSockName(string & strLocalAddress, USHORT & uLocalPort) = 0;

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
    virtual int getSockopt(int iLevel, int iOptname, void *pOptval, OS_SOCKET_LEN *uOptlen) = 0;

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
    virtual int setSockopt(int iLevel, int iOptname, const void *pOptval, OS_SOCKET_LEN uOptlen) = 0;
};