#pragma once

#include "NodeDef.h"
#include "ModuleDef/IModuleVersion.h"
#include "IPort.h"

/**
 * @file NodeApi.h
 * @brief Public C API for managing communication nodes, topics, and services.
 *
 * This file defines the interface for creating and managing communication nodes,
 * publishing and subscribing to topics, and invoking or handling remote procedure calls (services).
 */

/**
 * @brief Retrieves the module version information.
 * @return Pointer to an IModuleVersion object containing version details.
 */
NODE_API IModuleVersion *node_get_module_version();

// ====== Node Lifecycle Management ======

/**
 * @brief Initializes the node management system.
 *
 * This must be called before any other node-related functions are used.
 *
 * @param arg Optional argument count (similar to main function).
 * @param argv Optional argument values (similar to main function).
 * @param mode Runtime mode. Default is USE_STD.
 * @return true if initialization succeeded; false otherwise.
 */
NODE_API bool node_api_initialize(int arg = 0, char **argv = NULL, RT_MODE mode = USE_STD);

/**
 * @brief Uninitializes and cleans up the node management system.
 * @return true if uninitialization succeeded; false otherwise.
 */
NODE_API bool node_api_uninitialize();

/**
 * @brief Creates a new communication node.
 *
 * Each node has a unique name and can host multiple ports and services.
 *
 * @param nodeName Name of the node to create. Must be unique.
 * @return true if the node was created successfully; false otherwise.
 */
NODE_API bool node_create(const char *nodeName);

/**
 * @brief Destroys a previously created communication node.
 * @param nodeName Name of the node to destroy.
 * @return true if the node was destroyed successfully; false otherwise.
 */
NODE_API bool node_destroy(const char *nodeName);

/**
 * @brief Destroys all existing communication nodes.
 * @return true if all nodes were destroyed successfully; false otherwise.
 */
NODE_API bool node_destroy_all();

// ====== Topic-based Communication (Publish/Subscribe) ======

/**
 * @brief Creates an output port for publishing messages on a topic.
 *
 * @warning This function should not be called in real-time contexts as it may involve Python parsing of IDF files.
 *
 * @param nodeName Name of the node that owns this port.
 * @param topicName Name of the topic to publish to.
 * @param messageDefName Name of the message definition (e.g., schema name).
 * @param buffers Number of message buffers to allocate. Default is 1.
 * @return Pointer to the created IOutputPort instance, or nullptr on failure.
 */
NODE_API IOutputPort *node_publish(const char *nodeName, const char *topicName, const char *messageDefName,
                                  unsigned int buffers = 1);

/**
 * @brief Destroys a previously created output port.
 * @param nodeName Name of the owning node.
 * @param port Pointer to the output port to destroy.
 * @return true if the port was destroyed successfully; false otherwise.
 */
NODE_API bool node_unpublish(const char *nodeName, IOutputPort *port);

/**
 * @brief Creates an input port for subscribing to messages on a topic.
 *
 * @param nodeName Name of the node that owns this port.
 * @param topicName Name of the topic to subscribe to.
 * @param bufLength Length of each message buffer in bytes.
 * @param buffers Number of message buffers to allocate. Default is 1.
 * @return Pointer to the created IInputPort instance, or nullptr on failure.
 */
NODE_API IInputPort *node_subscribe(const char *nodeName, const char *topicName, unsigned int bufLength,
                                 unsigned int buffers = 1);

/**
 * @brief Destroys a previously created input port.
 * @param nodeName Name of the owning node.
 * @param port Pointer to the input port to destroy.
 * @return true if the port was destroyed successfully; false otherwise.
 */
NODE_API bool node_unsubscribe(const char *nodeName, IInputPort *port);

// ====== Service-based Communication (RPC) ======

/**
 * @brief Creates a service instance.
 *
 * @param nodeName Name of the node that will own this service.
 * @param serviceName Name of the service to create.
 * @param ppService Output parameter: pointer to the created service object.
 * @return Status code (0 for success).
 */
NODE_API int node_service_create(const char *nodeName, const char *serviceName, pService *ppService);

/**
 * @brief Destroys a service instance.
 *
 * @param nodeName Name of the node that owns the service.
 * @param pService Pointer to the service object to destroy.
 * @return Status code (0 for success).
 */
NODE_API int node_service_destroy(const char *nodeName, const pService pService);

/**
 * @brief Sets the callback handler for incoming service requests.
 *
 * If no handler is set, the service behaves as a client only.
 *
 * @param nodeName Name of the node that owns the service.
 * @param pService Pointer to the service object.
 * @param handler Function pointer to the request handler.
 * @param userData User-defined data passed to the handler.
 * @return Status code (0 for success).
 */
NODE_API int node_service_set_handle(const char *nodeName, const pService pService, service_callback_func handler,
                                     void *userData);

/**
 * @deprecated This function is obsolete and should not be used.
 * @brief [Deprecated] Sends a response to a service request.
 *
 * @param nodeName Name of the node that owns the service.
 * @param pService Pointer to the service object.
 * @param data Pointer to the response data.
 * @param dataLen Length of the response data in bytes.
 * @return Status code (0 for success).
 */
NODE_API int node_service_write_response(const char *nodeName, const pService pService, const void *data,
                                         unsigned long dataLen);

/**
 * @brief Checks whether a service provider is available.
 *
 * @param nodeName Name of the node that owns the service.
 * @param pService Pointer to the service object.
 * @param timeout Maximum time to wait in milliseconds.
 * @return 0 if a provider is available; -1 if no provider exists within the timeout;
 *         other negative values indicate errors.
 */
NODE_API int node_service_has_provider(const char *nodeName, const pService pService, unsigned int timeout = 1000);

/**
 * @brief Sends a service request and waits for a response.
 *
 * Must call [node_service_has_provider()](file://d:\Development\Source\Amw\Core\Node\NodeApi.h#L105-L105) before calling this function.
 *
 * @param nodeName Name of the node that owns the service.
 * @param pService Pointer to the service object.
 * @param baseServicePacket Pointer to the request message object.
 * @param timeout Maximum time to wait for a response in milliseconds.
 * @return Status code (0 for success).
 */
NODE_API int node_service_call(const char *nodeName, const pService pService, void *baseServicePacket,
                              unsigned int timeout = 1000);

/**
 * @brief Sends a raw service request and receives a raw response.
 *
 * Must call [node_service_has_provider()](file://d:\Development\Source\Amw\Core\Node\NodeApi.h#L105-L105) before calling this function.
 *
 * @param nodeName Name of the node that owns the service.
 * @param pService Pointer to the service object.
 * @param reqData Pointer to serialized request data.
 * @param reqLen Length of the request data in bytes.
 * @param resData Output parameter: pointer to allocated response data buffer.
 * @param resLen Output parameter: length of the response data in bytes.
 * @param timeout Maximum time to wait for a response in milliseconds.
 * @return Status code (0 for success).
 */
NODE_API int node_service_call_native(const char *nodeName, const pService pService, const void *reqData,
                                      size_t reqLen, void **resData, size_t *resLen,
                                      unsigned int timeout = 1000);

/**
 * @brief C-compatible entry point for Python to invoke a service request.
 *
 * @param nodeName Name of the node that owns the service.
 * @param serviceName Name of the service to call.
 * @param req Pointer to serialized request data.
 * @param reqLen Length of the request data in bytes.
 * @param timeout Maximum time to wait for a response in milliseconds.
 * @param res Output parameter: pointer to allocated response data buffer.
 * @param resLen Output parameter: length of the response data in bytes.
 * @return Status code (0 for success).
 */
NODE_API int serviceCall(const char *nodeName, const char *serviceName, const void *req, size_t reqLen,
                         size_t timeout, void **res, size_t *resLen);
