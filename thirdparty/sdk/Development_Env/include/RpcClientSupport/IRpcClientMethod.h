#pragma once

#include "Base/IBase.h"
#include "BoundedStream/BoundedStream.h"

/**
 * @interface IRpcRequest
 * @brief Interface for serializing RPC requests into a stream.
 *
 * This interface defines a method to serialize the request data into a bounded stream.
 */
interface IRpcRequest : public IBase {
    /**
     * @brief Serializes the request data into a bounded stream.
     *
     * @param requestStream The bounded stream to write the request data into.
     * @return true if serialization was successful; false otherwise.
     */
    virtual bool toStream(CBoundedStream & requestStream) = 0;
};

/**
 * @interface IRpcResponse
 * @brief Interface for handling RPC responses from a stream.
 *
 * This interface provides methods to process and deserialize response data from a bounded stream,
 * as well as check if the response is empty.
 */
interface IRpcResponse : public IBase {
    /**
     * @brief Processes the deserialized response data.
     *
     * @return true if processing was successful; false otherwise.
     */
    virtual bool processResponse() = 0;

    /**
     * @brief Deserializes response data from a bounded stream.
     *
     * @param responseStream The bounded stream containing the response data.
     * @return true if deserialization was successful; false otherwise.
     */
    virtual bool fromStream(CBoundedStream & responseStream) = 0;

    /**
     * @brief Checks whether the response contains any data.
     *
     * @return true if the response is empty; false otherwise.
     */
    virtual bool isEmpty() = 0;
};

/**
 * @interface IRpcNotify
 * @brief Interface for handling RPC notifications from a stream.
 *
 * This interface provides methods to process and deserialize notification data from a bounded stream.
 */
interface IRpcNotify : public IBase {
    /**
     * @brief Processes the notification after receiving it.
     *
     * @return true if processing was successful; false otherwise.
     */
    virtual bool processNotify() = 0;

    /**
     * @brief Deserializes notification data from a bounded stream.
     *
     * @param notifyStream The bounded stream containing the notification data.
     * @return true if deserialization was successful; false otherwise.
     */
    virtual bool fromStream(CBoundedStream & notifyStream) = 0;
};
