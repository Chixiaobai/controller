#pragma once

/**
 * @file MsgDef.h
 * @brief Message type definitions for the application framework.
 *
 * This header defines standard message identifiers used throughout the framework
 * for inter-object communication and event handling.
 */

/**
 * @defgroup FrameworkMessages Framework Core Messages
 * @{
 */

/**
 * @brief Timer event message.
 *
 * Sent when a registered timer triggers.
 */
#define OM_TIMER 1

/**
 * @brief Keyboard event message.
 *
 * Sent when a key is pressed or released.
 */
#define OM_KEYEVENT 2

/**
 * @brief Application exit request message.
 *
 * Sent to initiate application shutdown sequence.
 */
#define OM_EXITAPP 3

/**
 * @brief Initialization completion message.
 *
 * Sent when an object has completed its initialization process.
 */
#define OM_INIT 4

/**
 * @brief Base identifier for user-defined messages.
 *
 * Custom messages should start from this value to avoid conflicts with system messages.
 */
#define OM_USER 65535

/** @} */ // End of FrameworkMessages group
