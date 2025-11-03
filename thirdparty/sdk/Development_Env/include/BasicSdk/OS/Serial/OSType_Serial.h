#pragma once

#ifndef _WIN32
#include "termios.h"
#ifndef __APPLE__
#include "linux/serial.h"
#endif
#include "sys/ioctl.h"
#include "aio.h"

// Enumerates the possible parity options for serial communication.
enum PARITY {
    NOPARITY = 0,    // No parity bit is used.
    ODDPARITY = 1,   // Odd parity: sets the parity bit to make the total number of 1 bits odd.
    SPACEPARITY = 2, // Space parity: always sets the parity bit to 0.
    MARKPARITY = 3,  // Mark parity: always sets the parity bit to 1.
    EVENPARITY = 4   // Even parity: sets the parity bit to make the total number of 1 bits even.
};

// Enumerates the possible stop bits configurations for serial communication.
enum STOPBITS {
    ONESTOPBIT = 0,  // Configures the serial port to use one stop bit.
    TWOSTOPBITS = 1  // Configures the serial port to use two stop bits.
};
#endif

// Defines the type alias for the serial handle in the operating system context.
#define OS_SERIAL_HANDLE OS_DEVICE_HANDLE
