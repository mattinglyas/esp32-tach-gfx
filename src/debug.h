#ifndef _debug_h
#define _debug_h

#include <Arduino.h>
#define DEBUG 1
// see https://stackoverflow.com/questions/1644868/define-macro-for-debug-printing-in-c
// TL;DR compliler should always see variables for refactoring, do {} while 0
// causes macro to create a new frame like a function call
#define debug_println(...)               \
    do                                   \
    {                                    \
        if (DEBUG)                       \
            Serial.println(__VA_ARGS__); \
    } while (0);
#define debug_print(...)               \
    do                                 \
    {                                  \
        if (DEBUG)                     \
            Serial.print(__VA_ARGS__); \
    } while (0);

#endif