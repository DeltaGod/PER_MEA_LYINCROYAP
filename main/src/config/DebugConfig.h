#pragma once

// Set DEBUG_MODE 1 for verbose event logging on Serial.
// Set to 0 before water deployment — saves ~400 bytes RAM and removes serial latency.
#define DEBUG_MODE 0

#if DEBUG_MODE
// DBG("TAG", "fmt", args...)  →  [DBG:TAG] fmt\n
// Tag must be a string literal.
#  define DBG(tag, fmt, ...) Serial.printf("[DBG:" tag "] " fmt "\n", ##__VA_ARGS__)
#else
#  define DBG(tag, fmt, ...) ((void)0)
#endif
