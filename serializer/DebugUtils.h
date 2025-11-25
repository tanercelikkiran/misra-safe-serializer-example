#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <array>
#include <cmath>
#include <cstdio>
#include <typeinfo>

#define TEST_ENV 1

#ifdef TEST_ENV

// Color definitions
#define COLOR_RESET   "\033[0m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"

#define LOG_DEBUG(msg, ...) std::printf(COLOR_CYAN "[DEBUG] " msg COLOR_RESET "\n", ##__VA_ARGS__)
#define LOG_INFO(msg, ...)  std::printf(COLOR_GREEN "[INFO]  " msg COLOR_RESET "\n", ##__VA_ARGS__)
#define LOG_ERROR(msg, ...) std::printf(COLOR_RED "[ERROR] " msg COLOR_RESET "\n", ##__VA_ARGS__)

#else
#define LOG_DEBUG(msg, ...) ((void)0)
#define LOG_INFO(msg, ...)  ((void)0)
#define LOG_ERROR(msg, ...) ((void)0)
#define print_debug_value(v) ((void)0)
#define debug_hex_dump(b, l) ((void)0)
#endif

// ==========================================
// DEBUG UTILITIES
// ==========================================
#ifdef TEST_ENV
template <typename T>
void print_debug_value(const T& val) {
    if constexpr (std::is_floating_point_v<T>) {
        std::printf("%.4f", val);
    }
    else if constexpr (std::is_enum_v<T>) {
        std::printf("ENUM(%u)", static_cast<uint8_t>(val));
    }
    else if constexpr (std::is_same_v<T, bool>) {
        std::printf("%s", val ? "TRUE" : "FALSE");
    }
    else if constexpr (std::is_integral_v<T>) {
        if constexpr (sizeof(T) == 8) std::printf("0x%llX (%llu)", (unsigned long long)val, (unsigned long long)val);
        else if constexpr (sizeof(T) == 4) std::printf("0x%X (%u)", (unsigned int)val, (unsigned int)val);
        else std::printf("%u", (unsigned int)val);
    }
    else {
        std::printf("[Complex Object]");
    }
}

inline void debug_hex_dump(const uint8_t* buffer, size_t len) {
    std::printf("[HEX DUMP] Size: %zu bytes\n", len);
    for (size_t i = 0; i < len; ++i) {
        if (i % 16 == 0) std::printf("\n  %04zX: ", i);
        std::printf("%02X ", buffer[i]);
    }
    std::printf("\n\n");
}
#else
template <typename T> void print_debug_value(const T&) {}
inline void debug_hex_dump(const uint8_t*, size_t) {}
#endif

#endif // !DEBUG_UTILS_H
