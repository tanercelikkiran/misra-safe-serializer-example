#ifndef SAFE_SERIALIZER_H
#define SAFE_SERIALIZER_H

#define TEST_ENV 1

#include <utility>
#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <array>
#include "DebugUtils.h"


template <typename To, typename From>
To safe_bit_cast(const From& src) {
    static_assert(sizeof(To) == sizeof(From), "Size mismatch in bit_cast");
    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}

// ENDIANNESS SWAP
template <typename T>
constexpr T swap_bytes_int(T val) {
    if constexpr (sizeof(T) == 1) return val;
    else if constexpr (sizeof(T) == 2) {
        return static_cast<T>((val << 8) | (val >> 8));
    }
    else if constexpr (sizeof(T) == 4) {
        return static_cast<T>(
            ((val & 0xFF000000) >> 24) | ((val & 0x00FF0000) >> 8) |
            ((val & 0x0000FF00) << 8) | ((val & 0x000000FF) << 24)
            );
    }
    else if constexpr (sizeof(T) == 8) {
        return static_cast<T>(
            ((val & 0xFF00000000000000ULL) >> 56) | ((val & 0x00FF000000000000ULL) >> 40) |
            ((val & 0x0000FF0000000000ULL) >> 24) | ((val & 0x000000FF00000000ULL) >> 8) |
            ((val & 0x00000000FF000000ULL) << 8) | ((val & 0x0000000000FF0000ULL) << 24) |
            ((val & 0x000000000000FF00ULL) << 40) | ((val & 0x00000000000000FFULL) << 56)
            );
    }
    return val;
}

template <typename T>
T safe_ntoh(T val) {
    if constexpr (std::is_floating_point_v<T>) {
        if constexpr (sizeof(T) == 4) {
            uint32_t temp = safe_bit_cast<uint32_t>(val);
            temp = swap_bytes_int(temp);
            return safe_bit_cast<T>(temp);
        }
        else if constexpr (sizeof(T) == 8) {
            uint64_t temp = safe_bit_cast<uint64_t>(val);
            temp = swap_bytes_int(temp);
            return safe_bit_cast<T>(temp);
        }
    }
    else {
        if constexpr (sizeof(T) == 1) return val;
        else if constexpr (sizeof(T) == 2 || sizeof(T) == 4 || sizeof(T) == 8) {
            using IntType = std::conditional_t<sizeof(T) == 2, uint16_t,
                std::conditional_t<sizeof(T) == 4, uint32_t, uint64_t>>;

            IntType temp = static_cast<IntType>(val);
            temp = swap_bytes_int(temp);
            return static_cast<T>(temp);
        }
    }
    return val;
}

template <typename T>
T safe_hton(T val) {
    return safe_ntoh(val);
}

template <typename T>
void safe_read_from_buffer(T& dest, const uint8_t* src) {
    std::copy(src, src + sizeof(T), reinterpret_cast<uint8_t*>(&dest));
}

template <typename T>
void safe_write_to_buffer(uint8_t* dest, const T& src) {
    const uint8_t* src_ptr = reinterpret_cast<const uint8_t*>(&src);
    std::copy(src_ptr, src_ptr + sizeof(T), dest);
}

// ==========================================
// DESERIALIZATION TRAITS & ENGINE
// ==========================================

// SFINAE Check
template <typename T, typename = void>
struct has_deserialize : std::false_type {};
template <typename T>
struct has_deserialize < T, std::void_t<decltype(std::declval<T>().deserialize(std::declval<const uint8_t*>(), size_t{}, std::declval<size_t&>())) >> : std::true_type {};

template <typename... Args>
bool deserialize_from_buffer(const uint8_t* buffer, size_t buffer_len, size_t& offset, Args&... args) {
    bool global_success = true;
    int field_index = 0;

    auto process_field = [&](auto& field) {
        if (!global_success) return;
        field_index++;
        using T = std::decay_t<decltype(field)>;

#ifdef TEST_ENV
        std::printf(COLOR_YELLOW "  [DESER] [Field %02d]" COLOR_RESET " Offset: %-4zu Type: %-15s",
            field_index, offset, typeid(T).name());
#endif

        if constexpr (has_deserialize<T>::value) {
#ifdef TEST_ENV
            std::printf("\n" COLOR_CYAN "    >>> Enter Nested (Deser) >>>" COLOR_RESET "\n");
#endif
            size_t sub_consumed = 0;
            if (offset >= buffer_len) {
                global_success = false; return;
            }
            bool sub_result = field.deserialize(buffer + offset, buffer_len - offset, sub_consumed);
            if (!sub_result) global_success = false;
            else {
                offset += sub_consumed;
#ifdef TEST_ENV
                std::printf(COLOR_CYAN "    <<< Exit Nested (Deser) <<<" COLOR_RESET "\n");
#endif
            }
        }
        else {
            size_t needed = sizeof(T);
            if (offset + needed > buffer_len) {
                global_success = false;
                LOG_ERROR("Buffer Underrun!"); return;
            }
            safe_read_from_buffer(field, buffer + offset);
            field = safe_ntoh(field); // Endianness swap
#ifdef TEST_ENV
            std::printf(" Val: "); print_debug_value(field); std::printf("\n");
#endif
            offset += needed;
        }
        };
    (process_field(args), ...);
    return global_success;
}

// ==========================================
// SERIALIZATION TRAITS & ENGINE
// ==========================================

template <typename T, typename = void>
struct has_serialize : std::false_type {};
template <typename T>
struct has_serialize < T, std::void_t<decltype(std::declval<const T>().serialize(std::declval<uint8_t*>(), size_t{}, std::declval<size_t&>())) >> : std::true_type {};

template <typename... Args>
bool serialize_to_buffer(uint8_t* buffer, size_t buffer_len, size_t& offset, const Args&... args) {
    bool global_success = true;
    int field_index = 0;

    auto process_field = [&](const auto& field) {
        if (!global_success) return;
        field_index++;
        using T = std::decay_t<decltype(field)>;

#ifdef TEST_ENV
        std::printf(COLOR_YELLOW "  [SER]   [Field %02d]" COLOR_RESET " Offset: %-4zu Type: %-15s",
            field_index, offset, typeid(T).name());
        std::printf(" Val: "); print_debug_value(field); std::printf("\n");
#endif

        // DURUM 1: Nested Struct (Serialize)
        if constexpr (has_serialize<T>::value) {
#ifdef TEST_ENV
            std::printf(COLOR_CYAN "    >>> Enter Nested (Ser) >>>" COLOR_RESET "\n");
#endif
            size_t sub_consumed = 0;
            if (offset >= buffer_len) {
                global_success = false; return;
            }
            bool sub_result = field.serialize(buffer + offset, buffer_len - offset, sub_consumed);
            if (!sub_result) {
                global_success = false;
                LOG_ERROR("Nested serialization failed field %d", field_index);
            }
            else {
                offset += sub_consumed;
#ifdef TEST_ENV
                std::printf(COLOR_CYAN "    <<< Exit Nested (Ser) <<<" COLOR_RESET "\n");
#endif
            }
        }
        else {
            size_t needed = sizeof(T);
            if (offset + needed > buffer_len) {
                global_success = false;
                LOG_ERROR("Buffer Overflow! Need %zu, Has %zu", needed, buffer_len - offset);
                return;
            }

            // 1. Host to Network (Endian Swap)
            T net_val = safe_hton(field);

			// 2.Write to Buffer
            safe_write_to_buffer(buffer + offset, net_val);

            offset += needed;
        }
        };
    (process_field(args), ...); // Fold expression
    return global_success;
}

// ==========================================
// SIZE CALCULATION ENGINE (TrueSize)
// ==========================================

// SFINAE Check
template <typename T, typename = void>
struct has_trueSize : std::false_type {};
template <typename T>
struct has_trueSize<T, std::void_t<decltype(std::declval<const T>().trueSize())>> : std::true_type {};

template <typename... Args>
constexpr size_t calculate_packed_size(const Args&... args) {
    size_t total_size = 0;

    auto process_field = [&](const auto& field) {
        using T = std::decay_t<decltype(field)>;

        if constexpr (has_trueSize<T>::value) {
            total_size += field.trueSize();
        }
        else {
            total_size += sizeof(T);
        }
        };

    (process_field(args), ...); // Fold expression
    return total_size;
}

#endif // SAFE_SERIALIZER_H