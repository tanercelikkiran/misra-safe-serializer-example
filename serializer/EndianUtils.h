#pragma once
#ifndef ENDIAN_UTILS_HPP
#define ENDIAN_UTILS_HPP

#include <algorithm>
#include <type_traits>
#include <cstring>
#include <climits>

namespace EndianUtils {

    // Endian Türleri
    enum class Endian {
        Little,
        Big,
        Native =
#if defined(_MSC_VER)
        // Windows genellikle Little Endian'dýr (x86/x64/ARM)
        Little
#elif defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
    __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        Little
#elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && \
    __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        Big
#else
        // Fallback: Eðer makrolar bulunamazsa varsayýlan olarak Little kabul et 
        // veya runtime kontrolü eklenebilir. Çoðu modern CPU Little'dýr.
        Little
#endif
    };

    /**
     * @brief Bir verinin baytlarýný ters çevirir (Generic implementation).
     * Derleyici bunu genellikle iþlemciye özgü 'BSWAP' komutuna optimize eder.
     */
    template <typename T>
    constexpr T byteswap(T value) {
        // Sadece trivially copyable (basit kopyalanabilir) türler için çalýþmalýdýr.
        // C++17: std::is_trivially_copyable_v
        static_assert(std::is_trivially_copyable_v<T>, "Endian cevrimi sadece 'trivially copyable' turler icin yapilabilir.");

        // Tek baytlýk verilerde (char, bool, uint8_t) iþlem yapmaya gerek yok.
        if constexpr (sizeof(T) == 1) {
            return value;
        }
        else {
            // Nesnenin bayt temsili üzerinde iþlem yapýyoruz.
            unsigned char* ptr = reinterpret_cast<unsigned char*>(&value);
            std::reverse(ptr, ptr + sizeof(T));
            return value;
        }
    }

    /**
     * @brief Veriyi Native formattan Big Endian'a çevirir.
     */
    template <typename T>
    constexpr T to_big_endian(T value) {
        if constexpr (Endian::Native == Endian::Big) {
            return value; // Sistem zaten Big Endian ise hiçbir þey yapma.
        }
        else {
            return byteswap(value);
        }
    }

    /**
     * @brief Veriyi Native formattan Little Endian'a çevirir.
     */
    template <typename T>
    constexpr T to_little_endian(T value) {
        if constexpr (Endian::Native == Endian::Little) {
            return value; // Sistem zaten Little Endian ise hiçbir þey yapma.
        }
        else {
            return byteswap(value);
        }
    }

    /**
     * @brief Big Endian veriyi Native formata çevirir.
     */
    template <typename T>
    constexpr T from_big_endian(T value) {
        // to_big_endian ile mantýk aynýdýr (tersin tersi kendisidir).
        return to_big_endian(value);
    }

    /**
     * @brief Little Endian veriyi Native formata çevirir.
     */
    template <typename T>
    constexpr T from_little_endian(T value) {
        return to_little_endian(value);
    }
}

#endif // ENDIAN_UTILS_HPP
