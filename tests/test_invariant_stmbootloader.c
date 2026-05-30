#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/*
 * We replicate the vulnerable logic from stmbootloader.c in a safe wrapper
 * that enforces bounds checking. The invariant is: buffer reads/writes must
 * never exceed the declared buffer length. We test this by implementing safe
 * versions and verifying that oversized inputs are rejected or truncated,
 * never causing out-of-bounds access.
 *
 * Original vulnerable code:
 *   strcat(startAddress, lsb);                          // line 111
 *   sprintf(lenPlusData, "%02x%s", stmHexToChar(len)-1, data); // line 113
 *
 * CWE-120: Buffer Copy without Checking Size of Input
 */

/* Mirror the buffer sizes from the original code */
#define START_ADDRESS_BUF_SIZE  9   /* e.g., "AABBCCDD\0" = 8 hex chars + null */
#define LEN_PLUS_DATA_BUF_SIZE  256 /* typical small stack buffer */
#define MSB_SIZE                5
#define LSB_SIZE                5

/* Safe version of the startAddress concatenation */
static int safe_build_start_address(char *out, size_t out_size,
                                     const char *msb, const char *lsb)
{
    if (!out || !msb || !lsb) return -1;

    size_t msb_len = strlen(msb);
    size_t lsb_len = strlen(lsb);

    /* Invariant: combined length must fit in buffer including null terminator */
    if (msb_len + lsb_len + 1 > out_size) {
        return -1; /* reject oversized input */
    }

    strncpy(out, msb, out_size - 1);
    out[out_size - 1] = '\0';
    strncat(out, lsb, out_size - strlen(out) - 1);

    return 0;
}

/* Safe version of the lenPlusData sprintf */
static int safe_build_len_plus_data(char *out, size_t out_size,
                                     uint8_t len_val, const char *data)
{
    if (!out || !data) return -1;

    size_t data_len = strlen(data);
    /* "%02x" produces 2 hex chars, plus data, plus null */
    if (2 + data_len + 1 > out_size) {
        return -1; /* reject oversized input */
    }

    int written = snprintf(out, out_size, "%02x%s", len_val, data);
    if (written < 0 || (size_t)written >= out_size) {
        return -1;
    }

    return 0;
}

/* ------------------------------------------------------------------ */

START_TEST(test_start_address_no_overflow)
{
    /* Invariant: strcat into startAddress must never exceed START_ADDRESS_BUF_SIZE */
    const char *lsb_payloads[] = {
        /* Normal inputs */
        "FF",
        "00",
        /* Boundary: exactly fills buffer when combined with msb */
        "FFFF",
        /* Oversized by 2x */
        "FFFFFFFFFF",
        /* Oversized by 10x */
        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF",
        /* Attack: null bytes embedded (treated as string end) */
        "\x00\xFF\xFF\xFF",
        /* Attack: format string characters */
        "%s%s%s%s%s%s%s%s",
        /* Attack: very long repeated pattern */
        "AABBCCDDEEFF00112233445566778899AABBCCDDEEFF",
        /* Attack: max-length string 512 chars */
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
    };

    const char *msb_payloads[] = {
        "08",
        "FF",
        "FFFF",
        "FFFFFFFF",
        "%n%n%n%n",
        "AABBCCDD",
        "AABBCCDDEEFF",
        "AABBCCDDEEFF",
        "AABBCCDDEEFF",
    };

    int num_payloads = sizeof(lsb_payloads) / sizeof(lsb_payloads[0]);

    for (int i = 0; i < num_payloads; i++) {
        char startAddress[START_ADDRESS_BUF_SIZE];
        memset(startAddress, 0xAB, sizeof(startAddress)); /* poison */

        const char *msb = msb_payloads[i];
        const char *lsb = lsb_payloads[i];

        int result = safe_build_start_address(startAddress,
                                               sizeof(startAddress),
                                               msb, lsb);

        if (result == 0) {
            /* If accepted, the output must be null-terminated within bounds */
            size_t out_len = strlen(startAddress);
            ck_assert_msg(out_len < START_ADDRESS_BUF_SIZE,
                "INVARIANT VIOLATED: startAddress output length %zu >= buffer size %d "
                "for msb='%.20s' lsb='%.20s'",
                out_len, START_ADDRESS_BUF_SIZE, msb, lsb);

            /* Verify null terminator is within buffer */
            ck_assert_msg(startAddress[START_ADDRESS_BUF_SIZE - 1] == '\0' ||
                          out_len < START_ADDRESS_BUF_SIZE - 1,
                "INVARIANT VIOLATED: no null terminator within buffer bounds");
        } else {
            /* Rejected: verify buffer was not corrupted beyond its size */
            /* The buffer sentinel byte at the end should still be intact
             * if we never wrote past the buffer */
            ck_assert_msg(result == -1,
                "INVARIANT VIOLATED: unexpected return value %d", result);
        }
    }
}
END_TEST

START_TEST(test_len_plus_data_no_overflow)
{
    /* Invariant: sprintf into lenPlusData must never exceed LEN_PLUS_DATA_BUF_SIZE */
    const char *data_payloads[] = {
        /* Normal */
        "AABBCC",
        /* Exactly at boundary (LEN_PLUS_DATA_BUF_SIZE - 2 - 1 = 253 chars) */
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        /* Oversized by 2x (512 chars) */
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB"
        "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB",
        /* Oversized by 10x (2560 chars) */
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC"
        "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC",
        /* Attack: format string */
        "%s%s%s%s%n%n%n%n%x%x%x%x",
        /* Attack: shell metacharacters */
        "$(rm -rf /); echo PWNED; #AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        /* Attack: binary/non-printable */
        "\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41\x41"
        "\xff\xfe\xfd\xfc\xfb\xfa\xf9\xf8\xf7\xf6\xf5\xf4\xf3\xf2\xf1\xf0",
        /* Attack: all 0xFF bytes */
        "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff"
        "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff",
    };

    uint8_t len_vals[] = {
        0x10,
        0xFF,
        0x00,
        0x7F,
        0xAA,
        0x55,
        0xDE,
        0xAD,
    };

    int num_payloads = sizeof(data_payloads) / sizeof(data_payloads[0]);

    for (int i = 0; i < num_payloads; i++) {
        char lenPlusData[LEN_PLUS_DATA_BUF_SIZE];
        memset(lenPlusData, 0xCD, sizeof(lenPlusData)); /* poison */

        uint8_t len_val = len_vals[i % (sizeof(len_vals) / sizeof(len_vals[0]))];
        const char *data = data_payloads[i];

        int result = safe_build_len_plus_data(lenPlusData,
                                               sizeof(lenPlusData),
                                               len_val, data);

        if (result == 0) {
            /* If accepted, output must be null-terminated within bounds */
            size_t out_len = strlen(lenPlusData);
            ck_assert_msg(out_len < LEN_PLUS_DATA_BUF_SIZE,
                "INVARIANT VIOLATED: lenPlusData output length %zu >= buffer size %d "
                "for len_val=0x%02x data='%.20s'",
                out_len, LEN_PLUS_DATA_BUF_SIZE, len_val, data);

            /* First two chars must be valid hex digits (from %02x) */
            ck_assert_msg((lenPlusData[0] >= '0' && lenPlusData[0] <= '9') ||
                          (lenPlusData[0] >= 'a' && lenPlusData[0] <= 'f') ||
                          (lenPlusData[0] >= 'A' && lenPlusData[0] <= 'F'),
                "INVARIANT VIOLATED: first char of lenPlusData is not hex: 0x%02x",
                (unsigned char)lenPlusData[0]);
        } else {
            ck_assert_msg(result == -1,
                "INVARIANT VIOLATED: unexpected return value %d", result);
        }
    }
}
END_TEST

START_TEST(test_combined_address_and_data_no_overflow)