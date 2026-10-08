/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <string.h>

extern "C" {
    #include "common/streambuf.h"
    #include "pg/board.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define NAME_MAX_LEN MAX_BOARD_NAME_LENGTH
#define ID_MAX_LEN MAX_MANUFACTURER_ID_LENGTH
#define GUARD_BYTE 0xA5

// Same read sequence as the board-info handler in interface/msp.c, run on the real sbuf functions.
static void parseBoardInfo(sbuf_t *src, char *name, char *id)
{
    uint8_t length = sbufReadU8(src);
    sbufReadData(src, name, length < NAME_MAX_LEN ? length : NAME_MAX_LEN);
    if (length > NAME_MAX_LEN) {
        sbufAdvance(src, length - NAME_MAX_LEN);
        length = NAME_MAX_LEN;
    }
    name[length] = '\0';
    length = sbufReadU8(src);
    sbufReadData(src, id, length < ID_MAX_LEN ? length : ID_MAX_LEN);
    if (length > ID_MAX_LEN) {
        sbufAdvance(src, length - ID_MAX_LEN);
        length = ID_MAX_LEN;
    }
    id[length] = '\0';
}

TEST(StreambufTest, ReadDataAdvancesReadPointer)
{
    uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    sbuf_t sbuf;
    sbufInit(&sbuf, data, data + sizeof(data));

    uint8_t out[3] = {};
    sbufReadData(&sbuf, out, 3);

    EXPECT_EQ(5, sbufBytesRemaining(&sbuf));
    EXPECT_EQ(1, out[0]);
    EXPECT_EQ(3, out[2]);
    EXPECT_EQ(4, sbufReadU8(&sbuf));
}

TEST(StreambufTest, ReadDataZeroLengthLeavesPointer)
{
    uint8_t data[2] = { 9, 8 };
    sbuf_t sbuf;
    sbufInit(&sbuf, data, data + sizeof(data));

    sbufReadData(&sbuf, data, 0);

    EXPECT_EQ(2, sbufBytesRemaining(&sbuf));
}

TEST(StreambufTest, ConsecutiveReadDataCallsDoNotRepeatBytes)
{
    uint8_t data[4] = { 10, 20, 30, 40 };
    sbuf_t sbuf;
    sbufInit(&sbuf, data, data + sizeof(data));

    uint8_t first[2] = {};
    uint8_t second[2] = {};
    sbufReadData(&sbuf, first, 2);
    sbufReadData(&sbuf, second, 2);

    EXPECT_EQ(10, first[0]);
    EXPECT_EQ(30, second[0]);
    EXPECT_EQ(0, sbufBytesRemaining(&sbuf));
}

TEST(StreambufTest, BoardInfoParseKeepsFieldsAlignedForAnyLength)
{
    const uint8_t nameLens[] = { 0, 1, NAME_MAX_LEN - 1, NAME_MAX_LEN, NAME_MAX_LEN + 1, 255 };
    const uint8_t idLens[] = { 0, 1, ID_MAX_LEN - 1, ID_MAX_LEN, ID_MAX_LEN + 1, 255 };

    for (uint8_t nameLen : nameLens) {
        for (uint8_t idLen : idLens) {
            // 1 + 255 + 1 + 255 = 512 bytes fits a 520 byte buffer
            uint8_t payload[520] = {};
            uint8_t *end;
            {
                uint8_t *p = payload;
                *p++ = nameLen;
                memset(p, 'n', nameLen);
                p += nameLen;
                *p++ = idLen;
                memset(p, 'i', idLen);
                p += idLen;
                end = p;
            }

            // sentinel byte after the record proves the parser stops at the record end
            *end = 0x5A;

            // guard bytes sit one past each local buffer's terminator slot
            char nameBuf[NAME_MAX_LEN + 2];
            char idBuf[ID_MAX_LEN + 2];
            memset(nameBuf, GUARD_BYTE, sizeof(nameBuf));
            memset(idBuf, GUARD_BYTE, sizeof(idBuf));

            sbuf_t sbuf;
            sbufInit(&sbuf, payload, end + 1);
            parseBoardInfo(&sbuf, nameBuf, idBuf);

            const size_t expectedName = nameLen < NAME_MAX_LEN ? nameLen : NAME_MAX_LEN;
            const size_t expectedId = idLen < ID_MAX_LEN ? idLen : ID_MAX_LEN;
            EXPECT_EQ(expectedName, strlen(nameBuf)) << "nameLen=" << (int)nameLen;
            EXPECT_EQ(expectedId, strlen(idBuf)) << "idLen=" << (int)idLen;
            EXPECT_EQ((uint8_t)GUARD_BYTE, (uint8_t)nameBuf[NAME_MAX_LEN + 1]);
            EXPECT_EQ((uint8_t)GUARD_BYTE, (uint8_t)idBuf[ID_MAX_LEN + 1]);
            // reader consumed the whole record: only the sentinel remains
            EXPECT_EQ(1, sbufBytesRemaining(&sbuf)) << "nameLen=" << (int)nameLen << " idLen=" << (int)idLen;
            EXPECT_EQ(0x5A, sbufReadU8(&sbuf));
        }
    }
}
