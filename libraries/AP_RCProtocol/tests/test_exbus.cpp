/*
  test EX Bus channel frame decode against the JETI v1.21 PDF example
 */
#include <AP_gtest.h>
#include <AP_RCProtocol/AP_RCProtocol_EXBUS.h>

#if AP_RCPROTOCOL_EXBUS_ENABLED

TEST(EXBUSDecode, test_exbus_pdf_example_frame)
{
    // Packet example from EX Bus communication protocol v1.21:
    // 0x3E 0x03 0x28 0x06 0x31 0x20 + 16 x 0x1F82 + CRC 0xE24F (LE)
    uint8_t frame[0x28];
    frame[0] = 0x3E;
    frame[1] = 0x03;
    frame[2] = 0x28;
    frame[3] = 0x06;
    frame[4] = 0x31;
    frame[5] = 0x20;
    for (uint8_t i = 0; i < 16; i++) {
        frame[6 + (i * 2)] = 0x82;
        frame[7 + (i * 2)] = 0x1F;
    }
    frame[0x26] = 0x4F;
    frame[0x27] = 0xE2;

    uint16_t values[MAX_RCIN_CHANNELS] {};
    uint8_t nchan = 0;
    EXPECT_TRUE(AP_RCProtocol_EXBUS::decode_channel_frame(frame, sizeof(frame), values, nchan));
    EXPECT_EQ(nchan, 16);
    for (uint8_t i = 0; i < 16; i++) {
        // 0x1F82 >> 3 = 1008 µs
        EXPECT_EQ(values[i], 1008);
    }
}

TEST(EXBUSDecode, test_exbus_bad_crc)
{
    uint8_t frame[0x28];
    frame[0] = 0x3E;
    frame[1] = 0x03;
    frame[2] = 0x28;
    frame[3] = 0x06;
    frame[4] = 0x31;
    frame[5] = 0x20;
    for (uint8_t i = 0; i < 16; i++) {
        frame[6 + (i * 2)] = 0x82;
        frame[7 + (i * 2)] = 0x1F;
    }
    frame[0x26] = 0x00;
    frame[0x27] = 0x00;

    uint16_t values[MAX_RCIN_CHANNELS] {};
    uint8_t nchan = 0;
    EXPECT_FALSE(AP_RCProtocol_EXBUS::decode_channel_frame(frame, sizeof(frame), values, nchan));
}

#endif  // AP_RCPROTOCOL_EXBUS_ENABLED

AP_GTEST_MAIN()
int hal = 0;
