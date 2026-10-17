/*
  test EX Bus channel frame decode against the JETI v1.21 PDF example,
  and UART-batch mid-frame timing (must not abort on >1 ms wall gaps).
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_RCProtocol/AP_RCProtocol_EXBUS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_RCPROTOCOL_EXBUS_ENABLED

class RC_Channel_Test : public RC_Channel {};

class RC_Channels_Test : public RC_Channels
{
public:
    RC_Channel_Test obj_channels[NUM_RC_CHANNELS];

    const RC_Channel_Test *channel(const uint8_t chan) const override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }
    RC_Channel_Test *channel(const uint8_t chan) override
    {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:
    int8_t flight_mode_channel_number() const override
    {
        return 5;
    }
};

#define RC_CHANNELS_SUBCLASS RC_Channels_Test
#define RC_CHANNEL_SUBCLASS RC_Channel_Test

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Test rchannels;
static AP_SerialManager serial_manager;

static AP_RCProtocol &frontend()
{
    static AP_RCProtocol *rc = new AP_RCProtocol();
    return *rc;
}

static void make_pdf_example_frame(uint8_t frame[0x28])
{
    // Packet example from EX Bus communication protocol v1.21:
    // 0x3E 0x03 0x28 0x06 0x31 0x20 + 16 x 0x1F82 + CRC 0xE24F (LE)
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
}

TEST(EXBUSDecode, test_exbus_pdf_example_frame)
{
    uint8_t frame[0x28];
    make_pdf_example_frame(frame);

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
    make_pdf_example_frame(frame);
    frame[0x26] = 0x00;
    frame[0x27] = 0x00;

    uint16_t values[MAX_RCIN_CHANNELS] {};
    uint8_t nchan = 0;
    EXPECT_FALSE(AP_RCProtocol_EXBUS::decode_channel_frame(frame, sizeof(frame), values, nchan));
}

/*
  Simulate UART batching: feed half the PDF frame, wait 2 ms (wall clock),
  then the rest. Old 1 ms inter-byte gap aborted here (hdr without ok/crcfail).
 */
TEST(EXBUSDecode, test_exbus_uart_batch_gap_completes)
{
    // standalone backend (frontend only needed for add_input bookkeeping)
    AP_RCProtocol_EXBUS &exbus = *new AP_RCProtocol_EXBUS(frontend());

    uint8_t frame[0x28];
    make_pdf_example_frame(frame);

    const uint32_t baud = 125000;
    uint32_t t = 1000000;
    for (uint8_t i = 0; i < 20; i++) {
        exbus.process_byte_ts(frame[i], baud, t);
        t += 80; // ~125 kbit byte spacing
    }
    // scheduler pause between UART drains (> old 1 ms inter-byte limit)
    t += 2000;
    for (uint8_t i = 20; i < 0x28; i++) {
        exbus.process_byte_ts(frame[i], baud, t);
        t += 80;
    }

    EXPECT_TRUE(exbus.new_input());
    EXPECT_EQ(exbus.num_channels(), 16);
    EXPECT_EQ(exbus.read(0), 1008);
}

#endif  // AP_RCPROTOCOL_EXBUS_ENABLED

AP_GTEST_MAIN()
