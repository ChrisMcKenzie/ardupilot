/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * JETI EX Bus RC protocol (Master→Slave channel frames).
 * Spec: EX Bus communication protocol v1.21
 * Optional Slave EX telemetry: JETI Telemetry protocol v1.07 (embedded in EX Bus).
 */

#pragma once

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EXBUS_ENABLED

#include "AP_RCProtocol_Backend.h"
#include "SoftSerial.h"

// Max packet length including header and CRC (LEN field). Enough for 18 channels.
#define EXBUS_MAX_FRAME_LEN 64U
#define EXBUS_MIN_FRAME_LEN 8U
#define EXBUS_HEADER_LEN    6U
#define EXBUS_CRC_LEN       2U
#define EXBUS_OVERHEAD      (EXBUS_HEADER_LEN + EXBUS_CRC_LEN)

// Whole-frame collect timeout (not inter-byte). 64 B @ 125 kbit ≈ 5.1 ms;
// allow scheduler batching across several RC ticks.
#define EXBUS_FRAME_TIMEOUT_US 10000U

#define EXBUS_HEAD_CHANNEL  0x3E
#define EXBUS_HEAD_REQUEST  0x3D
#define EXBUS_HEAD_REPLY    0x3B
#define EXBUS_REQ_REPLY     0x01
#define EXBUS_REQ_NO_REPLY  0x03
#define EXBUS_DATA_CHANNELS 0x31
#define EXBUS_DATA_TELEM    0x3A

// Third-party EX manufacturer ID range is 0xA400–0xA41F (JETI telem PDF).
#define EXBUS_TELEM_MFG_ID  0xA400U
#define EXBUS_TELEM_DEV_ID  0x4150U  // 'AP'

// Master releases bus ≥4 ms; reply before ~3 ms (Betaflight uses 3000 µs).
#define EXBUS_TELEM_REPLY_TIMEOUT_US 3000U

class AP_RCProtocol_EXBUS : public AP_RCProtocol_Backend
{
public:
    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    void update(void) override;

    // decode a complete channel frame; returns true on valid CRC and channel block
    static bool decode_channel_frame(const uint8_t *frame, uint8_t frame_len,
                                     uint16_t *values, uint8_t &nchan);

    // EXTEL CRC8 (PDF poly X^8+X^2+X+1); covers type_len..last data byte
    static uint8_t ex_crc8(const uint8_t *data, uint8_t len);

#if AP_EXBUS_TELEM_ENABLED
    // build a Slave EX Bus telemetry reply into out[]; returns frame length or 0
    static uint8_t build_telem_reply(uint8_t packet_id, uint8_t sensor_step,
                                     uint8_t *out, uint8_t out_max,
                                     uint8_t &next_sensor_step);
#endif

    // feed one byte with an explicit timestamp (SITL/tests; UART path uses micros())
    void process_byte_ts(uint8_t byte, uint32_t baudrate, uint32_t timestamp_us);

private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);
#if AP_EXBUS_TELEM_ENABLED
    void handle_telem_request(const uint8_t *frame, uint8_t frame_len, uint32_t timestamp_us);
    void send_telem_reply(void);
#endif

    SoftSerial ss{125000, SoftSerial::SERIAL_CONFIG_8N1};

    struct {
        uint8_t buf[EXBUS_MAX_FRAME_LEN];
        uint8_t ofs;
        uint32_t start_us; // timestamp of first byte of current frame
    } byte_input;

#if AP_EXBUS_TELEM_ENABLED
    struct {
        bool pending;
        uint8_t packet_id;
        uint32_t request_us;
        uint8_t sensor_step; // 0..desc-1 text, then value ids
    } telem;
#endif

#if AP_RCPROTOCOL_EXBUS_DEBUG
    uint32_t _bytes_in;
    uint32_t _headers;
    uint32_t _frames_ok;
    uint32_t _crc_fail;
    uint32_t _drop;
    uint32_t _frame_timeout;
    uint32_t _last_debug_ms;
    uint32_t _last_baud;
#endif
};

#endif  // AP_RCPROTOCOL_EXBUS_ENABLED
