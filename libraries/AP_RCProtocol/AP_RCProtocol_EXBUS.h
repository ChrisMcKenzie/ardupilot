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

// Whole-frame collect timeout (not inter-byte). 64 B @ 125 kbit ≈ 5.1 ms;
// allow scheduler batching across several RC ticks.
#define EXBUS_FRAME_TIMEOUT_US 10000U

#define EXBUS_HEAD_CHANNEL  0x3E
#define EXBUS_HEAD_REQUEST  0x3D
#define EXBUS_REQ_REPLY     0x01
#define EXBUS_REQ_NO_REPLY  0x03
#define EXBUS_DATA_CHANNELS 0x31

class AP_RCProtocol_EXBUS : public AP_RCProtocol_Backend
{
public:
    using AP_RCProtocol_Backend::AP_RCProtocol_Backend;

    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void process_byte(uint8_t byte, uint32_t baudrate) override;
#if AP_RCPROTOCOL_EXBUS_DEBUG
    void update(void) override;
#endif

    // decode a complete channel frame; returns true on valid CRC and channel block
    static bool decode_channel_frame(const uint8_t *frame, uint8_t frame_len,
                                     uint16_t *values, uint8_t &nchan);

    // feed one byte with an explicit timestamp (SITL/tests; UART path uses micros())
    void process_byte_ts(uint8_t byte, uint32_t baudrate, uint32_t timestamp_us);

private:
    void _process_byte(uint32_t timestamp_us, uint8_t byte);

    SoftSerial ss{125000, SoftSerial::SERIAL_CONFIG_8N1};

    struct {
        uint8_t buf[EXBUS_MAX_FRAME_LEN];
        uint8_t ofs;
        uint32_t start_us; // timestamp of first byte of current frame
    } byte_input;

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
