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
 * JETI EX Bus RC protocol decoder (channel frames only).
 * Spec: EX Bus communication protocol v1.21
 */

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EXBUS_ENABLED

#include "AP_RCProtocol_EXBUS.h"
#include <AP_Math/crc.h>

/*
  decode a full EX Bus channel frame.
  Channel values are uint16 little-endian with 1 LSB = 1/8 µs.
 */
bool AP_RCProtocol_EXBUS::decode_channel_frame(const uint8_t *frame, uint8_t frame_len,
        uint16_t *values, uint8_t &nchan)
{
    nchan = 0;

    if (frame_len < EXBUS_MIN_FRAME_LEN || frame_len > EXBUS_MAX_FRAME_LEN) {
        return false;
    }
    if (frame[0] != EXBUS_HEAD_CHANNEL) {
        return false;
    }
    if (frame[1] != EXBUS_REQ_REPLY && frame[1] != EXBUS_REQ_NO_REPLY) {
        return false;
    }
    if (frame[2] != frame_len) {
        return false;
    }
    if (frame[4] != EXBUS_DATA_CHANNELS) {
        return false;
    }

    const uint8_t sub_len = frame[5];
    if ((sub_len & 1) != 0) {
        return false;
    }
    if (uint16_t(EXBUS_HEADER_LEN) + sub_len + EXBUS_CRC_LEN != frame_len) {
        return false;
    }

    const uint16_t crc_calc = crc16_ccitt_r(frame, frame_len - EXBUS_CRC_LEN, 0, 0);
    const uint16_t crc_rx = uint16_t(frame[frame_len - 2]) | (uint16_t(frame[frame_len - 1]) << 8);
    if (crc_calc != crc_rx) {
        return false;
    }

    const uint8_t max_chan = MIN(uint8_t(sub_len / 2), uint8_t(MAX_RCIN_CHANNELS));
    for (uint8_t i = 0; i < max_chan; i++) {
        const uint8_t off = EXBUS_HEADER_LEN + (i * 2);
        const uint16_t raw = uint16_t(frame[off]) | (uint16_t(frame[off + 1]) << 8);
        // 1 LSB = 1/8 µs
        values[i] = raw >> 3;
    }
    nchan = max_chan;
    return nchan > 0;
}

void AP_RCProtocol_EXBUS::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

void AP_RCProtocol_EXBUS::_process_byte(uint32_t timestamp_us, uint8_t b)
{
    // incomplete frame: resync on inter-byte gap (Betaflight uses 1ms)
    if (byte_input.ofs > 0 && (timestamp_us - byte_input.last_byte_us >= 1000U)) {
        byte_input.ofs = 0;
    }
    byte_input.last_byte_us = timestamp_us;

    if (byte_input.ofs == 0) {
        if (b != EXBUS_HEAD_CHANNEL && b != EXBUS_HEAD_REQUEST) {
            return;
        }
    } else if (byte_input.ofs == 1) {
        if (b != EXBUS_REQ_REPLY && b != EXBUS_REQ_NO_REPLY) {
            byte_input.ofs = 0;
            return;
        }
    } else if (byte_input.ofs == 2) {
        if (b < EXBUS_MIN_FRAME_LEN || b > EXBUS_MAX_FRAME_LEN) {
            byte_input.ofs = 0;
            return;
        }
    }

    byte_input.buf[byte_input.ofs++] = b;

    if (byte_input.ofs < 3) {
        return;
    }

    const uint8_t frame_len = byte_input.buf[2];
    if (byte_input.ofs < frame_len) {
        return;
    }

    // full frame collected (channel or request); only channel frames produce RC input
    log_data(AP_RCProtocol::EXBUS, timestamp_us, byte_input.buf, frame_len);

    uint16_t values[MAX_RCIN_CHANNELS];
    uint8_t nchan = 0;
    if (decode_channel_frame(byte_input.buf, frame_len, values, nchan)) {
        // EX Bus has no failsafe flag; Master stops sending on RF loss
        add_input(nchan, values, false);
    }

    byte_input.ofs = 0;
}

void AP_RCProtocol_EXBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    if (baudrate != 125000 && baudrate != 250000) {
        return;
    }
    _process_byte(AP_HAL::micros(), b);
}

#endif  // AP_RCPROTOCOL_EXBUS_ENABLED
