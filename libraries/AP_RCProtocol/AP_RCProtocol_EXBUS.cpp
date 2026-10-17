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
#if AP_RCPROTOCOL_EXBUS_DEBUG
#include <GCS_MAVLink/GCS.h>
#endif

extern const AP_HAL::HAL& hal;

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
#if AP_RCPROTOCOL_EXBUS_DEBUG
    _bytes_in++;
#endif

    /*
      Do NOT use a 1 ms inter-byte gap (Betaflight ISR style) on the UART path.
      check_added_uart() drains the ring buffer in scheduler batches and stamps
      each byte with AP_HAL::micros() at read time. A channel frame is ~3.2 ms
      @ 125 kbit, so a >1 ms pause between batches was aborting mid-frame.
      Use a whole-frame timeout from the first byte instead.
     */
    if (byte_input.ofs > 0 &&
        (timestamp_us - byte_input.start_us >= EXBUS_FRAME_TIMEOUT_US)) {
        byte_input.ofs = 0;
#if AP_RCPROTOCOL_EXBUS_DEBUG
        _frame_timeout++;
#endif
    }

    if (byte_input.ofs == 0) {
        if (b != EXBUS_HEAD_CHANNEL && b != EXBUS_HEAD_REQUEST) {
            return;
        }
#if AP_RCPROTOCOL_EXBUS_DEBUG
        _headers++;
#endif
        byte_input.start_us = timestamp_us;
    } else if (byte_input.ofs == 1) {
        if (b != EXBUS_REQ_REPLY && b != EXBUS_REQ_NO_REPLY) {
            byte_input.ofs = 0;
#if AP_RCPROTOCOL_EXBUS_DEBUG
            _drop++;
#endif
            // try this byte as a new header
            if (b == EXBUS_HEAD_CHANNEL || b == EXBUS_HEAD_REQUEST) {
#if AP_RCPROTOCOL_EXBUS_DEBUG
                _headers++;
#endif
                byte_input.start_us = timestamp_us;
                byte_input.buf[byte_input.ofs++] = b;
            }
            return;
        }
    } else if (byte_input.ofs == 2) {
        if (b < EXBUS_MIN_FRAME_LEN || b > EXBUS_MAX_FRAME_LEN) {
            byte_input.ofs = 0;
#if AP_RCPROTOCOL_EXBUS_DEBUG
            _drop++;
#endif
            return;
        }
    }

    if (byte_input.ofs >= EXBUS_MAX_FRAME_LEN) {
        byte_input.ofs = 0;
#if AP_RCPROTOCOL_EXBUS_DEBUG
        _drop++;
#endif
        return;
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
    if (byte_input.buf[0] == EXBUS_HEAD_CHANNEL) {
        const uint16_t crc_calc = crc16_ccitt_r(byte_input.buf, frame_len - EXBUS_CRC_LEN, 0, 0);
        const uint16_t crc_rx = uint16_t(byte_input.buf[frame_len - 2]) | (uint16_t(byte_input.buf[frame_len - 1]) << 8);
        if (crc_calc != crc_rx) {
#if AP_RCPROTOCOL_EXBUS_DEBUG
            _crc_fail++;
#endif
        } else if (decode_channel_frame(byte_input.buf, frame_len, values, nchan)) {
#if AP_RCPROTOCOL_EXBUS_DEBUG
            _frames_ok++;
#endif
            // EX Bus has no failsafe flag; Master stops sending on RF loss
            add_input(nchan, values, false);
#if AP_RCPROTOCOL_EXBUS_DEBUG
        } else {
            _drop++;
#endif
        }
    }

    byte_input.ofs = 0;
}

void AP_RCProtocol_EXBUS::process_byte(uint8_t b, uint32_t baudrate)
{
    process_byte_ts(b, baudrate, AP_HAL::micros());
}

void AP_RCProtocol_EXBUS::process_byte_ts(uint8_t b, uint32_t baudrate, uint32_t timestamp_us)
{
    if (baudrate != 125000 && baudrate != 250000) {
        return;
    }
#if AP_RCPROTOCOL_EXBUS_DEBUG
    _last_baud = baudrate;
#endif
    _process_byte(timestamp_us, b);
}

#if AP_RCPROTOCOL_EXBUS_DEBUG
void AP_RCProtocol_EXBUS::update(void)
{
#if HAL_GCS_ENABLED
    if (hal.util->get_soft_armed()) {
        return;
    }
    if (!protocol_enabled(AP_RCProtocol::EXBUS) || !have_UART()) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (now - _last_debug_ms < 3000) {
        return;
    }
    _last_debug_ms = now;
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,
                  "EXBUS b=%lu h=%lu ok=%lu crc=%lu to=%lu",
                  (unsigned long)_bytes_in,
                  (unsigned long)_headers,
                  (unsigned long)_frames_ok,
                  (unsigned long)_crc_fail,
                  (unsigned long)_frame_timeout);
#endif
}
#endif  // AP_RCPROTOCOL_EXBUS_DEBUG

#endif  // AP_RCPROTOCOL_EXBUS_ENABLED
