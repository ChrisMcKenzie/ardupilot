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
 * JETI EX Bus RC protocol decoder (+ optional Slave EX telemetry).
 * Spec: EX Bus communication protocol v1.21
 *       JETI Telemetry communication protocol v1.07 (EX payload)
 */

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_EXBUS_ENABLED

#include "AP_RCProtocol_EXBUS.h"
#include <AP_Math/crc.h>
#include <AP_Math/AP_Math.h>
#include <string.h>
#if AP_EXBUS_TELEM_ENABLED
#include <AP_AHRS/AP_AHRS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_GPS/AP_GPS_config.h>
#if AP_GPS_ENABLED
#include <AP_GPS/AP_GPS.h>
#endif
#include <AP_Airspeed/AP_Airspeed_config.h>
#if AP_AIRSPEED_ENABLED
#include <AP_Airspeed/AP_Airspeed.h>
#endif
#endif
#if AP_RCPROTOCOL_EXBUS_DEBUG
#include <GCS_MAVLink/GCS.h>
#endif

extern const AP_HAL::HAL& hal;

/*
  EXTEL CRC8 — PDF poly X^8+X^2+X+1 (same result as Betaflight jetiexbus).
 */
uint8_t AP_RCProtocol_EXBUS::ex_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = uint8_t((crc << 1) ^ 0x07);
            } else {
                crc = uint8_t(crc << 1);
            }
        }
    }
    return crc;
}

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

    log_data(AP_RCProtocol::EXBUS, timestamp_us, byte_input.buf, frame_len);

    const uint16_t crc_calc = crc16_ccitt_r(byte_input.buf, frame_len - EXBUS_CRC_LEN, 0, 0);
    const uint16_t crc_rx = uint16_t(byte_input.buf[frame_len - 2]) | (uint16_t(byte_input.buf[frame_len - 1]) << 8);
    const bool crc_ok = (crc_calc == crc_rx);

    if (byte_input.buf[0] == EXBUS_HEAD_CHANNEL) {
        if (!crc_ok) {
#if AP_RCPROTOCOL_EXBUS_DEBUG
            _crc_fail++;
#endif
        } else {
            uint16_t values[MAX_RCIN_CHANNELS];
            uint8_t nchan = 0;
            if (decode_channel_frame(byte_input.buf, frame_len, values, nchan)) {
#if AP_RCPROTOCOL_EXBUS_DEBUG
                _frames_ok++;
#endif
                add_input(nchan, values, false);
#if AP_RCPROTOCOL_EXBUS_DEBUG
            } else {
                _drop++;
#endif
            }
        }
#if AP_EXBUS_TELEM_ENABLED
    } else if (byte_input.buf[0] == EXBUS_HEAD_REQUEST && crc_ok) {
        handle_telem_request(byte_input.buf, frame_len, timestamp_us);
#endif
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

#if AP_EXBUS_TELEM_ENABLED

namespace {

enum class ExType : uint8_t {
    INT22 = 4,
    DES = 255,
};

struct ExSensor {
    const char *label;
    const char *unit;
    ExType type;
    uint8_t decimals; // bits 5..6 of last value byte (JETI EX)
};

/*
  One EX device, ids 0..14 (JETI allows 15 value ids per device).
  Id 0 = device name. Values cycle 1..N-1; several int22s packed per data frame.
 */
const ExSensor k_sensors[] = {
    { "ArduPilot",  "",    ExType::DES,   0 }, // 0 device
    { "Voltage",    "V",   ExType::INT22, 1 }, // 1
    { "Current",    "A",   ExType::INT22, 2 }, // 2
    { "Capacity",   "mAh", ExType::INT22, 0 }, // 3
    { "Power",      "W",   ExType::INT22, 1 }, // 4
    { "Altitude",   "m",   ExType::INT22, 1 }, // 5
    { "Vario",      "m/s", ExType::INT22, 2 }, // 6
    { "Roll",       "\xB0", ExType::INT22, 1 }, // 7
    { "Pitch",      "\xB0", ExType::INT22, 1 }, // 8
    { "Heading",    "\xB0", ExType::INT22, 1 }, // 9
    { "Airspeed",   "m/s", ExType::INT22, 1 }, // 10
    { "GPS Sats",   "",    ExType::INT22, 0 }, // 11
    { "GPS Speed",  "m/s", ExType::INT22, 1 }, // 12
    { "Home Dist",  "m",   ExType::INT22, 0 }, // 13
    { "Remaining",  "%",   ExType::INT22, 0 }, // 14
};

constexpr uint8_t k_sensor_count = ARRAY_SIZE(k_sensors);
constexpr uint8_t k_desc_rounds = k_sensor_count;
// EXTEL data payload budget after sync+type_len+serial+res (Betaflight ~18 B)
constexpr uint8_t k_extel_data_payload_max = 18;

uint8_t decimal_mask(uint8_t decimals)
{
    return uint8_t(decimals << 5);
}

// Pack signed int22 + decimal nibble into 3 LE bytes (JETI / Betaflight layout)
void pack_int22(uint8_t *p, int32_t val, uint8_t decimals)
{
    p[0] = uint8_t(val & 0xFF);
    p[1] = uint8_t((val >> 8) & 0xFF);
    p[2] = uint8_t(((val >> 16) & 0x1F) | decimal_mask(decimals));
    if (val < 0) {
        p[2] |= 0x80;
    }
}

int32_t sensor_value(uint8_t id)
{
    // Singletons may be absent in unit tests — never crash the telem path
    AP_AHRS *ahrs = AP_AHRS::get_singleton();
    AP_BattMonitor *battery = AP_BattMonitor::get_singleton();

    switch (id) {
    case 1: // Voltage * 10
        if (battery == nullptr) {
            return 0;
        }
        return int32_t(roundf(battery->voltage() * 10.0f));
    case 2: { // Current * 100
        if (battery == nullptr) {
            return 0;
        }
        float amps = 0;
        if (!battery->current_amps(amps)) {
            return 0;
        }
        return int32_t(roundf(amps * 100.0f));
    }
    case 3: { // Capacity mAh
        if (battery == nullptr) {
            return 0;
        }
        float mah = 0;
        if (!battery->consumed_mah(mah)) {
            return 0;
        }
        return int32_t(roundf(mah));
    }
    case 4: { // Power W * 10
        if (battery == nullptr) {
            return 0;
        }
        float amps = 0;
        if (!battery->current_amps(amps)) {
            return 0;
        }
        return int32_t(roundf(battery->voltage() * amps * 10.0f));
    }
    case 5: { // Altitude m * 10
        if (ahrs == nullptr) {
            return 0;
        }
        float alt = 0;
        ahrs->get_relative_position_D_home(alt);
        return int32_t(roundf((-alt) * 10.0f));
    }
    case 6: { // Vario m/s * 100
        if (ahrs == nullptr) {
            return 0;
        }
        Vector3f vel;
        if (!ahrs->get_velocity_NED(vel)) {
            return 0;
        }
        return int32_t(roundf((-vel.z) * 100.0f));
    }
    case 7: // Roll deg * 10
        if (ahrs == nullptr) {
            return 0;
        }
        return int32_t(roundf(degrees(ahrs->get_roll()) * 10.0f));
    case 8: // Pitch deg * 10
        if (ahrs == nullptr) {
            return 0;
        }
        return int32_t(roundf(degrees(ahrs->get_pitch()) * 10.0f));
    case 9: { // Heading deg * 10 (0..3600)
        if (ahrs == nullptr) {
            return 0;
        }
        float yaw_deg = degrees(ahrs->get_yaw());
        if (yaw_deg < 0) {
            yaw_deg += 360.0f;
        }
        return int32_t(roundf(yaw_deg * 10.0f));
    }
    case 10: { // Airspeed m/s * 10 (fallback groundspeed)
        float spd = 0;
#if AP_AIRSPEED_ENABLED
        const AP_Airspeed *as = AP_Airspeed::get_singleton();
        if (as != nullptr && as->healthy()) {
            spd = as->get_airspeed();
        } else
#endif
        if (ahrs != nullptr) {
            spd = ahrs->groundspeed();
        }
        return int32_t(roundf(spd * 10.0f));
    }
    case 11: // GPS satellites
#if AP_GPS_ENABLED
        if (AP_GPS::get_singleton() != nullptr) {
            return int32_t(AP::gps().num_sats());
        }
#endif
        return 0;
    case 12: // GPS speed m/s * 10
#if AP_GPS_ENABLED
        if (AP_GPS::get_singleton() != nullptr) {
            return int32_t(roundf(AP::gps().ground_speed() * 10.0f));
        }
#endif
        if (ahrs != nullptr) {
            return int32_t(roundf(ahrs->groundspeed() * 10.0f));
        }
        return 0;
    case 13: { // Home distance m
        if (ahrs == nullptr) {
            return 0;
        }
        Vector2f ne;
        if (!ahrs->get_relative_position_NE_home(ne)) {
            return 0;
        }
        return int32_t(roundf(ne.length()));
    }
    case 14: { // Remaining %
        if (battery == nullptr) {
            return 0;
        }
        uint8_t pct = 0;
        if (!battery->capacity_remaining_pct(pct)) {
            return 0;
        }
        return int32_t(pct);
    }
    default:
        return 0;
    }
}

} // namespace

void AP_RCProtocol_EXBUS::handle_telem_request(const uint8_t *frame, uint8_t frame_len, uint32_t timestamp_us)
{
    if (frame_len < EXBUS_MIN_FRAME_LEN) {
        return;
    }
    if (frame[1] != EXBUS_REQ_REPLY) {
        return;
    }
    if (frame[4] != EXBUS_DATA_TELEM) {
        return; // Jetibox 0x3B etc. not yet
    }
    telem.pending = true;
    telem.packet_id = frame[3];
    telem.request_us = timestamp_us;
}

uint8_t AP_RCProtocol_EXBUS::build_telem_reply(uint8_t packet_id, uint8_t sensor_step,
        uint8_t *out, uint8_t out_max, uint8_t &next_sensor_step)
{
    if (out_max < 16) {
        return 0;
    }

    out[0] = EXBUS_HEAD_REPLY;
    out[1] = EXBUS_REQ_REPLY;
    out[3] = packet_id;
    out[4] = EXBUS_DATA_TELEM;

    uint8_t ex_buf[40];
    memset(ex_buf, 0, sizeof(ex_buf));
    ex_buf[0] = 0x9F;
    ex_buf[2] = uint8_t(EXBUS_TELEM_MFG_ID & 0xFF);
    ex_buf[3] = uint8_t((EXBUS_TELEM_MFG_ID >> 8) & 0xFF);
    ex_buf[4] = uint8_t(EXBUS_TELEM_DEV_ID & 0xFF);
    ex_buf[5] = uint8_t((EXBUS_TELEM_DEV_ID >> 8) & 0xFF);
    ex_buf[6] = 0x00;

    uint8_t ex_len; // CRC coverage from type_len inclusive
    if (sensor_step < k_desc_rounds) {
        const uint8_t sid = sensor_step;
        const ExSensor &s = k_sensors[sid];
        const uint8_t lab_len = strnlen(s.label, 15);
        const uint8_t unit_len = strnlen(s.unit, 7);
        ex_len = uint8_t(1 + 4 + 1 + 1 + 1 + lab_len + unit_len);
        ex_buf[1] = ex_len; // type=0 text
        ex_buf[7] = sid;
        ex_buf[8] = uint8_t((lab_len << 3) | (unit_len & 0x07));
        memcpy(&ex_buf[9], s.label, lab_len);
        memcpy(&ex_buf[9 + lab_len], s.unit, unit_len);
        next_sensor_step = uint8_t(sensor_step + 1);
    } else {
        // Pack as many int22 values as fit (skip DES id 0)
        uint8_t sid = uint8_t(1 + ((sensor_step - k_desc_rounds) % (k_sensor_count - 1)));
        uint8_t *p = &ex_buf[7];
        uint8_t used = 0;
        const uint8_t start_sid = sid;
        do {
            if (k_sensors[sid].type == ExType::DES) {
                sid = uint8_t(sid + 1);
                if (sid >= k_sensor_count) {
                    sid = 1;
                }
                continue;
            }
            if (used + 4 > k_extel_data_payload_max) {
                break;
            }
            *p++ = uint8_t((sid << 4) | uint8_t(k_sensors[sid].type));
            pack_int22(p, sensor_value(sid), k_sensors[sid].decimals);
            p += 3;
            used = uint8_t(used + 4);
            sid = uint8_t(sid + 1);
            if (sid >= k_sensor_count) {
                sid = 1;
            }
        } while (sid != start_sid && used + 4 <= k_extel_data_payload_max);

        // type_len = bytes from type_len through last data byte
        ex_len = uint8_t(1 + 4 + 1 + used); // type_len + serial4 + res + payload
        ex_buf[1] = uint8_t(0x40 | ex_len); // type=1 data
        next_sensor_step = uint8_t(k_desc_rounds + (sid - 1));
    }

    const uint8_t crc8 = ex_crc8(&ex_buf[1], ex_len);
    ex_buf[1 + ex_len] = crc8;
    const uint8_t ex_total = uint8_t(1 + ex_len + 1);

    const uint8_t sub_len = ex_total;
    const uint8_t frame_len = uint8_t(EXBUS_OVERHEAD + sub_len);
    if (frame_len > out_max) {
        return 0;
    }

    out[2] = frame_len;
    out[5] = sub_len;
    memcpy(&out[6], ex_buf, ex_total);

    const uint16_t crc16 = crc16_ccitt_r(out, frame_len - EXBUS_CRC_LEN, 0, 0);
    out[frame_len - 2] = uint8_t(crc16 & 0xFF);
    out[frame_len - 1] = uint8_t((crc16 >> 8) & 0xFF);
    return frame_len;
}

void AP_RCProtocol_EXBUS::send_telem_reply(void)
{
    if (!telem.pending) {
        return;
    }
    const uint32_t now = AP_HAL::micros();
    if (now - telem.request_us > EXBUS_TELEM_REPLY_TIMEOUT_US) {
        telem.pending = false;
        return;
    }
    AP_HAL::UARTDriver *uart = get_available_UART();
    if (uart == nullptr) {
        telem.pending = false;
        return;
    }
    if (uart->available() > 0) {
        return; // half-duplex: wait for idle line within reply window
    }

    uint8_t frame[EXBUS_MAX_FRAME_LEN];
    uint8_t next_step = 0;
    const uint8_t len = build_telem_reply(telem.packet_id, telem.sensor_step, frame, sizeof(frame), next_step);
    if (len == 0) {
        telem.pending = false;
        return;
    }

    uart->write(frame, len);
    telem.pending = false;
    telem.sensor_step = next_step;
}

#endif  // AP_EXBUS_TELEM_ENABLED

void AP_RCProtocol_EXBUS::update(void)
{
#if AP_EXBUS_TELEM_ENABLED
    send_telem_reply();
#endif
#if AP_RCPROTOCOL_EXBUS_DEBUG
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
#endif
}

#endif  // AP_RCPROTOCOL_EXBUS_ENABLED
