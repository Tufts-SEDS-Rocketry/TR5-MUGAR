#pragma once

#include "errors.h"

#include <cstdint>
#include <optional>
#include <string_view>
#include <tuple>

// main reference doc:
// https://drive.google.com/file/d/1kfhYX9w9B1p3TzluX87McfV2YKG-f-gQ/view

namespace seds {
using namespace seds::errors;

// https://forum.arduino.cc/t/nmea-checksums-explained/1046083

struct UTCTime {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint16_t millis; 
};

struct UTCDate {
    uint16_t day;
    uint8_t month;
    uint16_t year;
};

// https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html
struct GNGGAData {
    UTCTime time;
    // ignore lat/long
    // ignore gnss
    // ignore num satellites
    float hdop;
    float altitude;
    // ignore geoidal separation
    // ignore differential time/reference station
};

Expected<GNGGAData> parse_gngga_msg(std::string_view msg);

// https://docs.novatel.com/OEM7/Content/Logs/GPGSA.htm
// https://www.qso.com.ar/datasheets/Receptores%20GNSS-GPS/NMEA_Format_v0.1.pdf
// GPGSA and GLGSA use different satellites, but give the same data
struct GXGSAData {
    // ignore mode
    // ignore prn numbers
    float pdop;
    float hdop;
    float vdop;
    // ignore sys id
};

Expected<GXGSAData> parse_gxgsa_msg(std::string_view msg);

enum class MagDeclinDirection : uint8_t { East, West };

struct MagInfo {
    float mag_declination;
    MagDeclinDirection mag_direction; 
};

// https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm
struct GNRMCData {
    UTCTime time;
    // ignore positioning status
    // ignore lat/long
    float ground_speed;
    float ground_heading;
    UTCDate date;
    std::optional<MagInfo> mag_info; // some messages won't have this field
    // ignore mode
};

Expected<GNRMCData> parse_gnrmc_msg(std::string_view msg);

// https://docs.fixposition.com/fd/nmea-gp-vtg
struct GNVTGData {
    float ground_course_north;
    std::optional<float> ground_course_mag;
    float ground_speed;
    // ignore mode
};

Expected<std::optional<GNVTGData>> parse_gnvtg_msg(std::string_view msg);

// https://docs.novategl.com/OEM7/Content/Logs/GPGSV.htm
// we actually ignore GXGSV data because all it gives
// is information on the satellites themselves

bool check_xor(std::string_view msg);

}