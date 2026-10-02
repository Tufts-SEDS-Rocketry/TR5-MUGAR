#include "parsing.h"

#include <cmath>
#include <cstdlib>

constexpr float knots_to_ms = 0.514;
constexpr float kmhr_to_ms = 1000.0/(60.0 * 60.0);
#define TAG "GPS Parser"

namespace seds {
using namespace seds::errors;

Expected<UTCTime> parse_utc_time(std::string_view msg) {
    UTCTime time = {
        .hours = 0,
        .minutes = 0,
        .seconds = 0,
        .millis = 0
    };

    for (int i = 0; i < msg.find_first_of(','); i++) {
        if (i == 6) { continue; } // is the '.' 
        if (msg[i] < '0' || msg[i] > '9') {
            return std::unexpected(
                std::make_unique<std::runtime_error>(
                    std::runtime_error("nondigit character encountered while parsing UTCtime")
            ));
        }
    }

    time.hours = 10 * (uint8_t)(msg[0] - '0') + (uint8_t)(msg[1] - '0');
    time.minutes = 10 * (uint8_t)(msg[2] - '0') + (uint8_t)(msg[3] - '0');
    time.seconds = 10 * (uint8_t)(msg[4] - '0') + (uint8_t)(msg[5] - '0');

    for (int i = 7; i < msg.find_first_of(','); i++) {
        time.millis += std::pow(10, (9 - i)) * (uint8_t)(msg[i] - '0');
    }

    return time;
}

Expected<UTCDate> parse_utc_date(std::string_view msg) {
    UTCDate date = {
        .day = 0,
        .month = 0,
        .year = 0
    };

    if (msg.find_first_of(',') != 6) {
        return std::unexpected(
            std::make_unique<std::runtime_error>(
                std::runtime_error("UTC date field not 6 characters long")
        ));
    }

    for (int i = 0; i < 6; i++) {
        if (msg[i] < '0' || msg[i] > '9') {
            return std::unexpected(
                std::make_unique<std::runtime_error>(
                    std::runtime_error("nondigit character encountered while parsing UTCdate")
            ));
        }
    }

    date.day = 10 * (uint8_t)(msg[0] - '0') + (uint8_t)(msg[1] - '0');
    date.month = 10 * (uint8_t)(msg[2] - '0') + (uint8_t)(msg[3] - '0');
    date.year = 2000 + 10 * (uint8_t)(msg[4] - '0') + (uint8_t)(msg[5] - '0'); // remember to update in 75 years!

    return date;
}

Expected<float> parse_float(std::string_view msg, const char* error) {
    size_t sv_end = msg.find_first_of(',');
    if (sv_end == std::string_view::npos) {
        sv_end = msg.find_first_of('*');
    }
    if (sv_end == std::string_view::npos) {
        // message not properly terminated
        ESP_LOGE(TAG, "message not properly terminated when parsing float, msg: %s", msg.data());
        return std::unexpected(
                std::make_unique<std::runtime_error>(
                    std::runtime_error(error)
        )); 
    }

    std::string_view float_sv = std::string_view(msg.data(), sv_end);
    char* end_ptr = nullptr;
    float val = std::strtof(float_sv.data(), &end_ptr);
    if (end_ptr != float_sv.data() + float_sv.size()) {
        // ran into float parsing error before next comma
        ESP_LOGE(TAG, "encountered error parsing float in message %s, float sv: %.*s", 
            msg.data(), sv_end, msg.data()
        );
        return std::unexpected(
                std::make_unique<std::runtime_error>(
                    std::runtime_error(error)
        ));
    }

    return val;
}

Expected<GNGGAData> parse_gngga_msg(std::string_view msg) {
    UTCTime time = TRY(parse_utc_time(msg));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past time
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past lat
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past lat hemi
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past long
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past long hemi
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past gnns pos status
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past num satellites

    float hdop = TRY(parse_float(msg, "encountered error parsing float for hdop in gngga message"));

    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past hdop
    
    float altitude = TRY(parse_float(msg, "encountered error parsing float for altitude in gngga message"));

    GNGGAData data = {
        .time = time,
        .hdop = hdop,
        .altitude = altitude
    };

    return data;
}

Expected<GXGSAData> parse_gxgsa_msg(std::string_view msg) {
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past mode letter
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past mode number
    
    for (int i = 0; i < 12; i++) {
        msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); 
    }

    // TODO: this field appears in the datasheet, but not in their examples, and not in the messages we receive
    //msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past QZSS satellite
    
    float pdop = TRY(parse_float(msg, "encountered error parsing float for pdop in gxgsa message"));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past pdop

    float hdop = TRY(parse_float(msg, "encountered error parsing float for hdop in gxgsa message"));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past hdop

    float vdop = TRY(parse_float(msg, "encountered error parsing float for vdop in gxgsa message"));
    
    GXGSAData data = {
        .pdop = pdop,
        .hdop = hdop,
        .vdop = vdop
    };

    return data;
}

Expected<GNRMCData> parse_gnrmc_msg(std::string_view msg) {
    UTCTime time = TRY(parse_utc_time(msg));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past time
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past position status
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past lat
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past lat direction
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past long
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past long direction

    float ground_speed = TRY(parse_float(msg, "encountered error parsing float for ground speed of GNRMC message")) * knots_to_ms;
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past ground speed

    float ground_heading = TRY(parse_float(msg, "encountered error parsing float for ground heading of GNRMC message"));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past ground heading

    UTCDate date = TRY(parse_utc_date(msg));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past utc date

    std::optional<MagInfo> mag_info = std::nullopt;
    
    // there is actual mag info
    if (msg[0] != ',') {
        float mag_declin = TRY(parse_float(msg, "encountered error parsing float for magnetic declination of GNRMC message"));
        msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past magnetic declination

        MagDeclinDirection mag_direction; 
        if (msg[0] == 'E') {
            mag_direction = MagDeclinDirection::East;
        } else if (msg[0] == 'W') {
            mag_direction = MagDeclinDirection::West;
        } else {
            ESP_LOGE(TAG, "Expected E or W in magnetic declination, instead got %c, full msg is %s", msg[0], msg.data());
            return std::unexpected(
                    std::make_unique<std::runtime_error>(
                        std::runtime_error("encountered error parsing direction for magnetic declination of GNRMC message")
            ));
        }

        mag_info = MagInfo {
            .mag_declination = mag_declin,
            .mag_direction = mag_direction
        };
    }

    

    GNRMCData data = {
        .time = time,
        .ground_speed = ground_speed,
        .ground_heading = ground_heading,
        .date = date,
        .mag_info = mag_info
    };

    return data;
}

Expected<std::optional<GNVTGData>> parse_gnvtg_msg(std::string_view msg) {
    if (msg[msg.find_last_of(',') + 1] == 'N') {
        // data not valid
        return std::nullopt;
    }

    float true_heading = TRY(parse_float(msg, "encountered error parsing float for north heading of GNVTG message"));
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past north heading
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past north reference

    std::optional<float> mag_heading = std::nullopt;
    if (msg.find_first_of(',') != 0) {
        mag_heading = TRY(parse_float(msg, "encountered error parsing float for magnetic heading of GNVTG message"));
    }
     
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past mag heading
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past mag reference

    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past speed in knots
    msg = std::string_view(msg.data() + msg.find_first_of(',') + 1); // skip past knots unit

    float ground_speed = TRY(parse_float(msg, "encountered error parsing float for ground speed of GNVTG message")) * kmhr_to_ms;

    GNVTGData data = {
        .ground_course_north = true_heading,
        .ground_course_mag = mag_heading,
        .ground_speed = ground_speed
    };

    return data;
}

}