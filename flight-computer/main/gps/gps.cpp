#include "gps.h"

#include <cstring>
#include "freertos/task.h"

constexpr uint32_t NO_DATA_TIMEOUT = pdMS_TO_TICKS(10);
constexpr uint32_t DATA_TIMEOUT = pdMS_TO_TICKS(50);

#define TAG "GPS"

namespace seds {

Expected<GPS> GPS::create() {
    PA1616D gps_module = TRY(PA1616D::create());
    gps_module.full_clear_queues();
    GPS gps = GPS(gps_module);
    
    return gps;
}

std::optional<Timestamped<UTCTime>> GPS::get_time() {
    Timestamped<GNGGAData> gga_data = this->gga_data.load();
    Timestamped<GNRMCData> rmc_data = this->rmc_data.load();

    // must be nonzero time, so is valid
    if (gga_data.time > rmc_data.time) {
        return {{gga_data.data.time, gga_data.time}};
    } else {
        if (rmc_data.time == 0) {
            return std::nullopt;
        }
        return {{rmc_data.data.time, rmc_data.time}};
    }
}

std::optional<Timestamped<float>> GPS::get_hdop() {
    Timestamped<GNGGAData> gga_data = this->gga_data.load();
    if (gga_data.time == 0) {
        return std::nullopt;
    }
    return {{gga_data.data.hdop, gga_data.time}};
}

std::optional<Timestamped<float>> GPS::get_altitude() {
    Timestamped<GNGGAData> gga_data = this->gga_data.load();
    if (gga_data.time == 0) {
        return std::nullopt;
    }
    return {{gga_data.data.altitude, gga_data.time}};
}

std::optional<Timestamped<float>> GPS::get_ground_speed() {
    Timestamped<GNRMCData> rmc_data = this->rmc_data.load();
    Timestamped<GNVTGData> vtg_data = this->vtg_data.load();

    if (rmc_data.time > vtg_data.time) {
        return {{rmc_data.data.ground_speed, rmc_data.time}};
    } else {
        if (vtg_data.time == 0) {
            return std::nullopt;
        }
        return {{vtg_data.data.ground_speed, vtg_data.time}};
    }
}

std::optional<Timestamped<float>> GPS::get_ground_heading_north() {
    Timestamped<GNRMCData> rmc_data = this->rmc_data.load();
    Timestamped<GNVTGData> vtg_data = this->vtg_data.load();

    if (rmc_data.time > vtg_data.time) {
        return {{rmc_data.data.ground_heading, rmc_data.time}};
    } else {
        if (vtg_data.time == 0) {
            return std::nullopt;
        }
        return {{vtg_data.data.ground_course_north, vtg_data.time}};
    }
}

std::optional<Timestamped<UTCDate>> GPS::get_date() {
    Timestamped<GNRMCData> rmc_data = this->rmc_data.load();
    if (rmc_data.time == 0) {
        return std::nullopt;
    }
    return {{rmc_data.data.date, rmc_data.time}};
}

// these impls are slower than they need to be, but maybe gcc will optimize them?
// this is probably good for now and is more readable

std::optional<UTCTime> GPS::get_unread_time() {
    std::optional<Timestamped<UTCTime>> time_opt = this->get_time();
    if (!time_opt.has_value()) {
        return std::nullopt;
    }

    Timestamped<UTCTime> time = time_opt.value();
    
    if (time.time > this->last_read_time_ts) {
        this->last_read_time_ts = time.time;
        return time.data;
    } else {
        return std::nullopt;
    }
}

std::optional<float> GPS::get_unread_hdop() {
    std::optional<Timestamped<float>> hdop_opt = this->get_hdop();
    if (!hdop_opt.has_value()) {
        return std::nullopt;
    }

    Timestamped<float> hdop = hdop_opt.value();

    if (hdop.time > this->last_read_hdop_ts) {
        this->last_read_hdop_ts = hdop.time;
        return hdop.data;
    } else {
        return std::nullopt;
    }
}

std::optional<float> GPS::get_unread_altitude() {
    std::optional<Timestamped<float>> altitude_opt = this->get_altitude();
    if (!altitude_opt.has_value()) {
        return std::nullopt;
    }

    Timestamped<float> altitude = altitude_opt.value();

    if (altitude.time > this->last_read_altitude_ts) {
        this->last_read_altitude_ts = altitude.time;
        return altitude.data;
    } else {
        return std::nullopt;
    }
}

std::optional<float> GPS::get_unread_ground_speed() {
    std::optional<Timestamped<float>> gs_opt = this->get_ground_speed();
    if (!gs_opt.has_value()) {
        return std::nullopt;
    }

    Timestamped<float> gs = gs_opt.value();

    if (gs.time > this->last_read_ground_speed_ts) {
        this->last_read_ground_speed_ts = gs.time;
        return gs.data;
    } else {
        return std::nullopt;
    }
}

std::optional<float> GPS::get_unread_ground_heading_north() {
    std::optional<Timestamped<float>> gh_opt = this->get_ground_heading_north();
    if (!gh_opt.has_value()) {
        return std::nullopt;
    }

    Timestamped<float> gh = gh_opt.value();

    if (gh.time > this->last_read_ground_heading_north_ts) {
        this->last_read_ground_heading_north_ts = gh.time;
        return gh.data;
    } else {
        return std::nullopt;
    }
}

std::optional<UTCDate> GPS::get_unread_date() {
    std::optional<Timestamped<UTCDate>> date_opt = this->get_date();
    if (!date_opt.has_value()) {
        return std::nullopt;
    }

    Timestamped<UTCDate> date = date_opt.value();

    if (date.time > this->last_read_date_ts) {
        this->last_read_date_ts = date.time;
        return date.data;
    } else {
        return std::nullopt;
    }
}

void GPS::data_poll_task_wrapper(void* gps_ptr) {
    GPS* gps = (GPS*)gps_ptr;
    gps->data_poll_task();
}

void GPS::data_poll_task() {
    struct timeval tv_now;

    //this->gps_module.full_clear_queues();

    gettimeofday(&tv_now, NULL);
    int64_t start_time = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;

    uint64_t num_gngga = 0;
    uint64_t num_gnrmc = 0;
    uint64_t num_gnvtg = 0;

    uint8_t iter = 0;

    while (true) {
        /*
        if (iter == 0) {
            gettimeofday(&tv_now, NULL);
            int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
            float elapsed_s = (float)(time_ms - start_time) / 1000.0;

            ESP_LOGI(TAG, "recieved %llu gga (%f per s), %llu rmc (%f per s), %llu vtg (%f per s)",
                num_gngga, (float)num_gngga / elapsed_s,
                num_gnrmc, (float)num_gnrmc / elapsed_s,
                num_gnvtg, (float)num_gnvtg / elapsed_s
            ); 
        }
        */

        PA1616D::PolledData res = this->gps_module.poll(
            &this->gga_data,
            &this->rmc_data,
            &this->vtg_data
        );

        
        if (res == PA1616D::PolledData::GNGGA) {
            num_gngga += 1;

            GNGGAData data = this->gga_data.load().data;
            ESP_LOGI(TAG, "Parsed GNGGA message; hours: %u, minutes: %u, seconds: %u, ms: %u, hdop: %f, altitude: %f",
                data.time.hours, data.time.minutes, data.time.seconds, data.time.millis,
                data.hdop, data.altitude
            ); 
        } else if (res == PA1616D::PolledData::GNRMC) {
            num_gnrmc += 1;

            GNRMCData data = this->rmc_data.load().data;
            char mag_info[100];
            std::strncpy(mag_info, "not present", sizeof(mag_info)/sizeof(char) - 1);
            if (data.mag_info.has_value()) {
                const char* mag_c = "E";
                if (data.mag_info.value().mag_direction == MagDeclinDirection::West) {
                    mag_c = "W";
                }
                snprintf(mag_info, strlen(mag_info), "mag declination: %f, mag declin direction: %s", 
                    data.mag_info.value().mag_declination, mag_c
                );
            }
            
            ESP_LOGI(TAG, "Parsed GNRMC message; hours: %u, minutes: %u, seconds: %u, ms: %u, ground speed: %f, ground direction: %f, day: %u, month: %u, year: %u, mag info: %s",
                data.time.hours, data.time.minutes, data.time.seconds, data.time.millis,
                data.ground_speed, data.ground_heading,
                data.date.day, data.date.month, data.date.year,
                mag_info
            );
        } else if (res == PA1616D::PolledData::GNVTG) {
            num_gnvtg += 1;

            GNVTGData data = this->vtg_data.load().data;
            char mag_info[100];
            std::strncpy(mag_info, "not present", sizeof(mag_info)/sizeof(char) - 1);
            if (data.ground_course_mag.has_value()) {
                std::snprintf(mag_info, sizeof(mag_info)/sizeof(char) - 1, "%f", data.ground_course_mag.value());
            }
            
            
            ESP_LOGI(TAG, "Parsed GNVTG message; true heading: %f, mag heading: %s, speed: %f",
                data.ground_course_north, mag_info, data.ground_speed
            ); 
        }
        

        if (res != PA1616D::PolledData::None) {
            vTaskDelay(DATA_TIMEOUT);
        } else {
            vTaskDelay(NO_DATA_TIMEOUT);
        }

        iter += 1;
        iter %= 150;
    }
}

}