#pragma once

#include "pa1616d.h"
#include "parsing.h"
#include <utility>

namespace seds {

class GPS {
public:
    static Expected<GPS> create();

    GPS(GPS&& other) : 
        gps_module(other.gps_module),
        gga_data(other.gga_data.load()),
        rmc_data(other.rmc_data.load()),
        vtg_data(other.vtg_data.load()),
        last_read_time_ts(other.last_read_time_ts),
        last_read_hdop_ts(other.last_read_hdop_ts),
        last_read_altitude_ts(other.last_read_altitude_ts),
        last_read_ground_speed_ts(other.last_read_ground_speed_ts),
        last_read_ground_heading_north_ts(other.last_read_ground_heading_north_ts),
        last_read_date_ts(other.last_read_date_ts)
    {}

    GPS& operator=(GPS&& other) {
        if (this != &other) {
            this->gps_module = other.gps_module;
            this->gga_data.store(other.gga_data.load());
            this->rmc_data.store(other.rmc_data.load());
            this->vtg_data.store(other.vtg_data.load());
            this->last_read_time_ts = other.last_read_time_ts;
            this->last_read_hdop_ts = other.last_read_hdop_ts;
            this->last_read_altitude_ts = other.last_read_altitude_ts;
            this->last_read_ground_speed_ts = other.last_read_ground_speed_ts;
            this->last_read_ground_heading_north_ts = other.last_read_ground_heading_north_ts;
            this->last_read_date_ts = other.last_read_date_ts;
        }

        return *this;
    }

    void data_poll_task();

    static void data_poll_task_wrapper(void* gps);

    std::optional<Timestamped<UTCTime>> get_time();
    std::optional<Timestamped<float>> get_hdop();
    std::optional<Timestamped<float>> get_altitude();
    std::optional<Timestamped<float>> get_ground_speed();
    std::optional<Timestamped<float>> get_ground_heading_north();
    std::optional<Timestamped<UTCDate>> get_date();

    std::optional<UTCTime> get_unread_time();
    std::optional<float> get_unread_hdop();
    std::optional<float> get_unread_altitude();
    std::optional<float> get_unread_ground_speed();
    std::optional<float> get_unread_ground_heading_north();
    std::optional<UTCDate> get_unread_date();

private:

    GPS(PA1616D module) : gps_module(module) {};

    PA1616D gps_module;
    std::atomic<Timestamped<GNGGAData>> gga_data = { Timestamped { 
        .data = GNGGAData {
            .time = UTCTime {
                .hours = 0,
                .minutes = 0,
                .seconds = 0,
                .millis = 0,
            }, 
            .hdop = 0,
            .altitude = 0,
        }, 
        .time = 0
    }};

    std::atomic<Timestamped<GNRMCData>> rmc_data = { Timestamped { 
        .data = GNRMCData {
            .time = UTCTime {
                .hours = 0,
                .minutes = 0,
                .seconds = 0,
                .millis = 0,
            },           
            .ground_speed = 0,
            .ground_heading = 0,            
            .date = UTCDate {
                .day = 0,
                .month = 0,
                .year = 0,
            },
            .mag_info = std::nullopt,
        }, 
        .time = 0
    }};

    std::atomic<Timestamped<GNVTGData>> vtg_data = {Timestamped { 
        .data = GNVTGData {
            .ground_course_north = 0,
            .ground_course_mag = std::nullopt,
            .ground_speed = 0 
        }, 
        .time = 0
    }};

    int64_t last_read_time_ts = 0;
    int64_t last_read_hdop_ts = 0;
    int64_t last_read_altitude_ts = 0;
    int64_t last_read_ground_speed_ts = 0;
    int64_t last_read_ground_heading_north_ts = 0;
    int64_t last_read_date_ts = 0;
};


}