#pragma once

#include <cmath>
#include "sdkconfig.h"

#include "errors.h"
#include "i2c/BMI323.h"
#include "i2c/BMP581.h"
#include "i2c/high_g_accel.h"

namespace seds {
    using namespace seds::errors;

    class StageStateMachine {
    public:
        // add more stages? 
        enum class Stage : uint8_t {
            Standby = 0,
            Boost,
            Coast,
            Descent,
        };

        StageStateMachine(bool a_lock, float a_boost, int64_t standby_time, bool ac_lockout, float a_coast_diff, bool tc_lockout, int64_t coast_time, float coast_alt, int64_t apogee_detect_window) 
            : accel_boost_lockout(a_lock), accel_boost(a_boost), standby_timeout(standby_time), 
              accel_coast_lockout(ac_lockout), time_coast_lockout(tc_lockout),
              accel_coast_diff(a_coast_diff), coast_timeout(coast_time), coast_altitude(coast_alt),
              apogee_window(apogee_detect_window),
              stage(Stage::Standby) {};

        Stage step(IMUData imu_data, HighGAccelData high_g_data, BarometerData b1_data, BarometerData b2_data);

        float min_baro1_pres = INFINITY;
        int64_t baro1_timestamp = 0;
        float min_baro2_pres = INFINITY;
        int64_t baro2_timestamp = 0;

    private:
        bool standby_to_boost(float imu_mag, float high_g_mag, float time_ms);
        bool boost_to_coast(float imu_mag, float high_g_mag, BarometerData b1_data, BarometerData b2_data, float time_ms);
        bool coast_to_descent(BarometerData b1_data, BarometerData b2_data, float time_ms);

        bool accel_boost_lockout;
        float accel_boost;
        int64_t standby_timeout = 0;
        int64_t boost_ts = 0;

        bool accel_coast_lockout;
        bool time_coast_lockout;
        float accel_coast_diff;
        int64_t coast_timeout;
        float coast_altitude;

        int64_t apogee_window;

        float prev_b1_pres = 0;
        float prev_b2_pres = 0;
        int64_t prev_time = 0;

        Stage stage;

        float min_imu_mag;
        float max_imu_mag;

        float min_high_g_mag;
        float max_high_g_mag;
    };
}