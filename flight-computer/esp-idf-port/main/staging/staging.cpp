#include "staging.h"

#include <cmath>
#include "computer/computer.h"
#include <sys/time.h>

namespace seds {

StageStateMachine::Stage StageStateMachine::step(IMUData imu_data, HighGAccelData high_g_data, BarometerData b1_data, BarometerData b2_data) {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
    float imu_mag = std::sqrt(imu_data.ax * imu_data.ax + imu_data.ay * imu_data.ay + imu_data.az * imu_data.az);
    float high_g_mag = std::sqrt(high_g_data.h_ax * high_g_data.h_ax + high_g_data.h_ay * high_g_data.h_ay + high_g_data.h_az * high_g_data.h_az);

    if (imu_mag < this->min_imu_mag) {
        this->min_imu_mag = imu_mag;
    }

    if (imu_mag > this->max_imu_mag) {
        this->max_imu_mag = imu_mag;
    }

    if (high_g_mag < this->min_high_g_mag) {
        this->min_high_g_mag = high_g_mag;
    }

    if (high_g_mag > this->max_high_g_mag) {
        this->max_high_g_mag = high_g_mag;
    }

    if  (b1_data.pressure < this->min_baro1_pres) {
        this->min_baro1_pres = b1_data.pressure;
        this->baro1_timestamp = time_ms;
    }

    if (b2_data.pressure < this->min_baro2_pres) {
        this->min_baro2_pres = b2_data.pressure;
        this->baro2_timestamp = time_ms;
    }

    switch (this->stage)
    {
        case Stage::Standby :
            if (this->standby_to_boost(imu_mag, high_g_mag, time_ms)) {
                this->stage = Stage::Boost;
                this->boost_ts = time_ms;
            }
            break;
        case Stage::Boost :
            if (this->boost_to_coast(imu_mag, high_g_mag, b1_data, b2_data, time_ms)) {
                this->stage = Stage::Coast;
            }
            break;
        case Stage::Coast :
            if (this->coast_to_descent(b1_data, b2_data, time_ms)) {
                this->stage = Stage::Descent;
            }
            break;
        default:
            break;
    }

    this->prev_b1_pres = b1_data.pressure;
    this->prev_b2_pres = b2_data.pressure;
    this->prev_time = time_ms;
    return this->stage;
}

bool StageStateMachine::standby_to_boost(float imu_mag, float high_g_mag, float time_ms) {
    return (time_ms > this->standby_timeout) && (!this->accel_boost_lockout || (imu_mag >= this->accel_boost && high_g_mag >= this->accel_boost));
}

bool StageStateMachine::boost_to_coast(float imu_mag, float high_g_mag, BarometerData b1_data, BarometerData b2_data, float time_ms) {
    /*
    bool drogue_trigger = ((BARO_AGREE_DROGUE && baro1_drogue_trigger_yes && baro2_drogue_trigger_yes) 
            || (!BARO_AGREE_DROGUE && (baro1_drogue_trigger_yes || baro2_drogue_trigger_yes))
        ) && !drogue_triggered;
    */

    // only trust b1
    return (FlightComputer::pressure_to_altitude(b1_data.pressure) >= this->coast_altitude) 
        && (!this->accel_coast_lockout || (imu_mag - this->max_imu_mag <= this->accel_coast_diff && high_g_mag - this->max_high_g_mag <= this->accel_coast_diff))
        && (!this->time_coast_lockout || (time_ms - this->boost_ts >= this->coast_timeout));
}

bool StageStateMachine::coast_to_descent(BarometerData b1_data, BarometerData b2_data, float time_ms) {
    bool baro1_drogue_trigger_yes = (time_ms - this->baro1_timestamp >= this->apogee_window) 
        && (this->min_baro1_pres < b1_data.pressure);

    [[maybe_unused]] bool baro2_drogue_trigger_yes = (time_ms - this->baro2_timestamp >= this->apogee_window) 
        && (this->min_baro2_pres < b2_data.pressure); 

    return baro1_drogue_trigger_yes; // only trust b1
}

}