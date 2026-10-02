#pragma once

#include <array>
#include <atomic>
#include <memory>

#include "errors.h"
#include "i2c/BMI323.h"
#include "gps/gps.h"
#include "i2c/high_g_accel.h"
#include "sensor/barometer.h"

namespace seds {
    using namespace seds::errors;

    struct IntegratedData {
        int64_t time;

        // integrate gyro
        std::array<float, 4> orientation = {1, 0, 0, 0}; // identity 

        // integrate imu/high g
        std::array<float, 3> velocity = {0, 0, 0};
        std::array<float, 3> position = {0, 0, 0};        
    };

    struct IntegrateDataArgs {
        std::atomic<IntegratedData> *data_ptr;
        std::shared_ptr<BMI323> imu;
        std::shared_ptr<HighGAccel> high_g_accel;
        std::shared_ptr<Barometer> baro;
        std::optional<std::shared_ptr<GPS>> gps;
        float *accel_bias;
        float *gyro_bias;
        float *high_g_bias;
    };

    void integrate_data(void *args);
}