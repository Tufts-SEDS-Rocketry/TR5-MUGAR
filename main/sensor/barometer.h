#pragma once

#include "esp_log.h"
#include "errors.h"
#include "i2c/I2C.h"

namespace seds {
    using namespace seds::errors;

    struct BarometerData {
        float baro_temp;
        float pressure;
    };

    class Barometer : std::enable_shared_from_this<Barometer> {
    public:
        Barometer() {
            ESP_LOGE("barometer", "Creating fake barometer!");
        }

        Barometer(Barometer&&) = default;
        Barometer& operator=(Barometer&&) = default;
        Barometer(const Barometer&) = delete;
        Barometer& operator=(Barometer const&) = delete;

        virtual bool is_connected() {
            ESP_LOGE("barometer", "Barometer tried to check connection");
            return false;
        };

        /// Read the last temperature and pressure measurement from the sensor.
        [[nodiscard]]
        virtual Expected<BarometerData> read_data() {
            ESP_LOGE("barometer", "Barometer tried to read data");
            return std::unexpected(std::make_unique<std::runtime_error>(std::runtime_error("Barometer tried to read data")));
        }
    private:
    };
}