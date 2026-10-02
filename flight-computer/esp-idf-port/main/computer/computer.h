#pragma once

#include <expected>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "errors.h"
#include "esp_log.h"
#include "i2c/BMI323.h"
#include "i2c/BMP581.h"
#include "i2c/high_g_accel.h"
#include "i2c/I2C.h"
#include "i2c/MLX90395.h"
#include "i2c/segment7.h"
#include "i2c/TMP1075.h"
#include "sd.h"
#include "utils.h"

namespace seds {
    using namespace seds::errors;

    const gpio_num_t MAIN_CONT = GPIO_NUM_18;
    const gpio_num_t DROGUE_CONT = GPIO_NUM_19;
    const gpio_num_t PIEZO = GPIO_NUM_21;
    const gpio_num_t DROGUE_CHUTE = GPIO_NUM_22;
    const gpio_num_t MAIN_CHUTE = GPIO_NUM_23;

    const ledc_channel_t PIEZO_CHAN = LEDC_CHANNEL_0;
    const ledc_mode_t PIEZO_MODE = LEDC_LOW_SPEED_MODE;
    const ledc_timer_t PIEZO_TIMER_NUM = LEDC_TIMER_0;
    const uint32_t PIEZO_FREQ_HZ = 4000;

    class FlightComputer {
    private:
        static constexpr size_t buf_len = MOUNT_POINT_LEN + 1 + 3 + 4 + 4 + 1;

        void init_piezo(); // add failure checks for this!
    public:
        std::shared_ptr<Barometer> baro1;
        std::shared_ptr<Barometer> baro2;
        // fix : SegmentDisplay display;
        BMI323 imu;
        HighGAccel high_g_accel;
        //MLX90395 mag;
        TMP1075 temp;
        SDCard sd;
        // mount point, slash, 3 numbers, 'data', '.csv'
        char filename[FlightComputer::buf_len] = MOUNT_POINT"/data.csv"; 

        Expected<std::monostate> init(void);
 
        void process(uint32_t times, bool endless);

        static float pressure_to_altitude(float pressure);
    };
}