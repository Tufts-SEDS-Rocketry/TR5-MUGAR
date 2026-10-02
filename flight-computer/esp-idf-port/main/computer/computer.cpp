#include "computer.h"

#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <format>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "staging/staging.h"
#include <sys/time.h>

static const char *TAG = "computer";

namespace seds {

void FlightComputer::init_piezo() {
    ledc_timer_config_t ledc_tim_config = {
        .speed_mode = PIEZO_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = PIEZO_TIMER_NUM,
        .freq_hz = PIEZO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_tim_config);

    ledc_channel_config_t ledc_cha_config = {
        .gpio_num = PIEZO,
        .speed_mode = PIEZO_MODE,
        .channel = PIEZO_CHAN,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PIEZO_TIMER_NUM,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&ledc_cha_config);

    ledc_set_duty(PIEZO_MODE, PIEZO_CHAN, 2048);
    ledc_update_duty(PIEZO_MODE, PIEZO_CHAN);
}

Expected<std::monostate> FlightComputer::init() {
    this->init_piezo();

    //char data[] = "timestamp, accel x, accel y, accel z, degrees x, degrees y, degrees z, baro 1 temp, baro 1 pressure, baro 2 temp, baro 2 pressure, high g accel x, high g accel y, high g accel z, temp\n";
    // test different filenames

    char data[] = ""; // no header
    return this->sd.create_file_numbered_name("data", "raw", (uint8_t*)data, 0,this->filename, FlightComputer::buf_len);
}

// same NOAA pressure-altitude calculation the rrc3 does
// returns altitude in meters
float FlightComputer::pressure_to_altitude(float pressure) {
    float pres_mb = pressure / 100;
    float h_alt = 145366.45 * (1 - powf(pres_mb / 1013.25, 0.190284));
    return h_alt * 0.3048;
}

const bool BARO_AGREE_DROGUE = true; // do the barometers have to agree we've reached apogee before deploying?
const bool BARO_AGREE_MAIN = false; // do the barometers have to agree we're below main altitude?

const float MAIN_ALTITUDE = 1200 * 0.3048; // meters

const float GROUND_PRES_DIFF = 1000; // pascal

const bool BACKUP = false;
const int64_t BACKUP_DELAY = 1000; // ms

const int64_t GPIO_POWER_TIME = 2500; // ms

const char* DEPLOY_FSTRING = "time: %lld, min b1 pressure: %f, ts: %lld, min b2 pressure: %f, ts: %lld, main deploy b1 pressure: %f, main deploy b2 pressure: %f, main deploy trigger b1 pressure: %f, main deploy trigger b2 pressure: %f, drogue triggered: %lld, main triggered: %lld, main deployed: %lld\n";

// write_size 750 -> max gap 119, average gap 9.5
struct FCData {
    int32_t time_ms;
    IMUData imu_data;
    BarometerData baro_1_data;
    BarometerData baro_2_data;
    HighGAccelData high_g_data;
    float temp_data;
};

constexpr size_t LOOPS_BEFORE_FLUSH = 24;
static_assert(SD_DMA_BUF_LEN >= LOOPS_BEFORE_FLUSH * sizeof(FCData), "chunk bytes not enough to store specified buf size");
char* buffers[2];

// fixme: find a better way to do this
char FAKE_BUF[1000];

void FlightComputer::process(uint32_t times, bool endless) {
    const size_t deploy_str_len = snprintf(FAKE_BUF, sizeof(FAKE_BUF), DEPLOY_FSTRING, LLONG_MIN, __FLT_MIN__, LLONG_MIN, __FLT_MIN__, LLONG_MIN, __FLT_MIN__, __FLT_MIN__, __FLT_MIN__, __FLT_MIN__, LLONG_MIN, LLONG_MIN, LLONG_MIN);

    struct timeval tv_now;
    buffers[0] = (char *)heap_caps_aligned_alloc(32, SD_DMA_BUF_LEN, MALLOC_CAP_DMA);
    buffers[1] = (char *)heap_caps_aligned_alloc(32, SD_DMA_BUF_LEN, MALLOC_CAP_DMA);

    memset(buffers[0], 'X', sizeof(buffers[0]));
    std::atomic<size_t> idxs[2] = {{0}, {0}};
    std::atomic<size_t> which_buf = {0};
    
    // spawn other task
    struct log_args args = {
        .path = this->filename,
        .buffers = {(uint8_t*)buffers[0], (uint8_t*)buffers[1]},
        .insert_idxs = idxs,
        .which_buffer = &which_buf,
        .write_size = 500,
    };

    [[maybe_unused]] TaskHandle_t handle = unwrap(this->sd.create_log_task(args));

    StageStateMachine staging = StageStateMachine(true, ACCEL_BOOST, STANDBY_TIME, true, ACCEL_COAST_DIFF, true, COAST_TIME, COAST_ALTITUDE, APOGEE_WINDOW);

    if (GROUND_TEST) {
        staging = StageStateMachine(false, ACCEL_BOOST, STANDBY_TIME, false, ACCEL_COAST_DIFF, false, COAST_TIME, COAST_ALTITUDE, APOGEE_WINDOW);
    }   

    StageStateMachine::Stage stage;

    bool drogue_triggered = false;
    bool main_triggered = false;

    int64_t drogue_trigger_timestamp = 0;
    int64_t drogue_power_timestamp = 0;
    int64_t main_trigger_timestamp = 0;
    int64_t main_power_timestamp = 0;
    int64_t main_deploy_timestamp = 0;

    uint16_t ref_collect_cnt = 0;
    float baro_1_ref_pressure = 0;
    float baro_2_ref_pressure = 0;
    bool ref_altitude_set = false;
    float baro_1_ref_altitude = 0;
    float baro_2_ref_altitude = 0;

    uint32_t drogue_cont_lost_cnt = 0;
    uint32_t main_cont_lost_cnt = 0;

    float main_deploy_trigger_b1_pressure = 0;
    float main_deploy_trigger_b2_pressure = 0;

    float main_deploy_b1_pressure = 0;
    float main_deploy_b2_pressure = 0;

    bool data_written = false;

    uint8_t current_conts = 0;
    bool beep = false;

    for (int i = 0; i < times || endless; i++) {
        gettimeofday(&tv_now, NULL);
        int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
        IMUData imu_data = {.ax = 0, .ay = 0, .az = 0, .gx = 0, .gy = 0, .gz = 0};
        BarometerData baro_1_data = { .baro_temp = 0, .pressure = 0 };
        BarometerData baro_2_data = { .baro_temp = 0, .pressure = 0 };
        HighGAccelData high_g_data = { .h_ax = 0, .h_ay = 0, .h_az = 0 };
        float tmp = 0.0;

        auto imu_data_try = this->imu.read_imu();
        if (imu_data_try.has_value()) {
            imu_data = imu_data_try.value();
        } else {
            ESP_LOGE(TAG, "imu data read failed");
        }

        auto baro_1_data_try = this->baro1->read_data();
        if (baro_1_data_try.has_value()) {
            baro_1_data = baro_1_data_try.value();
        } else {
            ESP_LOGE(TAG, "baro 1 data read failed");
        }

        auto baro_2_data_try = this->baro2->read_data();
        if (baro_2_data_try.has_value()) {
            baro_2_data = baro_2_data_try.value();
        } else {
            ESP_LOGE(TAG, "baro 2 data read failed");
        }

        auto high_g_data_try = this->high_g_accel.read_acceleration();
        if (high_g_data_try.has_value()) {
            high_g_data = high_g_data_try.value();
        } else {
            ESP_LOGE(TAG, "high g data read failed");
        }

        /*
        auto tmp_try = this->temp.read_temperature();
        if (tmp_try.has_value()) {
            tmp = tmp_try.value();
        } else {
            ESP_LOGE(TAG, "temp data read failed");
        }
            */ 

        uint8_t num_total_conts = (gpio_get_level(MAIN_CONT) << 1) + gpio_get_level(DROGUE_CONT);
        if (time_ms % 6000 <= 5) {
            current_conts = num_total_conts;
        }

        ESP_LOGI(TAG, "current conts: %u", current_conts);

        /*
        if (!beep && (num_total_conts == 0 || (current_conts > 0 && time_ms % 2048 < 1024))) {
            ESP_LOGE(TAG, "RUN %u", 0);
            ledc_set_duty(PIEZO_MODE, PIEZO_CHAN, 4096);
            ledc_update_duty(PIEZO_MODE, PIEZO_CHAN);
            current_conts--;
            beep = true;
        }

        if (beep && num_total_conts != 0 && time_ms % 2048 >= 1024) {
            ESP_LOGE(TAG, "END %u", 0);
            ledc_set_duty(PIEZO_MODE, PIEZO_CHAN, 0);
            ledc_update_duty(PIEZO_MODE, PIEZO_CHAN);
            beep = false;
        }
        */

        stage = staging.step(imu_data, high_g_data, baro_1_data, baro_2_data);
        
        auto idx = &idxs[which_buf.load(std::memory_order_relaxed)];

        // newline in front ensures that even if we cutoff a line, we still keep the right division
        struct FCData data = {
            .time_ms = (int32_t)time_ms,
            .imu_data = imu_data,
            .baro_1_data = baro_1_data,
            .baro_2_data = baro_2_data,
            .high_g_data = high_g_data,
            .temp_data = tmp
        };

        memcpy(&buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed)], &data, sizeof(FCData));
        size_t inc = sizeof(FCData);

        //ESP_LOGI(TAG, "%lld: which: %zu, idx: %zu,  %.*s", 
        //    time_ms, which_buf.load(std::memory_order_relaxed), idx->load(std::memory_order_relaxed),
        //    inc, &buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed)] );
        
        idx->fetch_add(inc, std::memory_order_acq_rel);

        if (stage == StageStateMachine::Stage::Standby) {
            if (ref_collect_cnt < 1000) {
                baro_1_ref_pressure = (baro_1_ref_pressure * ref_collect_cnt + baro_1_data.pressure) / (ref_collect_cnt + 1);
                baro_2_ref_pressure = (baro_2_ref_pressure * ref_collect_cnt + baro_2_data.pressure) / (ref_collect_cnt + 1);
                ref_collect_cnt++;
            }
        } else if (!ref_altitude_set) {
            ref_altitude_set = true;
            baro_1_ref_altitude = FlightComputer::pressure_to_altitude(baro_1_ref_pressure);
            baro_2_ref_altitude = FlightComputer::pressure_to_altitude(baro_2_ref_pressure);
        }

        if (stage > StageStateMachine::Stage::Standby) {
            // barometer data processing
            // don't mix data across barometers to mitigate barometer errors

            bool drogue_trigger = (stage == StageStateMachine::Stage::Descent) && !drogue_triggered;

            // ESP_LOGI(TAG, "time: %lld, drogue trigger: %d, b1 ts: %lld, min pres: %f, current pres: %f, drogue power time: %lld, main power time: %lld, altitude: %f, target main: %f", 
            //     time_ms, drogue_trigger, staging.baro1_timestamp, staging.min_baro1_pres, baro_1_data.pressure,
            //     drogue_power_timestamp, main_power_timestamp, FlightComputer::pressure_to_altitude(baro_2_data.pressure) * 3.28084, MAIN_ALTITUDE * 3.2804
                // );
            //ESP_LOGI(TAG, "time: %lld, current altitude: %f, target altitude: %f, max altitude %f, ref altitude: %f\n drogue trigger: %d, drogue triggered: %d, main triggered: %d, drogue cont: %d, main cont: %d",
            //    time_ms, FlightComputer::pressure_to_altitude(baro_1_data.pressure) * 3.28084, MAIN_ALTITUDE * 3.2804 + baro_1_ref_altitude * 3.2804, 
            //    FlightComputer::pressure_to_altitude(staging.min_baro1_pres) * 3.28084, baro_1_ref_altitude * 3.2804,
            //    drogue_trigger, drogue_triggered, main_triggered, gpio_get_level(DROGUE_CONT), gpio_get_level(MAIN_CONT)
            //);

            if (drogue_trigger) {
                if (!BACKUP) {
                    gpio_set_level(DROGUE_CHUTE, 1);
                    drogue_power_timestamp = time_ms;
                    drogue_trigger_timestamp = time_ms;
                } else {
                    // only start backup if we aren't already on a backup countdown
                    if (drogue_trigger_timestamp == 0) {
                        drogue_trigger_timestamp = time_ms;
                    }
                }
            }

            if ((drogue_trigger || drogue_triggered) && !main_triggered) {
                bool baro1_main_trigger_yes = FlightComputer::pressure_to_altitude(baro_1_data.pressure) < (MAIN_ALTITUDE + baro_1_ref_altitude);
                [[maybe_unused]]  bool baro2_main_trigger_yes = FlightComputer::pressure_to_altitude(baro_2_data.pressure) < (MAIN_ALTITUDE + baro_2_ref_altitude);
                /*
                bool main_trigger = ((BARO_AGREE_DROGUE && baro1_main_trigger_yes && baro2_main_trigger_yes) 
                    || (!BARO_AGREE_MAIN && (baro1_main_trigger_yes || baro2_main_trigger_yes)))
                */
                bool main_trigger = baro1_main_trigger_yes;

                if (main_trigger) {
                    if (!BACKUP) {
                        gpio_set_level(MAIN_CHUTE, 1);
                        main_power_timestamp = time_ms;
                        main_trigger_timestamp = time_ms;
                        main_triggered = true;
                    } else {
                        // only start backup if we aren't already on a backup countdown
                        if (main_trigger_timestamp == 0) {
                            main_trigger_timestamp = time_ms;
                        }
                    }
                    main_deploy_trigger_b1_pressure = baro_1_data.pressure;
                    main_deploy_trigger_b2_pressure = baro_2_data.pressure;
                }
            }

            if (BACKUP) {
                if (!drogue_triggered && drogue_trigger_timestamp != 0 && time_ms - drogue_trigger_timestamp > BACKUP_DELAY) {
                    gpio_set_level(DROGUE_CHUTE, 1);
                    drogue_power_timestamp = time_ms;
                }

                else if (!main_triggered && main_trigger_timestamp != 0 && time_ms - main_trigger_timestamp > BACKUP_DELAY) {
                    gpio_set_level(MAIN_CHUTE, 1);
                    main_power_timestamp = time_ms;
                }
            }

            // force drogue trigged if we lose continuity
            // this way sequencing works even if we aren't the ones to deploy the drogue
            if (gpio_get_level(DROGUE_CONT) == 0) {
                drogue_cont_lost_cnt++;
            } else {
                drogue_cont_lost_cnt = 0;
            }
            if (drogue_cont_lost_cnt > 20) {
                drogue_triggered = true;
            }

            if (gpio_get_level(MAIN_CONT) == 0) {
                main_cont_lost_cnt++;
            } else {
                main_cont_lost_cnt = 0;
            }
            if (main_cont_lost_cnt > 20) {
                main_triggered = true;
                main_deploy_b1_pressure = baro_1_data.pressure;
                main_deploy_b2_pressure = baro_2_data.pressure; 
                main_deploy_timestamp = time_ms;
            }

            if (gpio_get_level(DROGUE_CHUTE) == 1 && time_ms - drogue_power_timestamp >= GPIO_POWER_TIME) {
                gpio_set_level(DROGUE_CHUTE, 0);
            }

            if (gpio_get_level(MAIN_CHUTE) == 1 && time_ms - main_power_timestamp >= GPIO_POWER_TIME) {
                gpio_set_level(MAIN_CHUTE, 0);
            }
        }
        
        // no room left
        if (SD_DMA_BUF_LEN - idx->load(std::memory_order_relaxed) <= sizeof(FCData) + deploy_str_len) {
            ESP_LOGI(TAG, "FLUSHING");

            if (drogue_triggered && main_triggered && !data_written) {
                size_t inc = snprintf(&buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed)], 1000, DEPLOY_FSTRING, 
                    time_ms, staging.min_baro1_pres, staging.baro1_timestamp, staging.min_baro2_pres, staging.baro2_timestamp, main_deploy_b1_pressure, main_deploy_b2_pressure, main_deploy_trigger_b1_pressure, 
                    main_deploy_trigger_b2_pressure, drogue_trigger_timestamp, main_trigger_timestamp, main_deploy_timestamp
                );
                idx->fetch_add(inc, std::memory_order_acq_rel);

                data_written = true;
            }           

            size_t next_which = (which_buf.load(std::memory_order_relaxed) + 1) % 2;
            auto next_idx = &idxs[next_which];
            next_idx->store(0, std::memory_order_seq_cst);
            which_buf.store(next_which, std::memory_order_seq_cst);

            ESP_LOGI(TAG, "RESUMING OTHER TASK");
        }
    }
}

}