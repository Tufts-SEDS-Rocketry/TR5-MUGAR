#include "airbrakes.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "computer/computer.h"
#include "control/control.h"
#include "integration.h"
#include "rotation/rotation.h"
#include "staging/staging.h"

static const char *TAG = "airbrakes";

namespace seds {

constexpr bool CALIB_AVERAGE = true;

const int32_t servo_gpio = 15;
const uint32_t period = 20000; // 20ms
const uint32_t min_pulsewidth = (0.0461 * (float)period); // microsecond
const uint32_t max_pulsewidth = (0.0708 * (float)period);
const uint32_t min_degree = 0;
const uint32_t max_degree = 198; // degrees
const uint32_t resolution = 1000000; // 1MHz


const float TARGET_APOGEE = 10000.0 * 0.3048; // meters
const float START_AIRBRAKING = 2500.0; 

const int64_t TIME_PER_EXT = 200; // 0.2 seconds

Expected<Airbrakes> Airbrakes::create(std::shared_ptr<Barometer> b1, std::shared_ptr<Barometer> b2, BMI323 im, HighGAccel high_g, SDCard s, std::optional<GPS> gps) {
    // make motor
    Motor m = TRY(Motor::create(servo_gpio, min_pulsewidth, max_pulsewidth, min_degree, max_degree, resolution, period));

    return Airbrakes(b1, b2, std::move(im), std::move(high_g), std::move(m), std::move(s), std::move(gps));
}

const char* BASE_FILE_NAME = "exts";
const char* FILE_EXT = "raw";

const size_t fname_buf = 30;

float filter_coeefs[11] = {
    0.000787,
    0.003923,
    0.01026,
    0.019747,
    0.03207,
    0.046555,
    0.062254,
    0.077977,
    0.092471,
    0.104473,
    0.108992
};

float filter_over_rbuf(float* baro_ring_buffer, uint32_t ring_idx) {
    float v = 0;
    
    for (int i = 0; i < 10; i++) {
        size_t real_idx_end = (ring_idx + i) % 21;
        size_t real_idx_start = 20 - real_idx_end;
        v += (baro_ring_buffer[real_idx_end] + baro_ring_buffer[real_idx_start]) * filter_coeefs[i];
    }
    v += baro_ring_buffer[(ring_idx + 10) % 21] * filter_coeefs[10];

    return v;
}

char* airbrake_buffers[2];

struct AirbrakeDataExtend {
    int32_t time;
    float proportion;
    float orientation[4];
    float velocity[3];
    float position[4];
};

void Airbrakes::run_steps() {
    float baro_ring_buffer[21];
    
    for (int i = 0; i < 21; i++) {
        baro_ring_buffer[i] = 0.0;
    }

    uint32_t ring_idx = 0;

    // create buffer file
    char filename[fname_buf];

    char data[] = ""; // no header
    unwrap(this->sd.create_file_numbered_name(BASE_FILE_NAME, FILE_EXT, (uint8_t*)data, 0,filename, fname_buf));

    ESP_LOGI(TAG, "Extention file name is %s", filename);

    struct timeval tv_now;
    airbrake_buffers[0] = (char *)heap_caps_aligned_alloc(32, SD_DMA_BUF_LEN, MALLOC_CAP_DMA);
    airbrake_buffers[1] = (char *)heap_caps_aligned_alloc(32, SD_DMA_BUF_LEN, MALLOC_CAP_DMA);

    memset(airbrake_buffers[0], 'X', sizeof(airbrake_buffers[0]));
    memset(airbrake_buffers[1], 'X', sizeof(airbrake_buffers[1]));
    std::atomic<size_t> idxs[2] = {{0}, {0}};
    std::atomic<size_t> which_buf = {0};
    
    // spawn other task
    struct log_args log_args = {
        .path = filename,
        .buffers = {(uint8_t*)airbrake_buffers[0], (uint8_t*)airbrake_buffers[1]},
        .insert_idxs = idxs,
        .which_buffer = &which_buf,
        .write_size = 500,
    };

    [[maybe_unused]] TaskHandle_t handle = unwrap(this->sd.create_log_task(log_args));

    float gyro_bias[3] = {0,0,0};
    float accel_bias[3] = {0,0,0};
    float high_g_bias[3] = {0,0,0};
    std::atomic<IntegratedData> integration_data;

    struct seds::IntegrateDataArgs integrate_args = {
        .data_ptr = &integration_data,
        .imu = this->imu,
        .high_g_accel = this->high_g_accel,
        .baro = this->baro1,
        .gps = this->gps,
        .accel_bias = accel_bias,
        .gyro_bias = gyro_bias,
        .high_g_bias = high_g_bias
    };

    int priority = 1; // same as sd task
    // put on same core as sd task
    auto t_core = 1;
    if (CONFIG_ESP_MAIN_TASK_AFFINITY == t_core) {
        t_core = 0;
    }

    TaskHandle_t integration_handle;
    xTaskCreatePinnedToCore(integrate_data, "integrating task", 4096, (void*)&integrate_args, priority, &integration_handle, t_core);

    StageStateMachine staging = StageStateMachine(true, ACCEL_BOOST, STANDBY_TIME, true, ACCEL_COAST_DIFF, true, COAST_TIME, COAST_ALTITUDE, APOGEE_WINDOW);

    if (GROUND_TEST) {
        staging = StageStateMachine(false, ACCEL_BOOST, STANDBY_TIME, false, ACCEL_COAST_DIFF, false, COAST_TIME, COAST_ALTITUDE, APOGEE_WINDOW);
    }

    int64_t time_ms = 0;

    BarometerData baro_1_data = { .baro_temp = 0, .pressure = 0 };
    BarometerData baro_2_data = { .baro_temp = 0, .pressure = 0 };

    IMUData imu_data = {.ax = 0, .ay = 0, .az = 0, .gx = 0, .gy = 0, .gz = 0};
    HighGAccelData high_g_data = { .h_ax = 0, .h_ay = 0, .h_az = 0 };

    float prev_z = 0.0;
    int64_t pz_ts = 0;
    float z_est = 0.0;
    int64_t z_ts = 0;

    float vz = 0;

    // DUPLICATE CODE FROM FLIGHT COMPUTER
    auto update_data = [this, &baro_ring_buffer, &ring_idx, &imu_data, &high_g_data, &baro_1_data, &baro_2_data, &z_est, &prev_z, &z_ts, &pz_ts] 
        (int64_t time_ms) 
    {
        auto imu_data_try = this->imu->read_imu();
        if (imu_data_try.has_value()) {
            imu_data = imu_data_try.value();
        } else {
            ESP_LOGE(TAG, "imu data read failed");
        }

        auto high_g_data_try = this->high_g_accel->read_acceleration();
        if (high_g_data_try.has_value()) {
            high_g_data = high_g_data_try.value();
        } else {
            ESP_LOGE(TAG, "high g data read failed");
        }

        auto baro_1_data_try = this->baro1->read_data();
        if (baro_1_data_try.has_value()) {
            baro_1_data = baro_1_data_try.value();
            baro_ring_buffer[ring_idx] = baro_1_data.pressure;
            ring_idx += 1;
            ring_idx %= 21;

            prev_z = z_est;
            pz_ts = z_ts;
            z_est = FlightComputer::pressure_to_altitude(filter_over_rbuf(baro_ring_buffer, ring_idx));
            z_ts = time_ms;
        } else {
            ESP_LOGE(TAG, "baro 1 data read failed");
        }

        //auto baro_2_data_try = this->baro2->read_data();
        //if (baro_2_data_try.has_value()) {
        //    baro_2_data = baro_2_data_try.value();
        //} else {
            //ESP_LOGE(TAG, "baro 2 data read failed");
        //}
    };

    gettimeofday(&tv_now, NULL);
    time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
    update_data(time_ms);

    // reset airbrakes
    Expected<std::monostate> motor_res = this->motor.set_proportion_min_to_max(0.0);
    if (!motor_res.has_value()) {
        ESP_LOGE(TAG, "%s", motor_res.error()->what());
    }


    // determines 'bias' in gyro speed reading
    // since during standby we're still on the rail and thus fixed
    // use double to ensure average is stable
    // over 10 seconds at 100 hz we should have ~1000 samples, so we probably wont need to error correct
    // if we're really worried, look into kahan summation
    // https://ieeexplore.ieee.org/document/5289161    
    uint32_t num_samples = 0;

    // standby
    while (staging.step(imu_data, high_g_data, baro_1_data, baro_2_data) == StageStateMachine::Stage::Standby) {
        gettimeofday(&tv_now, NULL);
        time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
        ESP_LOGI(TAG, "time: %lld", time_ms);
        update_data(time_ms);

        if (CALIB_AVERAGE) {
            num_samples += 1;
            gyro_bias[0] += (imu_data.gx - gyro_bias[0]) / num_samples;
            gyro_bias[1] += (imu_data.gy  - gyro_bias[1]) / num_samples;
            gyro_bias[2] += ( imu_data.gz  - gyro_bias[2]) / num_samples;

            accel_bias[0] += (imu_data.ax - accel_bias[0]) / num_samples;
            accel_bias[1] += (imu_data.ay - accel_bias[1]) / num_samples;
            accel_bias[2] += (imu_data.az + 9.81 - accel_bias[2]) / num_samples; // add gravity since our sensors don't account for it

            high_g_bias[0] += (high_g_data.h_ax - high_g_bias[0]) / num_samples;
            high_g_bias[1] += (high_g_data.h_ay - high_g_bias[1]) / num_samples;
            high_g_bias[2] += (high_g_data.h_az + 9.81 - high_g_bias[2]) / num_samples; // add gravity since our sensors don't account for it
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    //vTaskResume(integration_handle);

    while (staging.step(imu_data, high_g_data, baro_1_data, baro_2_data) == StageStateMachine::Stage::Boost) {
        gettimeofday(&tv_now, NULL);
        time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
        update_data(time_ms);

        //auto idx = &idxs[which_buf.load(std::memory_order_relaxed)];

        //memcpy(&airbrake_buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed)], &time_ms, sizeof(int64_t));
        //memcpy(&airbrake_buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed) + sizeof(int64_t)], &imu_data, sizeof(IMUData));
        
        //size_t inc = sizeof(IMUData) + sizeof(int64_t);
        
        //idx->fetch_add(inc, std::memory_order_acq_rel);

        //size_t next_which = (which_buf.load(std::memory_order_relaxed) + 1) % 2;
        //auto next_idx = &idxs[next_which];
        //next_idx->store(0, std::memory_order_seq_cst);
        //which_buf.store(next_which, std::memory_order_seq_cst);

        auto id_data = integration_data.load();
        std::array<float, 4> orientation = id_data.orientation;
        std::array<float, 3> pos = id_data.position;

        auto vertical = rot_vec({0, 0, 1}, orientation);
        auto x_axis = rot_vec({1, 0, 0}, orientation);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vz = (z_est - prev_z) / (z_ts - pz_ts) * 1000; // timestamps are in ms

    while (staging.step(imu_data, high_g_data, baro_1_data, baro_2_data) < StageStateMachine::Stage::Descent) {
        gettimeofday(&tv_now, NULL);
        time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
        update_data(time_ms);
        auto id_data = integration_data.load();
        std::array<float, 4> orientation = id_data.orientation;
        std::array<float, 3> pos = id_data.position;
        std::array<float, 3> velocity = id_data.velocity;

        vz = (z_est - prev_z) / (z_ts - pz_ts) * 1000;

        float control_u = control(z_est, vz);
        motor_res = this->motor.set_proportion_min_to_max(control_u);
        if (!motor_res.has_value()) {
            ESP_LOGE(TAG, "%s", motor_res.error()->what());
        } 

        ESP_LOGI(TAG, "position: %f, velocity: %f, control: %f", pos[2], velocity[2], control_u);

        auto idx = &idxs[which_buf.load(std::memory_order_relaxed)];

        memcpy(&airbrake_buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed)], &time_ms, sizeof(time_ms));
        memcpy(&airbrake_buffers[which_buf.load(std::memory_order_relaxed)][idx->load(std::memory_order_relaxed) + sizeof(time_ms)], &control_u, sizeof(control_u));

        size_t inc = sizeof(time_ms) + sizeof(control_u);
        
        idx->fetch_add(inc, std::memory_order_acq_rel);

        size_t next_which = (which_buf.load(std::memory_order_relaxed) + 1) % 2;
        auto next_idx = &idxs[next_which];
        next_idx->store(0, std::memory_order_seq_cst);
        which_buf.store(next_which, std::memory_order_seq_cst);  
       
        int64_t start_time = time_ms;

        while (staging.step(imu_data, high_g_data, baro_1_data, baro_2_data) < StageStateMachine::Stage::Descent 
            && time_ms - start_time <= TIME_PER_EXT) 
        {
            gettimeofday(&tv_now, NULL);
            time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
            update_data(time_ms);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }

    motor_res = this->motor.set_proportion_min_to_max(0.0);
    if (!motor_res.has_value()) {
        ESP_LOGE(TAG, "%s", motor_res.error()->what());
    }
    while (true) {
        motor_res = this->motor.set_proportion_min_to_max(0.0);
        if (!motor_res.has_value()) {
            ESP_LOGE(TAG, "%s", motor_res.error()->what());
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

}