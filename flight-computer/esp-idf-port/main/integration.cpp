#include "computer/computer.h"
#include "integration.h"
#include "rotation/rotation.h"

constexpr float high_g_eps = 0.5 * 9.81; // if high g is more than this m/s^2 above normal accel, consider using it if...
constexpr float normal_accel_diff_from_max = 0.25 * 9.81; // high g is less than this off from its supposed max range

constexpr float v_limit = 270.0; // mach 0.8 in m/s

static const char *TAG = "integration";

constexpr float baro_complement = 0.01;
namespace seds {

void integrate_data(void *arg_ptr) 
{
    struct IntegrateDataArgs args = *(struct IntegrateDataArgs*) arg_ptr;

    // spawn gps task
    /*
    [[maybe_unused]] TaskHandle_t gps_handle;
    xTaskCreatePinnedToCore(
        GPS::data_poll_task_wrapper,
        "gps task",
        4096,   
        (void*) &args.gps,
        1, // same priority as main task
        &gps_handle,
        1  // core is 1 so it runs on this core
    );
    */


    // start paused
    vTaskSuspend(NULL);
    ESP_LOGI(TAG, "Integration started!");

    struct IntegratedData data;
    struct timeval tv_now;

    gettimeofday(&tv_now, NULL);
    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
    int64_t prev_time;
    
    IMUData imu_data = {.ax = 0, .ay = 0, .az = 0, .gx = 0, .gy = 0, .gz = 0};
    HighGAccelData high_g_data = { .h_ax = 0, .h_ay = 0, .h_az = 0 };

    while (true) {
        prev_time = time_ms;
        gettimeofday(&tv_now, NULL);
        time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
        float delta_time = (float)(time_ms - prev_time) / 1000.0;

        auto imu_data_try = args.imu->read_imu();
        if (imu_data_try.has_value()) {
            imu_data = imu_data_try.value();
        } else {
            ESP_LOGE(TAG, "imu data read failed");
        }

        auto high_g_data_try = args.high_g_accel->read_acceleration();
        if (high_g_data_try.has_value()) {
            high_g_data = high_g_data_try.value();
        } else {
            ESP_LOGE(TAG, "high g data read failed");
        }

        data = args.data_ptr->load();
        data.time = time_ms;
        
        // integrate gyro
        data.orientation = quat_rot(
            { 
                (imu_data.gx - args.gyro_bias[0]) * ((float)std::numbers::pi / 180),
                (imu_data.gy - args.gyro_bias[1]) * ((float)std::numbers::pi / 180), 
                (imu_data.gz - args.gyro_bias[2]) * ((float)std::numbers::pi / 180)
            }, 
            data.orientation,
            ((float)time_ms - (float)prev_time)/1000
        );
        data.orientation = norm4(data.orientation);

        // integrate velocity

        // acceleration from ground reference frame
        std::array<float, 3> accels = {imu_data.ax - args.accel_bias[0], imu_data.ay - args.accel_bias[1], imu_data.az - args.accel_bias[2]};
        
        // check whether to use high-g or not
        // FIXME: should this account for rotation also?
        float accel_range = args.imu->get_max_accel();
        // is x saturated?
        if (accel_range - std::abs(accels[0]) < normal_accel_diff_from_max && std::abs(high_g_data.h_ax - args.high_g_bias[0]) - std::abs(accels[0]) > high_g_eps) {
            accels[0] = high_g_data.h_ax - args.high_g_bias[0];
        }
        // is y saturated?
        if (accel_range - std::abs(accels[1]) < normal_accel_diff_from_max && std::abs(high_g_data.h_ay - args.high_g_bias[1]) - std::abs(accels[1]) > high_g_eps) {
            accels[1] = high_g_data.h_ay - args.high_g_bias[1];
        }
        // is z saturated?
        if (accel_range - std::abs(accels[2]) < normal_accel_diff_from_max && std::abs(high_g_data.h_az - args.high_g_bias[2]) - std::abs(accels[2]) > high_g_eps) {
            accels[2] = high_g_data.h_az - args.high_g_bias[2];
        }


        // data.orientation is the rockets attitude relative to ground
        // to convert accel, we use the inverse rotation
        // the operation below is technically the conjugate, but for versors (unit quaternions), its the same as the inverse
        std::array<float, 4> inv_orientation = { data.orientation[0], -data.orientation[1], -data.orientation[2], -data.orientation[3] };
        std::array<float, 3> true_accel = rot_vec(accels, inv_orientation);
        true_accel[2] = -true_accel[2]; // z axis is upside down
        ESP_LOGI(TAG, "read accel: %f,%f,%f, true accel (without added g): %f,%f,%f",
            accels[0],accels[1],accels[2],
            true_accel[0],true_accel[1],true_accel[2]
        );
        true_accel[2] -= 9.81; // account for g force

        // TODO: integrate past accel data (using rk4?)
        data.position[0] += data.velocity[0] * delta_time + (delta_time / 2.0) * true_accel[0];
        data.position[1] += data.velocity[1] * delta_time + (delta_time / 2.0) * true_accel[1];
        data.position[2] += data.velocity[2] * delta_time + (delta_time / 2.0) * true_accel[2];

        data.velocity[0] += true_accel[0] * delta_time;
        data.velocity[1] += true_accel[1] * delta_time;
        data.velocity[2] += true_accel[2] * delta_time;

        float v_mag = std::sqrt(data.velocity[0]*data.velocity[0] + data.velocity[1]*data.velocity[1] + data.velocity[2]*data.velocity[2]);
        auto baro_try = args.baro->read_data();
        if (v_mag < v_limit && baro_try.has_value()) {
            float altitude = FlightComputer::pressure_to_altitude(baro_try.value().pressure);
            data.position[2] *= (1.0 - baro_complement);
            data.position[2] += baro_complement * altitude;
        }

        args.data_ptr->store(data);
        vTaskDelay(5); // very short pause
    }
}


}