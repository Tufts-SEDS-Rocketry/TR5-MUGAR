#include "pa1616d.h"

#include <cstring>
#include "freertos/task.h"
#include "freertos/queue.h"
#include "parsing.h"
#include <string_view>
#include <sys/time.h>

constexpr uart_port_t uart_port = UART_NUM_1;
constexpr uint32_t uart_buffer_size = 1024 * 2;

// apparently the uart fifo size is 128 bytes 
// (source: https://www.reddit.com/r/embedded/comments/12rjn5k/comment/jgxg04b/)
// so we set the high mark at 120
// I just made up the low value of 20, but we can adjust it
constexpr uint8_t rx_thresh_xon = 20;
constexpr uint8_t rx_thresh_xoff = 60; 

constexpr uint8_t event_queue_size = 20;
constexpr uint8_t pattern_queue_size = event_queue_size;
constexpr uint32_t baud_rate = 9600; 
constexpr uint32_t gps_tx_pin = 32;
constexpr uint32_t gps_rx_pin = 35;

constexpr int64_t restart_timeout = 5000;

// use this sheet!
// https://cdn-shop.adafruit.com/product-files/5186/5186_PA1616D_Datasheet.pdf
// https://simcom.ee/documents/SIM33ELA/MT3333%20Platform%20NMEA%20Message%20Specification%20For%20GPS%2BGLONASS_V1.00.pdf

// turn on GNRMC, GNVTG, GPGGA
// https://www.hhhh.org/wiml/proj/nmeaxor.html
const char* PMTK_DISABLE = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
const char* PMTK_HOT_RESTART = "$PMTK101*32\r\n";
const char* PMTK_SET_SPEED = "$PMTK220,100*2F\r\n";
const char* PMTK_SET_BAUDRATE = "$PMTK251,9600*17\r\n";
const char* PMTK_SET_NMEA_OUTPUT 
    = "$PMTK314,0,1,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2A\r\n"; 

const char* PMTK_ENABLE_SATS
    = "$PMTK314,0,1,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*2A\r\n";

// https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/PMTK_Packet_User_Manual.pdf
// https://cdn.sparkfun.com/assets/f/0/9/1/c/PMTK_Protocol.pdf
// 220 PMTK_SET_NMEA_UPDATERATE
// 251 PMTK_SET_NMEA_BAUDRATE
// 314 PMTK_API_SET_NMEA_OUTPUT
// 250 PMTK_SET_DATA_PORT


#define TAG "PA16161D"

namespace seds {

Expected<PA1616D> PA1616D::create() {
    const char* cfg_msg = PMTK_SET_NMEA_OUTPUT;

    PA1616D gps_sensor = PA1616D();

    ESP_LOGI(TAG, "installing uart driver");
    ESP_TRY(uart_driver_install(uart_port, uart_buffer_size, uart_buffer_size, event_queue_size, &gps_sensor.uart_queue, 0));

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };
    
    // configure UART parameters
    ESP_LOGI(TAG, "configuring uart params");
    ESP_TRY(uart_param_config(uart_port, &uart_config));

    ESP_LOGI(TAG, "enabling software flow control");
    ESP_TRY(uart_set_sw_flow_ctrl(uart_port, true, rx_thresh_xon, rx_thresh_xoff));
    
    // set UART pins
    ESP_LOGI(TAG, "setting uart pins");
    ESP_TRY(uart_set_pin(uart_port, gps_tx_pin, gps_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // wait
    vTaskDelay(pdMS_TO_TICKS(2500));

    // disable messages
    ESP_LOGI(TAG, "sending disable message");
    int32_t num_written = uart_write_bytes(uart_port, PMTK_DISABLE, strlen(PMTK_DISABLE));
    if (num_written != strlen(PMTK_DISABLE)) {
        ESP_LOGE(TAG, "wasn't able to write all bytes of disable msg to gps");
        ESP_LOGE(TAG, "wrote %d bytes out of %d total", num_written, strlen(PMTK_DISABLE));
        return std::unexpected(
            std::make_unique<std::runtime_error>(
                std::runtime_error("wasn't able to write all bytes of disable msg to gps")
        ));
    }

    // wait
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_flush(uart_port);
    xQueueReset(gps_sensor.uart_queue);

    // enable pattern matching on end character
    // timeout is 10000 bc our baud cycle is 9600
    // we get a message every tenth of a second (timeout of 960)
    // but we use a whole second just to avoid errors
    ESP_TRY(uart_enable_pattern_det_baud_intr(uart_port, '\n', 1, 10000, 0, 0));

    // restart gps
    ESP_LOGI(TAG, "sending full restart message");
    num_written = uart_write_bytes(uart_port, PMTK_HOT_RESTART, strlen(PMTK_HOT_RESTART));
    if (num_written != strlen(PMTK_HOT_RESTART)) {
        ESP_LOGE(TAG, "wasn't able to write all bytes of restart msg to gps");
        ESP_LOGE(TAG, "wrote %d bytes out of %d total", num_written, strlen(PMTK_HOT_RESTART));
        return std::unexpected(
            std::make_unique<std::runtime_error>(
                std::runtime_error("wasn't able to write all bytes of restart msg to gps")
        ));
    }
    
    // poll for restart message
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t start_time = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
    int64_t time_ms = start_time;

    bool got_mtkgps = false;
    bool got_001 = false;

    uart_event_t event;
    size_t buffered_size;
    std::array<uint8_t, uart_buffer_size> tmp_buf;
    
    while (time_ms - start_time < restart_timeout && !(got_mtkgps && got_001 )) {
        gettimeofday(&tv_now, NULL);
        time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;
        if (xQueueReceive(gps_sensor.uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {  
            if (event.type == UART_PATTERN_DET) {
                uart_get_buffered_data_len(uart_port, &buffered_size);
                int pos = uart_pattern_pop_pos(uart_port);
                if (pos == -1) {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    ESP_LOGE(TAG, "No room to store pattern det event in queue");
                    uart_flush_input(uart_port);
                } else {
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                    uart_read_bytes(uart_port, tmp_buf.data(), pos + 1, 100 / portTICK_PERIOD_MS);
                    tmp_buf[pos + 1] = 0;
                    std::string_view msg = std::string_view((char*)(tmp_buf.data() + 1));

                    if (msg == "PMTK011,MTKGPS*08\r\n") {
                        got_mtkgps = true;
                    } else if (msg == "PMTK010,001*2E\r\n") {
                        got_001 = true;
                    } else {
                        ESP_LOGE(TAG, "Unrecognized gps message %s", msg.data());
                    }
                }
            }
        }
    }

    if (!got_mtkgps) {
        ESP_LOGE(TAG, "Timed out, didn't receive message \"PMTK011,MTKGPS*08\" after restart");
    } 

    if (!got_001) {
        ESP_LOGE(TAG, "Timed out, didn't receive message \"PMTK010,001*2E\" after restart");
    }

    if (got_mtkgps && got_001) {
        ESP_LOGI(TAG, "Successfully receieved restart messages");
    }

    // set baud
    ESP_LOGI(TAG, "sending set baud message");
    num_written = uart_write_bytes(uart_port, PMTK_SET_BAUDRATE, strlen(PMTK_SET_BAUDRATE));
    if (num_written != strlen(PMTK_SET_BAUDRATE)) {
        ESP_LOGE(TAG, "wasn't able to write all bytes of baud msg to gps");
        ESP_LOGE(TAG, "wrote %d bytes out of %d total", num_written, strlen(PMTK_SET_BAUDRATE));
        return std::unexpected(
            std::make_unique<std::runtime_error>(
                std::runtime_error("wasn't able to write all bytes of baud msg to gps")
        ));
    }

    // wait
    vTaskDelay(pdMS_TO_TICKS(100));

    // set fix rate
    ESP_LOGI(TAG, "sending set fix rate message");
    num_written = uart_write_bytes(uart_port, PMTK_SET_SPEED, strlen(PMTK_SET_SPEED));
    if (num_written != strlen(PMTK_SET_SPEED)) {
        ESP_LOGE(TAG, "wasn't able to write all bytes of speed msg to gps");
        ESP_LOGE(TAG, "wrote %d bytes out of %d total", num_written, strlen(PMTK_SET_SPEED));
        return std::unexpected(
            std::make_unique<std::runtime_error>(
                std::runtime_error("wasn't able to write all bytes of speed msg to gps")
        ));
    }

    // wait
    vTaskDelay(pdMS_TO_TICKS(100));

    // set config
    ESP_LOGI(TAG, "sending format message");
    num_written = uart_write_bytes(uart_port, cfg_msg, strlen(cfg_msg));
    if (num_written != strlen(cfg_msg)) {
        ESP_LOGE(TAG, "wasn't able to write all bytes of format msg to gps");
        ESP_LOGE(TAG, "wrote %d bytes out of %d total", num_written, strlen(cfg_msg));
        vTaskDelay(pdMS_TO_TICKS(50));
        return std::unexpected(
            std::make_unique<std::runtime_error>(
                std::runtime_error("wasn't able to write all bytes of format msg to gps")
        ));
    }

    return gps_sensor;
}

void PA1616D::clear_queues() {
    uart_flush(uart_port);
    //xQueueReset(this->uart_queue);
    uart_pattern_queue_reset(uart_port, pattern_queue_size);
}

void PA1616D::full_clear_queues() {
    uart_flush(uart_port);
    xQueueReset(this->uart_queue);
    uart_pattern_queue_reset(uart_port, pattern_queue_size);
}

PA1616D::PolledData PA1616D::poll(std::atomic<Timestamped<GNGGAData>>* gga_ptr, 
        std::atomic<Timestamped<GNRMCData>>* rmc_ptr,
        std::atomic<Timestamped<GNVTGData>>* vtg_ptr) 
{
    uart_event_t event;
    size_t buffered_size;
    std::array<uint8_t, uart_buffer_size> tmp_buf;
    int pos;

    if (xQueueReceive(this->uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {  
        if (event.type == UART_PATTERN_DET) {
            uart_get_buffered_data_len(uart_port, &buffered_size);
            //ESP_LOGI(TAG, "num items in queue: %u, buffered data length: %u", 
            //    uxQueueMessagesWaiting(this->uart_queue), buffered_size
            //);
            pos = uart_pattern_pop_pos(uart_port);
            if (pos == -1) {
                // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                // record the position. We should set a larger queue size.
                // As an example, we directly flush the rx buffer here.
                ESP_LOGE(TAG, "No room to store pattern det event in queue");
                uart_flush(uart_port);
                return PolledData::None;
            } else {
                vTaskDelay(5 / portTICK_PERIOD_MS);
                uart_read_bytes(uart_port, tmp_buf.data(), pos + 1, 100 / portTICK_PERIOD_MS);
                tmp_buf[pos + 1] = 0;
            }
        } else {
            return PolledData::None;
        }
    } else {
        return PolledData::None;
    }

    struct timeval tv_now;  
    gettimeofday(&tv_now, NULL);
    int64_t time_ms = (int64_t)tv_now.tv_sec * 1000L + (int64_t)tv_now.tv_usec / 1000L;


    //ESP_LOGI(TAG, "pos: %d, buffered size: %u, message: %s", pos, buffered_size, (char*)tmp_buf.data());
    std::string_view code = std::string_view((char*)(tmp_buf.data() + 1), 5);

    if (code == "GNGGA") {
        //ESP_LOGI(TAG, "detected message with code GNGGA\n");
        auto parse_res = parse_gngga_msg(std::string_view((char*)(tmp_buf.data() + 7)));
        if (!parse_res.has_value()) {
            ESP_LOGE(TAG, "Error in parsing gngga message: %s", parse_res.error()->what());
            return PolledData::None;
        }
        GNGGAData data = parse_res.value();
        gga_ptr->store( Timestamped {
            .data = data,
            .time = time_ms
        });
        return PolledData::GNGGA;
    } else if (code == "GNRMC") {
        //ESP_LOGI(TAG, "detected message with code GNRMC\n");
        auto parse_res = parse_gnrmc_msg(std::string_view((char*)(tmp_buf.data() + 7)));
        if (!parse_res.has_value()) {
            ESP_LOGE(TAG, "Error in parsing gnrmc message: %s", parse_res.error()->what());
            return PolledData::None;
        }
        GNRMCData data = parse_res.value();
        rmc_ptr->store( Timestamped {
            .data = data,
            .time = time_ms
        });
        return PolledData::GNRMC;
    } else if (code == "GNVTG") {
        //ESP_LOGI(TAG, "detected message with code GNVTG\n");
        auto parse_res = parse_gnvtg_msg(std::string_view((char*)(tmp_buf.data() + 7)));
        if (!parse_res.has_value()) {
            ESP_LOGE(TAG, "Error in parsing gnvtg message: %s", parse_res.error()->what());
            return PolledData::None;
        }

        std::optional<GNVTGData> maybe_data = parse_res.value();
        if (!maybe_data.has_value()) {
            // data invalid
            ESP_LOGI(TAG, "Parsed GNVTG message; data invalid");
            return PolledData::None;
        }
        GNVTGData data = maybe_data.value();
        vtg_ptr->store( Timestamped {
            .data = data,
            .time = time_ms
        });
        return PolledData::GNVTG;
    } else if (code == "GPGSV") {
        ESP_LOGE(TAG, "detected message with code GPGSV, skipping");
        return PolledData::None;
    } else if (code == "GLGSV") {
        ESP_LOGE(TAG, "detected message with code GLGSV, skipping");
        return PolledData::None;
    } else if (code == "GPGSA") {
        ESP_LOGE(TAG, "detected message with code GPGSA, skipping");
        return PolledData::None;
    } else if (code == "GLGSA") {
        ESP_LOGE(TAG, "detected message with code GLGSA, skipping");
        return PolledData::None;
    } else {
        ESP_LOGE(TAG, "Unrecognized gps message: %.*s", pos, (char*)(tmp_buf.data() + 1));
        ESP_LOGE(TAG, "Code: %.*s", 5, (char*)(tmp_buf.data() + 1));
        return PolledData::None;
    }
}

}