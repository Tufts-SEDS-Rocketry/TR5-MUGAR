#pragma once

#include <algorithm>
#include <atomic>
#include <array>

#include "driver/uart.h"
#include "esp_err.h"
#include "errors.h"
#include "esp_log.h"
#include "parsing.h"
#include "utils.h"

namespace seds {

using namespace seds::errors;

// https://cdn-shop.adafruit.com/product-files/5186/5186_PA1616D_Datasheet.pdf
// https://cdn-shop.adafruit.com/datasheets/PMTK_A08.pdf

class PA1616D {
public:
    enum class PolledData {
        GNGGA,
        GNRMC,
        GNVTG,
        None,
    };

    static Expected<PA1616D> create();

    void clear_queues();
    void full_clear_queues();

    PolledData poll(
        std::atomic<Timestamped<GNGGAData>>* gga_ptr,
        std::atomic<Timestamped<GNRMCData>>* rmc_ptr,
        std::atomic<Timestamped<GNVTGData>>* vtg_ptr
    );

private:
    explicit PA1616D() : uart_queue(NULL) {};
    QueueHandle_t uart_queue;
};

}