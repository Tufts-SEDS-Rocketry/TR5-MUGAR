#pragma once

#include "driver/mcpwm_prelude.h"
#include "errors.h"

namespace seds {
    using namespace seds::errors;

    class Motor {
    public:
        Motor(Motor&&) = default;
        Motor& operator=(Motor&&) = default;
        Motor(Motor const&) = delete;
        Motor& operator=(Motor) = delete;

        [[nodiscard]]
        static Expected<Motor> create(int32_t gpio_port, uint32_t min_pulsewidth, uint32_t max_pulsewidth, 
            uint32_t min_degree, uint32_t max_degree, uint32_t resolution, uint32_t period);

        /// Be sure to add a delay after calling this function to allow the motor to extend
        [[nodiscard]]
        Expected<std::monostate> set_angle(float angle);

        /// Users should pass in a value between 0.0 and 1.0
        /// Set proportion of period
        [[nodiscard]]
        Expected<std::monostate> set_proportion(float proportion);

        /// Set porportion of duty cycle range (min to max)
        [[nodiscard]]
        Expected<std::monostate> set_proportion_min_to_max(float proportion);

        // TODO ADD READ METHODS
    private:
        Motor(uint32_t min_pw, uint32_t max_pw, uint32_t p, float min_d, float max_d) 
            : min_pulsewidth(min_pw), max_pulsewidth(max_pw), period(p), min_degree(min_d), max_degree(max_d) {};

        mcpwm_timer_handle_t timer = NULL;
        mcpwm_oper_handle_t oper = NULL;
        mcpwm_cmpr_handle_t comparator = NULL;
        mcpwm_gen_handle_t generator = NULL;
        uint32_t min_pulsewidth;
        uint32_t max_pulsewidth;
        uint32_t period;
        float min_degree;
        float max_degree;

        uint32_t current_compare = 0;
    };
}