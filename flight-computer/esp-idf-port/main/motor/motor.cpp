#include "motor.h"

namespace seds {

Expected<Motor> Motor::create(int32_t gpio_port, uint32_t min_pulsewidth, uint32_t max_pulsewidth, 
    uint32_t min_degree, uint32_t max_degree, uint32_t resolution, uint32_t period)
{
    Motor m = Motor(min_pulsewidth, max_pulsewidth, period, min_degree, max_degree);

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = resolution,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = period,
    };

    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };

    mcpwm_comparator_config_t comparator_config;
    comparator_config.flags.update_cmp_on_tez = true;
    comparator_config.intr_priority = 0;

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_port,
    };

    // FIXME CHECK ERRORS
    mcpwm_new_timer(&timer_config, &m.timer);
    mcpwm_new_operator(&operator_config, &m.oper);
    mcpwm_operator_connect_timer(m.oper, m.timer);
    mcpwm_new_comparator(m.oper, &comparator_config, &m.comparator);
    mcpwm_new_generator(m.oper, &generator_config, &m.generator);
    mcpwm_comparator_set_compare_value(m.comparator, 0);

    // go high on counter empty
    mcpwm_generator_set_action_on_timer_event(
        m.generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH
    ));

    // go low on compare threshold
    mcpwm_generator_set_action_on_compare_event(
        m.generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m.comparator, MCPWM_GEN_ACTION_LOW
    ));

    mcpwm_timer_enable(m.timer);
    mcpwm_timer_start_stop(m.timer, MCPWM_TIMER_START_NO_STOP);

    return m;
}

Expected<std::monostate> Motor::set_angle(float angle) {
    angle = std::max(angle, this->min_degree);
    angle = std::min(angle, this->max_degree);

    float ratio = (angle-min_degree)/(max_degree-min_degree);
    uint32_t compare = ratio * (max_pulsewidth-min_pulsewidth) + min_pulsewidth;
    mcpwm_comparator_set_compare_value(comparator, compare);
    this->current_compare = compare;

    return {};
}

Expected<std::monostate> Motor::set_proportion(float proportion) {
    uint32_t compare = uint32_t ((proportion * (float)this->period));
    compare = std::max(compare, this->min_pulsewidth);
    compare = std::min(compare, this->max_pulsewidth);
    mcpwm_comparator_set_compare_value(comparator, compare);
    this->current_compare = compare;

    return {};
}

Expected<std::monostate> Motor::set_proportion_min_to_max(float proportion) {
    uint32_t compare = min_pulsewidth + (max_pulsewidth - min_pulsewidth) * proportion;
    mcpwm_comparator_set_compare_value(comparator, compare);
    this->current_compare = compare;

    return {};
}

}