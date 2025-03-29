#include "svpwm.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "math.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"


mcpwm_timer_handle_t timer;
mcpwm_oper_handle_t operators[3];
mcpwm_cmpr_handle_t comparators[3];

mcpwm_gen_handle_t gen_A;
mcpwm_gen_handle_t gen_A_inv;
mcpwm_gen_handle_t gen_B;
mcpwm_gen_handle_t gen_B_inv;
mcpwm_gen_handle_t gen_C;
mcpwm_gen_handle_t gen_C_inv;

/*
 * Init PWM stuff so the main setup loop doesn't get too cluttered
 */
void setupPWM() {
    // Initialize pins

    // Init overall timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = RESOLUTION_HZ, 
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, //MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PERIOD_TICKS // https://www.microforum.cc/blogs/entry/43-pwm-resolution/
    };
    mcpwm_new_timer(&timer_config, &timer);

    // Init channels:

    // New API requires operator and comparator
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator should be in the same group of the above timers
    };
    for (int i = 0; i < 3; ++i) {
        mcpwm_new_operator(&operator_config, &operators[i]);
        mcpwm_operator_connect_timer(operators[i], timer);
    }
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };
    for (int i = 0; i < 3; i++) {
        mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]);
        mcpwm_comparator_set_compare_value(comparators[i], 0);
    }

    // Generators
    mcpwm_generator_config_t gen_A_config = {.gen_gpio_num = PWM_A};
    mcpwm_generator_config_t gen_A_inv_config = {.gen_gpio_num = PWM_A_INV};
    mcpwm_generator_config_t gen_B_config = {.gen_gpio_num = PWM_B};
    mcpwm_generator_config_t gen_B_inv_config = {.gen_gpio_num = PWM_B_INV};
    mcpwm_generator_config_t gen_C_config = {.gen_gpio_num = PWM_C};
    mcpwm_generator_config_t gen_C_inv_config = {.gen_gpio_num = PWM_C_INV};

    mcpwm_new_generator(operators[0], &gen_A_config, &gen_A);
    mcpwm_new_generator(operators[0], &gen_A_inv_config, &gen_A_inv);
    mcpwm_new_generator(operators[1], &gen_B_config, &gen_B);
    mcpwm_new_generator(operators[1], &gen_B_inv_config, &gen_B_inv);
    mcpwm_new_generator(operators[2], &gen_C_config, &gen_C);
    mcpwm_new_generator(operators[2], &gen_C_inv_config, &gen_C_inv);

    mcpwm_gen_handle_t generators[3][2] = {{gen_A, gen_A_inv}, {gen_B, gen_B_inv}, {gen_C, gen_C_inv}};

    for (int i = 0; i < 3; i++) {
        // PWM inverted here manually, so no need to do when define generators... I think
        mcpwm_generator_set_actions_on_compare_event(generators[i][0],
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_HIGH),
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators[i], MCPWM_GEN_ACTION_LOW),
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION_END());
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[i][0],
        //     MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        //     MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        // ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[i][0],
        //     MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW),
        //     MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
    }
    mcpwm_dead_time_config_t dt_config = {
        .negedge_delay_ticks = 10
    };
    mcpwm_dead_time_config_t inv_dt_config = {
        .posedge_delay_ticks = 10, 
        .flags.invert_output = true
    };
    for (int i = 0; i < 3; i++) {
        mcpwm_generator_set_dead_time(generators[i][0], generators[i][0], &dt_config);
        mcpwm_generator_set_dead_time(generators[i][0], generators[i][1], &inv_dt_config);
    }

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}

/*
 * Generate sinusoidal PWM
 */
void svpwmTimerCallback() {
    // Test
    mcpwm_comparator_set_compare_value(comparators[0], (uint32_t)(0.4 * 0.5 * PERIOD_TICKS));
    mcpwm_comparator_set_compare_value(comparators[1], (uint32_t)(0.8 * 0.5 * PERIOD_TICKS));


    // // Parameters
    // static float angle = 0.0f;
    // float v_magnitude = 50.0f;
    // float v_freq = 1000.0f;

    // float T1, T2, T0; // T1 and T2 are times in the two states, and T0 is time in zero vector
    // float duty_cycle_a, duty_cycle_b, duty_cycle_c;
    
    // float v_alpha = v_magnitude/POWER_RAIL * cosf(angle);
    // float v_beta = v_magnitude/POWER_RAIL * sinf(angle);
    
    // // Calculate sector (1-6)
    // int sector = 0;
    // float X = v_alpha;
    // float Y = (-v_alpha + sqrtf(3) * v_beta) / 2;
    // float Z = (-v_alpha - sqrtf(3) * v_beta) / 2;
    
    // if (X > 0) sector |= 1;
    // if (Y > 0) sector |= 2;
    // if (Z > 0) sector |= 4;

    // // Compute T1 and T2 based on sector
    // switch (sector) {
    //     case 1:  // Sector 1
    //         T1 = X;
    //         T2 = Y;
    //         break;
    //     case 2:  // Sector 2
    //         T1 = Y;
    //         T2 = -Z;
    //         break;
    //     case 3:  // Sector 3
    //         T1 = -Z;
    //         T2 = X;
    //         break;
    //     case 4:  // Sector 4
    //         T1 = -X;
    //         T2 = Z;
    //         break;
    //     case 5:  // Sector 5
    //         T1 = Z;
    //         T2 = -Y;
    //         break;
    //     case 6:  // Sector 6
    //         T1 = -Y;
    //         T2 = -X;
    //         break;
    //     default:
    //         T1 = 0;
    //         T2 = 0;
    //         break;
    // }

    // // Compute T0 time
    // T0 = 1 - (T1 + T2);

    // // Calculate duty cycles
    // duty_cycle_a = (T1 + T2 + T0 / 2) * 100;
    // duty_cycle_b = (T2 + T0 / 2) * 100;
    // duty_cycle_c = (T0 / 2) * 100;

    // // Update PWM duty cycle
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle_a);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle_b);
    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_cycle_c);

    // // Increment the angle
    // angle += 2 * PI * v_freq / LOOP_RATE;
    // if (angle > 2 * PI) {
    //     angle -= 2 * PI;
    // }
}

void startSvpwmTask() {
    const esp_timer_create_args_t timer_args = {
        .callback = &svpwmTimerCallback,
        .name = "svpwm_timer"
    };
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 1000000 / LOOP_RATE); // Note this is rounded to integer!!
}









