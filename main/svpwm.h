#pragma once

#define PWM_A GPIO_NUM_15
#define PWM_A_INV GPIO_NUM_2
#define PWM_B GPIO_NUM_4
#define PWM_B_INV GPIO_NUM_16
#define PWM_C GPIO_NUM_17
#define PWM_C_INV GPIO_NUM_5
#define POT ADC_CHANNEL_0 

#define PI 3.14159265358979323846f

#define LOOP_RATE 2000                          // Control loop frequency (Hz)
#define PWM_FREQ 10000                          // Hz
#define RESOLUTION_HZ 10000000                  // 10MHz, 1 tick = 0.1us
#define PERIOD_TICKS RESOLUTION_HZ/PWM_FREQ     // 1000 * 0.1us = 100us, 10KHz
#define DEAD_TM 10                              // ns between high and low side switching
#define POWER_RAIL 100.0f                       // V

#define POT_READ_HZ 5

void setupPWM();
void svpwmTimerCallback();
void startSvpwmTask();

void readPotentiometerTask(void *pvParameters);
void setupPotentiometer();
