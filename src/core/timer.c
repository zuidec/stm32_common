#include "../../include/core/timer.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

// FREQ = CPU_FREQ / (PRESCALER -1 ) * (ARR_VALUE - 1)
// CPU_FREQ = 84 000 000
#define PRESCALER       (84)
#define ARR_VALUE       (1000)

void timer_setup(void)  {
    
    // Start TIM2
    rcc_periph_clock_enable(RCC_TIM2);

    // Configure timer 2 to no division, edge, count up
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Configure PWN mode to output channel 1
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);

    // Enable PWN
    timer_enable_counter(TIM2);
    timer_enable_oc_output(TIM2,TIM_OC1);

    // Configure frequency and resolution
    timer_set_prescaler(TIM2, PRESCALER -1);
    timer_set_period(TIM2, ARR_VALUE - 1);

}

void timer_pwm_set_duty_cycle(float duty_cycle) {

    // CCR = ARR_VALUE * (duty_cycle * 100)
    
    // Need to check for numbers >1 and <0 ?? not sure about this
    if(duty_cycle > 1.0f || duty_cycle < 0.0f)  {
       // return; // Return if duty cycle is not valid
    }

    // Calculate the right CCR value based on the duty cycle
    const float raw_ccr_value = (float)ARR_VALUE * (duty_cycle * 100.0f);

    // Set the new value
    timer_set_oc_value(TIM2, TIM_OC1, (uint32_t)raw_ccr_value);
}