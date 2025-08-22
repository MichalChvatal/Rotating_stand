/**
 * @file main.c
 * @author Michal Chvatal
 * @copyright 2025, Michal Chvatal. All rights reserved.
 * @license BSD 3-Clause License
 *
 * @brief This program controls a 28byj-48 stepper motor using an ATtiny microcontroller.
 *
 * @details The program configures the microcontroller's fuses, GPIO pins, and Timer/Counter0 to drive a 28byj-48 stepper motor.
 * Three different control modes are available: single-phase (wave drive), dual-phase (full drive), and half-step.
 * The appropriate control mode is selected at compile time using preprocessor directives. The program operates in
 * a low-power state by utilizing the microcontroller's sleep mode and disabling unused peripherals. The motor steps
 * are controlled by a Timer0 interrupt service routine.
 *
 * @note This program is designed for use with an ATtiny microcontroller with a clock speed of 4.8 MHz.
 *
 * **Fuses Configuration:**
 * - Low Fuse Byte: 0x79
 * - High Fuse Byte: 0xFF
 *
 * **Fuses settings:**
 * - Internal 4.8 MHz clock
 * - Disable internal clock division by 8
 * - Disable Brown-out Detection (BODLEVEL) to reduce power consumption
 * - Disable Watchdog Timer to reduce power consumption
 *
 * **Power Consumption:**
 * - MCU (ATtiny) consumption: 2.3 mA
 *
 * **Stepper Motor (28byj-48):**
 * - Driver: ULN2003
 * - Steps per cycle: 2048
 * - Voltage: 5V
 * - Degree per step: 0.176 °
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// Select the motor control type
#define SINGLE_PHASE_CONTROL (1) /**< Wave drive - Full step; single-phase control. Consumption: 148 mA. */
#define DUAL_PHASE_CONTROL   (0) /**< Full drive - Full step; higher consumption (approx. 260 mA) but higher torque. */
#define HALF_STEP            (0) /**< Half-step drive; higher consumption (approx. 200 mA) for smoother motion. */

// F_CPU 4800000 // 4.8MHz - for informational purposes only

#define MAX_PHASE_COUNT (4)

/**
 * @name Configuration for different drive modes
 * @{
 */
#if (SINGLE_PHASE_CONTROL == 1)
#define MAX_STEP_COUNT (4)
#define TIMER_CLOCK    (0x04) /**< 256 divider for Timer0. */
#define TIMER_VALUE    (74)   /**< Compare value for Timer0 to achieve 250 Hz step frequency. */
/** @brief Stepping sequence for single-phase control. */
volatile uint8_t step_array[MAX_STEP_COUNT][MAX_PHASE_COUNT] = {{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};
#elif (DUAL_PHASE_CONTROL == 1)
#define MAX_STEP_COUNT (4)
#define TIMER_CLOCK    (0x04) /**< 256 divider for Timer0. */
#define TIMER_VALUE    (37)   /**< Compare value for Timer0 to achieve 250 Hz step frequency. */
/** @brief Stepping sequence for dual-phase control. */
volatile uint8_t step_array[MAX_STEP_COUNT][MAX_PHASE_COUNT] = {{1, 1, 0, 0},{0, 1, 1, 0},{0, 0, 1, 1},{1, 0, 0, 1}};
#elif (HALF_STEP == 1)
#define MAX_STEP_COUNT (8)
#define TIMER_CLOCK    (0x03) /**< 64 divider for Timer0. */
#define TIMER_VALUE    (149)  /**< Compare value for Timer0 to achieve 500 Hz step frequency. */
/** @brief Stepping sequence for half-step control. */
volatile uint8_t step_array[MAX_STEP_COUNT][MAX_PHASE_COUNT] = {{1, 0, 0, 0},{1, 0, 0, 1},{0, 0, 0, 1},{0, 0, 1, 1},{0, 0, 1, 0},{0, 1, 1, 0},{0, 1, 0, 0},{1, 1, 0, 0}};
#else
#error "Choose one of the control options by setting a macro to 1."
#endif
/** @} */

/** @brief Array mapping motor phases to their corresponding GPIO pins on PORTB. */
volatile uint8_t step_gpio[MAX_PHASE_COUNT] = {PORTB0, PORTB1, PORTB2, PORTB3}; // {IN1, IN2, IN3, IN4}
/** @brief Current index in the step sequence array. */
volatile uint8_t step_index = 0;

/**
 * @brief Interrupt Service Routine (ISR) for Timer/Counter0 Compare Match A.
 *
 * @details This ISR is triggered by Timer0 and performs one step of the stepper motor
 * by updating the state of the output pins on PORTB according to the selected step sequence.
 * The step flag is cleared automatically after the routine is executed.
 */
ISR(TIM0_COMPA_vect)
{
    volatile uint8_t tmp = 0x00;

    //PINB = (1 << PINB4); // measure timer cycle (timer period is 60us, ISR takes 40us) - for debugging

    // Copy the current state of PORTB
    tmp = PORTB;

    // Perform one step by updating the motor phases
    for(uint8_t phase = 0; phase < MAX_PHASE_COUNT; phase++)
    {
        if(step_array[step_index][phase] == 1)
        {
            // Set the corresponding pin
            tmp |= (1 << step_gpio[phase]);
        }
        else
        {
            // Clear the corresponding pin
            tmp &= ~(1 << step_gpio[phase]);
        }
    }

    PORTB = tmp; // Write data in one atomic operation for synchronized outputs

    //PINB = (1 << PINB4); // measure timer cycle (timer period is 60us, ISR takes 40us) - for debugging

    // Increment the step index and loop back to the start if the end of the sequence is reached
    step_index++;
    if(step_index == MAX_STEP_COUNT)
    {
        step_index = 0;
    }
}

/**
 * @brief Configures the General Purpose Input/Output (GPIO) pins.
 *
 * @details Sets PORTB pins 0-3 as outputs for controlling the stepper motor phases.
 * Initializes all configured output pins to a logical low state (0).
 */
void gpio_config(void)
{
    // Set pins 0-4 of PORTB as outputs. PINB4 is used for debugging purposes.
    DDRB   = ((1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (1 << DDB4));
    // Set all configured output pins to a logical low state.
    PORTB &= ((~(1 << PORTB0)) & (~(1 << PORTB1)) & (~(1 << PORTB2)) & (~(1 << PORTB3)) & (~(1 << PORTB4)));
}

/**
 * @brief Configures Timer/Counter0 for stepper motor control.
 *
 * @details Configures Timer/Counter0 to operate in CTC (Clear Timer on Compare Match) mode.
 * The timer clock prescaler and compare value are set based on the selected control mode
 * to determine the stepping frequency. An interrupt on Compare Match A is enabled to drive the motor.
 */
void timer_config(void)
{
    TCCR0A = 0x02;           // Set CTC mode
    TCCR0B = TIMER_CLOCK;    // Set the timer clock prescaler
    TCNT0  = 0x00;           // Clear the counter value
    OCR0A  = TIMER_VALUE;    // Set the compare value for the desired step speed
    OCR0B  = 0xFF;           // Ensure OCR0B does not trigger an interrupt
    TIMSK0 |= (1 << OCIE0A); // Enable interrupt for Compare Match A
}

/**
 * @brief Main function of the program.
 *
 * @details Initializes the GPIO and timer configurations. Disables global interrupts before configuration
 * and re-enables them afterward. The program enters an infinite loop, where it puts the microcontroller
 * into sleep mode (Idle mode) to conserve power. The system is woken up by the Timer0 interrupt.
 */
int main(void)
{
    cli(); // Disable global interrupts for safe configuration
    gpio_config();
    timer_config();
    sei(); // Enable global interrupts to allow the timer ISR to run

    // Enable sleep mode (Idle mode) to conserve power
    MCUCR |= (1 << SE);
    ACSR  |= (1 << ACD); // Disable the Analog Comparator to further reduce consumption

    while (1)
    {
        sleep_cpu(); // Put the microcontroller to sleep
    }
}