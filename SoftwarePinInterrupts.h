/*
 * SoftwarePinInterrupts.cpp
 *
 * This library implements a software-polling-based version of the "attachInterrupt" function,
 * called "attachSoftwareInterrupt". This version supports all digital pins on all hardware types,
 * supports attaching multiple handlers to a single pin, and also has built-in optional debouncing.
 *
 * Created by Erik Nyquist (eknyquist@gmail.com), January 2021
 */

#ifndef SoftwarePinInterrupts_h
#define SoftwarePinInterrupts_h

/*
 * Defines how many pins can have have software interrupts attached. Increasing has a memory penalty.
 * Must not be larger than 63.
 */
#ifndef SW_PIN_INTERRUPTS_MAX_PINS
#define SW_PIN_INTERRUPTS_MAX_PINS 12u
#endif // SW_PIN_INTERRUPTS_MAX_PINS

/*
 * Defines how many handler functions can be attached to a single pin. Increasing has a memory penalty.
 * Must not be larger than 31.
 */
#ifndef SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN
#define SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN 4u
#endif // SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN

/*
 * Prints extra debugging information on the serial port
 */
#ifndef SW_PIN_INTERRUPTS_SERIAL_DEBUG
#define SW_PIN_INTERRUPTS_SERIAL_DEBUG 0
#endif // SW_PIN_INTERRUPTS_SERIAL_DEBUG

#define SW_PIN_INTERRUPTS_HIGH 4
#define SW_PIN_INTERRUPTS_LOW 5

/*
 * Attach a handler function to the provided pin with the provided interrupt mode
 *
 * @param pinNumber         Pin number to attach handler to
 * @param pinChangeHandler  Function to run on pin state change
 * @param interruptMode     Interrupt mode (must be RISING, FALLING, CHANGE, LOW or HIGH)
 */
void attachSoftwareInterrupt(int pinNumber, void (*pinChangeHandler)(void), int interruptMode);

/*
 * Set the debounce time for triggering software interrupts on a particular pin.
 *
 * @param pinNumber         Pin number to set debounce time for.
 * @param debounceMillis    Debounce time in milliseconds. Max. valid value is 1023.
 */
void setSoftwareInterruptDebounceMillis(int pinNumber, int debounceMillis);

/*
 * Temporarily disable interrupts for a pin that has software interrupts attached to it.
 *
 * @param pinNumber  Pin number to disable interrupts for
 */
void disableSoftwareInterrupt(int pinNumber);

/*
 * Re-enable interrupts for a pin that has software interrupts attached to it.
 *
 * @param pinNumber  Pin number to re-enable interrupts for
 */
void enableSoftwareInterrupt(int pinNumber);

/*
 * Main processing function for software interrupts. This must be called in "loop()".
 */
void handleSoftwareInterrupts();

#endif // SoftwarePinInterrupts_h
