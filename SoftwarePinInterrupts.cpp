/*
 * SoftwarePinInterrupts.cpp
 *
 * Library for sensing state changes on digital input pins, similar to the attachInterrupt() function
 * but works on all digital pins. Supports optional debouncing, and multiple handlers on a single pin.
 *
 * Created by Erik Nyquist, January 3rd, 2021
 */

#include "Arduino.h"
#include "SoftwarePinInterrupts.h"

 /* Structure to hold all the information required to debounce a single pin */
typedef struct
{
  bool enabled;                                                        // SW interrupts enabled for this pin?
  int pin;                                                             // Pin number
  int interrupt_mode;                                                  // Defines when the interrupt should be triggered
  int debounce_time_ms;                                                // Debounce time in milliseconds
  void (*handlers[SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN])(void);      // Handlers to run on pin state change
  int handler_count;                                                   // Number of handlers registered
  unsigned long last_change_ms;                                        // millis() when last debounce was started
  bool debouncing;                                                     // Currently debouncing?
  int pin_state;                                                       // Current pin state
} input_debounce_t;


static int num_pins = 0;
static input_debounce_t inputs[SW_PIN_INTERRUPTS_MAX_PINS];


static void run_handlers(input_debounce_t *info)
{
    for (int i = 0; i < info->handler_count; i++)
    {
        info->handlers[info->handler_count]();
    }
}

static void attach_pin_interrupt(int pinNumber, void (*pinChangeHandler)(void), int interruptMode, int debounceMs)
{
    input_debounce_t *info = NULL;

    // Is this pin already registered?
    for (int i = 0; i < num_pins; i++)
    {
        if (pinNumber == inputs[i].pin)
        {
            info = &inputs[i];
            break;
        }
    }

    // Pin is not already registered-- need to add a new pin
    if (NULL == info)
    {
        if (num_pins >= SW_PIN_INTERRUPTS_MAX_PINS)
        {
            // Reached max. number of pins, can go no further.
            return;
        }

        info = &inputs[num_pins];
        num_pins += 1;
    }

    if (info->handler_count >= SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN)
    {
        // Reached max. number of handlers for this pin, can go no further
        return;
    }

    info->enabled = true;
    info->pin = pinNumber;
    info->interrupt_mode = interruptMode;
    info->debounce_time_ms = debounceMs;
    info->last_change_ms = 0;
    info->debouncing = false;
    info->pin_state = digitalRead(pinNumber);
    info->handlers[info->handler_count] = pinChangeHandler;
    info->handler_count += 1;
}

/*
 * @see SoftwarePinInterrupts.h
 */
void attachSoftwareInterrupt(int pinNumber, void (*pinChangeHandler)(void), int interruptMode)
{
    attach_pin_interrupt(pinNumber, pinChangeHandler, interruptMode, 0);
}

/*
 * @see SoftwarePinInterrupts.h
 */
void attachSoftwareInterrupt(int pinNumber, void (*pinChangeHandler)(void), int interruptMode, int debounceMs)
{
    attach_pin_interrupt(pinNumber, pinChangeHandler, interruptMode, debounceMs);
}

/*
 * @see SoftwarePinInterrupts.h
 */
void disableSoftwareInterrupt(int pinNumber)
{
    for (int i = 0; i < num_pins; i++)
    {
        if (inputs[i].pin == pinNumber)
        {
            inputs[i].enabled = false;
            break;
        }
    }
}

/*
 * @see SoftwarePinInterrupts.h
 */
void enableSoftwareInterrupt(int pinNumber)
{
    for (int i = 0; i < num_pins; i++)
    {
        if (inputs[i].pin == pinNumber)
        {
            inputs[i].enabled = true;
            break;
        }
    }
}

/*
 * @see SoftwarePinInterrupts.h
 */
void handleSoftwareInterrupts()
{
    // Loop through all of the input signals, looking for changes
    for (int i = 0; i < num_pins; i++)
    {
        if (!inputs[i].enabled)
        {
            continue;
        }

        if (inputs[i].debouncing)
        {
            // Waiting for the debounce timer for this pin to expire
            if ((millis() - inputs[i].last_change_ms) >= inputs[i].debounce_time_ms)
            {
                // Timer expired. Re-read pin state and run handler if it changed.
                int new_state = digitalRead(inputs[i].pin);
                if (new_state != inputs[i].pin_state)
                {
                    inputs[i].pin_state = new_state;

                    if ((inputs[i].interrupt_mode == FALLING) ||
                        (inputs[i].interrupt_mode == RISING) ||
                        (inputs[i].interrupt_mode == CHANGE))
                    {
                        run_handlers(&inputs[i]);
                    }
                }

                inputs[i].debouncing = false;
            }
        }
        else
        {
            // This pin is not currently being debounced, look for signal change
            int new_state = digitalRead(inputs[i].pin);

            if (new_state != inputs[i].pin_state)
            {
                inputs[i].pin_state = new_state;

                if ((inputs[i].interrupt_mode == RISING) && (new_state == LOW))
                {
                    // Signal just fell, nothing to do if mode is RISING
                    continue;
                }

                if ((inputs[i].interrupt_mode == FALLING) && (new_state == HIGH))
                {
                    // Signal just rose, nothing to do if mode is FALLING
                    continue;
                }

                // Pin state changed, start debounce timer or run handler immediately if no debounce time
                if (inputs[i].debounce_time_ms > 0)
                {
                    inputs[i].last_change_ms = millis();
                    inputs[i].debouncing = true;
                }
                else
                {
                    if ((inputs[i].interrupt_mode == FALLING) ||
                        (inputs[i].interrupt_mode == RISING) ||
                        (inputs[i].interrupt_mode == CHANGE))
                    {
                        run_handlers(&inputs[i]);
                    }
                }
            }
        }

        // Run handlers for HIGH/LOW interrupt modes, based on debounced signal for this pin
        if ((inputs[i].interrupt_mode == HIGH) && (inputs[i].pin_state == HIGH))
        {
            run_handlers(&inputs[i]);
        }
        else if ((inputs[i].interrupt_mode == LOW) && (inputs[i].pin_state == LOW))
        {
            run_handlers(&inputs[i]);
        }
    }
}
