/*
 * SoftwarePinInterrupts.cpp
 *
 * Library for sensing state changes on digital input pins, similar to the attachInterrupt() function
 * but works on all digital pins. Supports multiple handlers on a single pin, and debouncing.
 *
 * Created by Erik Nyquist, January 3rd, 2021
 */

#include "Arduino.h"
#include "SoftwarePinInterrupts.h"

 /* Structure to hold all the information required to debounce/track a single pin */
typedef struct
{
    bool enabled;                                                        // SW interrupts enabled for this pin?
    int pin;                                                             // Pin number
    int interrupt_mode;                                                  // Defines when the interrupt should be triggered
    unsigned debounce_time_ms;                                           // Debounce time in milliseconds
    void (*handlers[SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN])(void);      // Handlers to run on pin state change
    unsigned handler_count;                                              // Number of handlers registered
    unsigned long last_change_ms;                                        // millis() when last debounce was started
    bool debouncing;                                                     // Currently debouncing?
    int pin_state;                                                       // Current pin state
} input_debounce_t;


static int num_pins = 0;
static input_debounce_t inputs[SW_PIN_INTERRUPTS_MAX_PINS];

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
static void log_event(input_debounce_t *info, const char *msg)
{
    unsigned long ms = millis();
    unsigned long secs = ms / 1000u;
    unsigned long remaining_ms = ms - (secs * 1000);

    Serial.print("[");
    Serial.print(secs);
    Serial.print(".");
    Serial.print(remaining_ms);
    Serial.print("] [SoftwarePinInterrupts] ");
    if (NULL != info)
    {
        Serial.print("pin ");
        Serial.print(info->pin);
        Serial.print(", state=");
        Serial.print((info->pin_state == HIGH) ? 1 : 0);
    }

    if (NULL != msg)
    {
        Serial.print(": ");
        Serial.print(msg);
    }

    Serial.println();
}
#endif // SW_PIN_INTERRUPTS_SERIAL_DEBUG

static void run_handlers(input_debounce_t *info)
{
#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
    const char *event = "unknown";
    char buf[32];
    switch (info->interrupt_mode)
    {
        case SW_PIN_INTERRUPTS_HIGH:
            event = "HIGH";
            break;
        case SW_PIN_INTERRUPTS_LOW:
            event = "LOW";
            break;
        case RISING:
            event = "RISING";
            break;
        case FALLING:
            event = "FALLING";
            break;
        case CHANGE:
            event = "CHANGE";
            break;
    }

    (void) snprintf(buf, sizeof(buf), "%s event occurred", event);
    log_event(info, buf);
#endif

    for (unsigned i = 0u; i < info->handler_count; i++)
    {
        info->handlers[i]();
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
    info->debounce_time_ms = (unsigned) debounceMs;
    info->last_change_ms = 0;
    info->debouncing = false;
    info->pin_state = digitalRead(pinNumber);
    info->handlers[info->handler_count] = pinChangeHandler;
    info->handler_count += 1u;

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
    log_event(info, "registering handler");
#endif
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

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
            log_event(&inputs[i], "input disabled");
#endif
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
            inputs[i].pin_state = digitalRead(inputs[i].pin);
            inputs[i].enabled = true;

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
            log_event(&inputs[i], "input enabled");
#endif
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

                    if (((inputs[i].interrupt_mode == FALLING) && (new_state == LOW))||
                        ((inputs[i].interrupt_mode == RISING) && (new_state == HIGH)) ||
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
                if (inputs[i].debounce_time_ms > 0u)
                {
                    // Non-zero debounce time, start debounce timer for this pin

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
                    log_event(&inputs[i], "state changed, starting debounce");
#endif
                    inputs[i].last_change_ms = millis();
                    inputs[i].debouncing = true;
                }
                else
                {
                    // Debounce time is 0. Run handlers for this pin immediately, if applicable.
                    if ((inputs[i].interrupt_mode == FALLING) ||
                        (inputs[i].interrupt_mode == RISING) ||
                        (inputs[i].interrupt_mode == CHANGE))
                    {
                        inputs[i].pin_state = new_state;

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
                        log_event(&inputs[i], "state changed");
#endif
                        run_handlers(&inputs[i]);
                    }
                }
            }
        }

        // Run handlers for HIGH/LOW interrupt modes, based on debounced signal for this pin
        if ((inputs[i].interrupt_mode == SW_PIN_INTERRUPTS_HIGH) && (inputs[i].pin_state == HIGH))
        {
            run_handlers(&inputs[i]);
        }
        else if ((inputs[i].interrupt_mode == SW_PIN_INTERRUPTS_LOW) && (inputs[i].pin_state == LOW))
        {
            run_handlers(&inputs[i]);
        }
    }
}
