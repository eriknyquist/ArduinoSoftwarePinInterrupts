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

/* Bit flag masks for input_debounce_t 'flags' field */
#define ENABLED_BITMASK    (1u)
#define PIN_STATE_BITMASK  (1u << 1u)
#define DEBOUNCING_BITMASK (1u << 2u)

/* Structure to hold a single event callback, and the event it corresponds to */
typedef struct
{
    void (*handler)(void);         // The handler to run
    uint8_t mode;                  // The interrupt mode this handler was registered with
} event_handler_t;

 /* Structure to hold all the information required to debounce/track a single pin */
typedef struct
{
    event_handler_t handlers[SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN];    // Handlers to run on pin state change
    unsigned long last_change_ms;                                        // millis() when last debounce was started
    unsigned debounce_time_ms;                                           // Debounce time in milliseconds
    int pin;                                                             // Pin number
    uint8_t flags;                                                       // Bit flags for this pin. 0=enabled,1=pin state,2=debouncing
    uint8_t handler_count;                                               // Number of handlers registered
} input_debounce_t;


static uint8_t num_pins = 0;
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
        Serial.print((info->flags & PIN_STATE_BITMASK) ? 1 : 0);
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
    for (unsigned i = 0u; i < info->handler_count; i++)
    {
        event_handler_t *event = &info->handlers[i];
        if ((event->mode == CHANGE) ||
            (((info->flags & PIN_STATE_BITMASK) == 0u) && (event->mode == FALLING)) ||
            ((info->flags & PIN_STATE_BITMASK) && (event->mode == RISING)))
        {
#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
            const char *eventstr = "unknown";
            char buf[32];
            switch (event->mode)
            {
                case RISING:
                    eventstr = "RISING";
                    break;
                case FALLING:
                    eventstr = "FALLING";
                    break;
                case CHANGE:
                    eventstr = "CHANGE";
                    break;
            }

            (void) snprintf(buf, sizeof(buf), "%s event occurred", eventstr);
            log_event(info, buf);
#endif
            event->handler();
        }
    }
}

static void check_for_highlow(input_debounce_t *info)
{
    for (unsigned i = 0u; i < info->handler_count; i++)
    {
        event_handler_t *event = &info->handlers[i];
        if ((((info->flags & PIN_STATE_BITMASK) == 0u) && (event->mode == SW_PIN_INTERRUPTS_LOW)) ||
            ((info->flags & PIN_STATE_BITMASK) && (event->mode == SW_PIN_INTERRUPTS_HIGH)))
        {
#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
            const char *eventstr = "unknown";
            char buf[32];
            switch (event->mode)
            {
                case SW_PIN_INTERRUPTS_HIGH:
                    eventstr = "HIGH";
                    break;
                case SW_PIN_INTERRUPTS_LOW:
                    eventstr = "LOW";
                    break;
            }

            (void) snprintf(buf, sizeof(buf), "%s event occurred", eventstr);
            log_event(info, buf);
#endif
            event->handler();
        }
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

        // First handler for this pin, set some initial values
        info->pin = pinNumber;
        info->debounce_time_ms = (unsigned) debounceMs;
        info->last_change_ms = 0;
        info->flags &= ~DEBOUNCING_BITMASK;
        info->flags |= ENABLED_BITMASK;

        if (digitalRead(pinNumber) == HIGH)
        {
            info->flags |= PIN_STATE_BITMASK;
        }
        else
        {
            info->flags &= ~PIN_STATE_BITMASK;
        }
    }

    if (info->handler_count >= SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN)
    {
        // Reached max. number of handlers for this pin, can go no further
        return;
    }

    info->handlers[info->handler_count].handler = pinChangeHandler;
    info->handlers[info->handler_count].mode = interruptMode;
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
            inputs[i].flags &= ~ENABLED_BITMASK;

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
            if (digitalRead(pinNumber) == HIGH)
            {
                inputs[i].flags |= PIN_STATE_BITMASK;
            }
            else
            {
                inputs[i].flags &= ~PIN_STATE_BITMASK;
            }

            inputs[i].flags |= ENABLED_BITMASK;

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
        if ((inputs[i].flags & ENABLED_BITMASK) == 0u)
        {
            // Interrupts are disabled for this pin
            continue;
        }

        if (inputs[i].flags & DEBOUNCING_BITMASK)
        {
            // Waiting for the debounce timer for this pin to expire
            if ((millis() - inputs[i].last_change_ms) >= inputs[i].debounce_time_ms)
            {
                // Timer expired. Re-read pin state and run handler if it changed.
                int new_state = digitalRead(inputs[i].pin);
                int old_state = (inputs[i].flags & PIN_STATE_BITMASK) ? HIGH : LOW;

                if (new_state != old_state)
                {
                    if (new_state == HIGH)
                    {
                        inputs[i].flags |= PIN_STATE_BITMASK;
                    }
                    else
                    {
                        inputs[i].flags &= ~PIN_STATE_BITMASK;
                    }

                    run_handlers(&inputs[i]);
                }

                inputs[i].flags &= ~DEBOUNCING_BITMASK;
            }
        }
        else
        {
            // This pin is not currently being debounced, look for signal change
            int new_state = digitalRead(inputs[i].pin);
            int old_state = (inputs[i].flags & PIN_STATE_BITMASK) ? HIGH : LOW;

            if (new_state != old_state)
            {
                if (inputs[i].debounce_time_ms > 0u)
                {
                    // Non-zero debounce time, start debounce timer for this pin

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
                    log_event(&inputs[i], "state changed, starting debounce");
#endif
                    inputs[i].last_change_ms = millis();
                    inputs[i].flags |= DEBOUNCING_BITMASK;
                }
                else
                {
                    // Debounce time is 0. Run handlers for this pin immediately, if applicable.
                    if (new_state == HIGH)
                    {
                        inputs[i].flags |= PIN_STATE_BITMASK;
                    }
                    else
                    {
                        inputs[i].flags &= ~PIN_STATE_BITMASK;
                    }

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
                    log_event(&inputs[i], "state changed");
#endif
                    run_handlers(&inputs[i]);
                }
            }
        }

        // Run handlers for HIGH/LOW interrupt modes, based on debounced signal for this pin
        check_for_highlow(&inputs[i]);
    }
}
