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


#define SW_PIN_INTERRUPTS_MAX_DEBOUNCE_MS 1024u

#define SW_PIN_INTERRUPTS_MAX_PIN_NUM 64u

// Bitmasks for "data1" field
#define DEBOUNCE_TIME_BITMASK    (0b0000001111111111)
#define PIN_NUM_BITMASK          (0b1111110000000000)

// Bit field positions for "data1" field
#define DEBOUNCE_TIME_POS        (0u)
#define PIN_NUM_POS              (10u)

// Bitmasks for "data" field
#define HANDLER_COUNT_BITMASK    (0b00011111)
#define ENABLED_BITMASK          (0b00100000)
#define PIN_STATE_BITMASK        (0b01000000)
#define DEBOUNCING_BITMASK       (0b10000000)

// Helper macros to read fields from a pin_info_t instance
#define GET_PIN_NUMBER(info)     ((int) (((info)->data1 & PIN_NUM_BITMASK) >> PIN_NUM_POS))
#define GET_DEBOUNCE_TIME(info)  ((unsigned long) ((info)->data1 & DEBOUNCE_TIME_BITMASK))
#define GET_HANDLER_COUNT(info)  ((uint8_t) ((info)->data2 & HANDLER_COUNT_BITMASK))
#define GET_PIN_STATE(info)      (((info)->data2 & PIN_STATE_BITMASK) > 0u)
#define IS_PIN_ENABLED(info)     (((info)->data2 & ENABLED_BITMASK) > 0u)
#define IS_PIN_DEBOUNCING(info)  (((info)->data2 & DEBOUNCING_BITMASK) > 0u)

// Helper macros to set fields on a pin_info_t instance
#define SET_DEBOUNCE_TIME(info, ms)   ((info)->data1 |= ((info)->data1 & ~DEBOUNCE_TIME_BITMASK) | (((uint16_t) (ms)) & DEBOUNCE_TIME_BITMASK))
#define SET_PIN_NUMBER(info, pin)     ((info)->data1 |= ((info)->data1 & ~PIN_NUM_BITMASK) | ((((uint16_t) (pin)) << PIN_NUM_POS) & PIN_NUM_BITMASK))
#define SET_HANDLER_COUNT(info, c)    ((info)->data2 = ((info)->data2 & ~HANDLER_COUNT_BITMASK) | (((uint8_t) (c)) & HANDLER_COUNT_BITMASK))
#define SET_PIN_STATE(info, s)        {if ((s) == HIGH) (info)->data2 |= PIN_STATE_BITMASK; else (info)->data2 &= ~PIN_STATE_BITMASK;}
#define SET_PIN_ENABLED(info, e)      {if (e) (info)->data2 |= ENABLED_BITMASK; else (info)->data2 &= ~ENABLED_BITMASK;}
#define SET_PIN_DEBOUNCING(info, s)   {if (s) (info)->data2 |= DEBOUNCING_BITMASK; else (info)->data2 &= ~DEBOUNCING_BITMASK;}

/* Structure to hold a single event callback, and the event it corresponds to */
typedef struct
{
    void (*handler)(void);         // The handler to run
    uint8_t mode;                  // The interrupt mode this handler was registered with
} interrupt_handler_t;

 /* Structure to hold all the information required to debounce/track a single pin.
  * In order to save as much RAM as possible, some integer fields are packed with multiple values,
  * using only many bits as required. */
typedef struct
{
    interrupt_handler_t handlers[SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN]; // Handlers to run on pin state change
    unsigned long last_change_ms;                                         // millis() when last debounce was started

    /* [bits 0-9: debounce time, milliseconds] [bits 10-15: pin number]*/
    uint16_t data1;

    /* [bits 0-4: handler count] [bit 5: interrupts enabled] [bit 6: pin state] [bit 7: pin debouncing] */
    uint8_t data2;
} pin_info_t;

static uint8_t num_pins = 0;
static pin_info_t inputs[SW_PIN_INTERRUPTS_MAX_PINS];


#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
static void log_event(pin_info_t *info, const char *msg)
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
        Serial.print(GET_PIN_NUMBER(info));
        Serial.print(", state=");
        Serial.print(GET_PIN_STATE(info));
    }

    if (NULL != msg)
    {
        Serial.print(": ");
        Serial.print(msg);
    }

    Serial.println();
}
#endif // SW_PIN_INTERRUPTS_SERIAL_DEBUG

static void run_handlers(pin_info_t *info)
{
    uint16_t handler_count = GET_HANDLER_COUNT(info);
    for (uint16_t i = 0u; i < handler_count; i++)
    {
        interrupt_handler_t *event = &info->handlers[i];
        if ((event->mode == CHANGE) ||
            ((GET_PIN_STATE(info) == 0u) && (event->mode == FALLING)) ||
            (GET_PIN_STATE(info) && (event->mode == RISING)))
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

static void check_for_highlow(pin_info_t *info)
{
    uint16_t handler_count = GET_HANDLER_COUNT(info);
    for (uint16_t i = 0u; i < handler_count; i++)
    {
        interrupt_handler_t *event = &info->handlers[i];
        if (((GET_PIN_STATE(info) == 0u) && (event->mode == SW_PIN_INTERRUPTS_LOW)) ||
            (GET_PIN_STATE(info) && (event->mode == SW_PIN_INTERRUPTS_HIGH)))
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

static void init_pin_info(pin_info_t *info, int pinNumber, int debounceMs)
{
    SET_PIN_NUMBER(info, pinNumber);
    SET_DEBOUNCE_TIME(info, debounceMs);
    SET_HANDLER_COUNT(info, 0u);
    SET_PIN_DEBOUNCING(info, 0u);
    SET_PIN_ENABLED(info, 1u);
    SET_PIN_STATE(info, digitalRead(pinNumber));
    info->last_change_ms = 0u;
}

static pin_info_t *get_pin_with_init_values(int pinNumber, int debounceMs)
{
    pin_info_t *info = NULL;

    // Is this pin already registered?
    for (int i = 0; i < num_pins; i++)
    {
        if (pinNumber == GET_PIN_NUMBER(&inputs[i]))
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
            return NULL;
        }

        info = &inputs[num_pins];
        num_pins += 1;

        // First handler for this pin, set some initial values
        init_pin_info(info, pinNumber, debounceMs);
    }

    return info;
}

/*
 * @see SoftwarePinInterrupts.h
 */
static void attachSoftwareInterrupt(int pinNumber, void (*pinChangeHandler)(void), int interruptMode)
{
    pin_info_t *info = get_pin_with_init_values(pinNumber, 0);

    if (NULL == info)
    {
        return;
    }

    uint16_t count = GET_HANDLER_COUNT(info);

    if (count >= SW_PIN_INTERRUPTS_MAX_HANDLERS_PER_PIN)
    {
        // Reached max. number of handlers for this pin, can go no further
        return;
    }

    info->handlers[count].handler = pinChangeHandler;
    info->handlers[count].mode = interruptMode;

    // Increment handler count
    SET_HANDLER_COUNT(info, count + 1u);

#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
    log_event(info, "registering handler");
#endif
}

/*
 * @see SoftwarePinInterrupts.h
 */
void setSoftwareInterruptDebounceMillis(int pinNumber, int debounceMillis)
{
    if (debounceMillis < 0)
    {
        debounceMillis = 0;
    }
    else if (debounceMillis > SW_PIN_INTERRUPTS_MAX_DEBOUNCE_MS)
    {
        debounceMillis = SW_PIN_INTERRUPTS_MAX_DEBOUNCE_MS;
    }

    pin_info_t *info = get_pin_with_init_values(pinNumber, 0u);

    if (NULL == info)
    {
        return;
    }

    SET_DEBOUNCE_TIME(info, debounceMillis);
}

/*
 * @see SoftwarePinInterrupts.h
 */
void disableSoftwareInterrupt(int pinNumber)
{
    for (int i = 0; i < num_pins; i++)
    {
        if (GET_PIN_NUMBER(&inputs[i]) == pinNumber)
        {
            SET_PIN_ENABLED(&inputs[i], 0u);

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
        if (GET_PIN_NUMBER(&inputs[i]) == pinNumber)
        {
            SET_PIN_STATE(&inputs[i], digitalRead(pinNumber));
            SET_PIN_ENABLED(&inputs[i], 1u);

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
    for (uint8_t i = 0u; i < num_pins; i++)
    {
        if (!IS_PIN_ENABLED(&inputs[i]))
        {
            // Interrupts are disabled for this pin
            continue;
        }

        if (IS_PIN_DEBOUNCING(&inputs[i]))
        {
            // Waiting for the debounce timer for this pin to expire
            if ((millis() - inputs[i].last_change_ms) >= GET_DEBOUNCE_TIME(&inputs[i]))
            {
                // Timer expired. Re-read pin state and run handler if it changed.
                int new_state = digitalRead(GET_PIN_NUMBER(&inputs[i]));
                int old_state = (GET_PIN_STATE(&inputs[i])) ? HIGH : LOW;

                if (new_state != old_state)
                {
                    SET_PIN_STATE(&inputs[i], new_state);
                    run_handlers(&inputs[i]);
                }

                SET_PIN_DEBOUNCING(&inputs[i], 0u);
            }
        }
        else
        {
            // This pin is not currently being debounced, look for signal change
            int new_state = digitalRead(GET_PIN_NUMBER(&inputs[i]));
            int old_state = (GET_PIN_STATE(&inputs[i])) ? HIGH : LOW;

            if (new_state != old_state)
            {
                if (GET_DEBOUNCE_TIME(&inputs[i]) > 0u)
                {
                    // Non-zero debounce time, start debounce timer for this pin
#if SW_PIN_INTERRUPTS_SERIAL_DEBUG
                    log_event(&inputs[i], "state changed, starting debounce");
#endif
                    inputs[i].last_change_ms = millis();
                    SET_PIN_DEBOUNCING(&inputs[i], 1u);
                }
                else
                {
                    // Debounce time is 0. Run handlers for this pin immediately, if applicable.
                    SET_PIN_STATE(&inputs[i], new_state);

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
