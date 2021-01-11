# Software Pin Interrupts

This library implements a software-polling-based version of the "attachInterrupt" function, called
"attachSoftwareInterrupt". This version supports all digital pins on all hardware types, supports
attaching multiple handlers to a single pin, and also has built-in optional debouncing for your input
signals.

# Details

* Supports any digital pin that works with ```digitalRead()``` (up to 12 pins may be used simultaneously)
* Multiple handlers may be attached to a single pin (up to 4 handlers per pin)
* Uses 238 bytes of RAM on Arduino Uno

# Library Reference

## attachSoftwareInterrupt

Registers a function to run whenever a specific event occurs on a specific pin

### Syntax

```attachSoftwareInterrupt(pin, ISR, mode)```

### Parameters

* ```pin```: The pin number to attach this ISR to. Allowed data types: ```int```
* ```ISR```: the ISR to call when the interrupt occurs; this function must take no
  parameters and return nothing. This function is sometimes referred to as an interrupt
  service routine.
* ```mode```: defines when the interrupt should be triggered. Four constants are
  predefined as valid values:

  * **SW_PIN_INTERRUPTS_LOW** to trigger the interrupt whenever the pin is low
  * **SW_PIN_INTERRUPTS_HIGH** to trigger the interrupt whenever the pin is high
  * **CHANGE** to trigger the interrupt whenever the pin changes values
  * **RISING** to trigger the interrupt whenever the pin goes from low to high
  * **FALLING** to trigger the interrupt whenever the pin goes from high to low

## setSoftwareInterruptDebounceMillis

Sets the debounce delay time for all handlers on a specific pin (when the state of a pin
changes, instead of calling your handler immediately, we will wait this long and then
check the state of the pin again, and only call you handler if it really did change after
the delay. Helps when reading a "bouncy" signal like a button).

### Syntax

```setSoftwareInterruptDebounceMillis(pin, debounceMillis)```

### Parameters

* ```pin```: The pin number to set the debounce time for. Allowed data types: ```int```
* ```debounceMillis```: The debounce time for this pin, in milliseconds. Allowed data types: ```int```

## disableSoftwareInterrupt

Disable all interrupts for a given pin (all pins with handlers attached are enabled by default)

### Syntax

```disableSoftwareInterrupt(pin)```


### Parameters

* ```pin```: The pin number to disable interrupts for. Allowed data types: ```int```

## enableSoftwareInterrupt

Enable all interrupts for a given pin (all pins with handlers attached are enabled by default)

### Syntax

```enableSoftwareInterrupt(pin)```


### Parameters

* ```pin```: The pin number to enable interrupts for. Allowed data types: ```int```

## handleSoftwareInterrupts

Main processing function for software pin interrupts. Must be called in ```loop```
if you want any of your attached software interrupts to work!

### Syntax

```handleSoftwareInterrupts()```

### Parameters

None.