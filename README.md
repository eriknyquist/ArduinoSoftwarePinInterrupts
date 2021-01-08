# Software Pin Interrupts

This library implements a software-polling-based version of the "attachInterrupt" function, called
"attachSoftwareInterrupt". This version supports all digital pins on all hardware types, supports
attaching multiple handlers to a single pin, and also has built-in optional debouncing for your input
signals.

## attachSoftwareInterrupt

### Syntax

```attachSoftwareInterrupt(pin, ISR, mode)```

### Parameters

* ```pin```: The pin number to attach this ISR to
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
