# F1/10 ATmega servo-board -- Development documentation

## Important design decisions

Our primary goal was to achieve most precise readings of input signals from
receiver. (As long as performance of other parts is not affected by this.)

So we reserved two registers `r2` and `r16` exclusively to be used by interrupt
handlers written in assembly. And we are not using interrupts for anything
else.

So if you modify code, be sure to include `global.h` as the first thing inside
every compiled source file. This tells the compiler to not use those registers
by any function written in C. If you fail to reserve them, anything can happen.

Additionaly, lowest two bits of `GPIOR0` and while `GPIOR1` and `GPIOR2` are
used. Those register are quite convenient because we can access them quickly.
So we are reducing time needed by interrupt handlers ant hus slightly
increasing measurement precision.


## Overall architecture

Most of the tasks are run periodically in fixed schedule (see while loop inside
`main()` function -- near end of file `main.c`). Timing constraints of those
tasks is quite lose. So there is still plenty of space to implement more
complex stuff.

Only part, that is not run fully from there is the input capture part of the
code (written in assembly).

### Input capture

Associated files:
 - `input_capture.c`
 - `input_capture_asm.S`
 - `input_capture.h`
 - `global.h`
 - and partly `hw.h`

Note, that we describe the measurement just for the first channel. Second works
in the same way, only we use interrupt `INT1`, Timer2, second lowest bit
inside `GPIOR0` for storing the rising/falling edge of interrupt and `GPIOR2` for
upper byte (instead of `INT0`, Timer0, lowest bit of `GPIOR0` and `GPIOR1`).


Start of measurement is done by running function `input_capture_0_single_shot()`
(file `input_capture.c`). Thhis enables sternal interrupt on rising edge using INT0.

Then the hardware waits until interrupt `INT0_vect:` (file
`input_capture_asm.S`) is called. We save `SREG` to reserved register and check
lowest bit in `GPIOR0`. It will be zero at this state, so we quickly branch to
the label `int0_rising:`. There we restart 8-bit Timer0, switch interrupt for
falling edge, enable Timer0 and set the lowest bit in GPIOR0 to one, so we will
run code for falling edge the next time.

Overflow of Timer0 is used as software extension of Timer0 to 16-bit. Upper
half is stored inside `GPIOR1` register.

When the falling edge is detected, we want just to stop the Timer0, disable
INT0 interrupt and write whole 16-bit state to `counter0` variable in RAM. The
code is a bit more complicated, because we need to handle situations, where the
falling edge happens at the same time as the timer overflow.

 - If the overflow occurs prior to reading of `TCNT0` (but was not handled
   yet), we need to increase upper byte by one.
 - It is more complicated, because this overflow can happen at any time during
   execution of `INT0_vect` handler.

So whole falling edge interrupt handler works like this:

 1. Store `TCNT0` to reserved register `r16`
 2. If there is pending overflow interrupt on Timer0 (bit `TOV0` inside `TIFR0`) and `r16` is lower than 0x80, inrement `GPIOR1` by one.
 3. Store 16-bit value `GPIOR1:r16` to `counter0`.

Those steps are somehow "interleaved" in the real code: First we store `TCNT0`
to lower part of `counter0` and than do all the checks. This is done, because
we can reuse `r16` and save a few instruction cycles.

This is important if the second channel is still running and both falling edges
happen at the same time. We do not want to delay handling of the second
channel's interrupt vector.

Also note, that we are not interacting with input_capture susbsystem directly
from other parts of the code. We use macros defined in `hw.h`, so it is
possible to simply swap both channels.

### Generating the PWM

Associated files:
 - `servo.c`
 - `servo.h`
 - and partly `hw.h`

This subsystem is quite straight-forward. We are using 16-bit Timer1 with two
hardware generated PWM.

Timer1 is set to work in dual-slope mode: It counts from 0 to `ICR1` and then
back to 0. PWM outputs are changed anytime the timer value reaches `OCR1A`/`OCR1B`.
Output is set to one during downcounting and to zero during up-counting.

Values of `OCR1A` and `OCR1B` are double-buffered. They are updated only at top
of the timer. This allows us to run the simulation of Traxxas driver: we know
exactly, which value will get out of the microcontroller.

We are also using this timer for synchronizing some higher level tasks with
outputs: This module provides macros `SERVO_OVERFLOW` and
`SERVO_OVERFLOW_CLEAR()`. Main loop is using this to run specific tasks only
when the timer overflows (it is at the top).

Desired PWM length can be set directly by writing to `OCR1A` and `OCR1B`. But
it is better to use macros `OCR1_ANGLE` and `OCR1_SPEED` defined in `hw.h`.
This allows us to swap the output channels there.

### Traxxas driver simulation (speed_controller)

Associated files:
 - `speed_controller.c`
 - `speed_controller.h`

When the speed_controller is working, nobody should write to `OCR1A`/`OCR1B`
directly. It is better to use functions:

```
uint8_t speed_controller_try_set_speed_us(uint16_t speed_us);
uint8_t speed_controller_try_set_angle_us(uint16_t angle_us);
```

Those functions are intended to be run from main loop. It is possible, that
those function will not succeed: They refuse to update the output if we are too
close to timer overflow, or when the tasks happening on overflow were not yet
run. This allows us to succesfully obtain the real values generated in this
step and do the Traxxas simulation.

Speed controller expects, that you will run its function
`void speed_controller_simulate_state(uint16_t set_speed);` on each Timer1
interrupt. So that you provide it with length of each signal which is
generated by the microcontroller.

#### Simulation

Driver may be in five states:

 - Neutral 1.
 - Froward
 - Brake
 - Neutral 2.
 - Backward

Based on observation, the internal state machine of Traxxas driver works like
this:

```
                      |            action:
                      |    +          N         -
      previous state: *-----------------------------
0x01:    neutral-1    | forward   neutral-1   brake
0x01:    forward      | forward   neutral-1   brake
0x02:    brake        | forward   neutral-2   brake
0x04:    neutral-2    | forward   neutral-2   backward
0x04:    backward     | forward   neutral-2   backward
```

For practical implementation, we do not need to distinguish between Neutral
1. and Forward and between Neutral 2. and Backward.


The Traxxas driver will not switch between the states immediately. There is
some sort of filtering. We expect, that we have to send the same action four
times in a row to succesfully do the transition inside the driver. If the
action is changed earlier, we can't be sure, if the transition already happened
or not.

So we are actually using bits for each state -- our simulated driver can be in
multiple states at the same time. And similar problem is with actions: We can't
be sure, if some values near neutral/forward threshold are interpreted as
forward or neutral action. (And same applies also to neutral/backward threshold.)

The variable `speed_controller_current_state` in fact represents this state
twice: lower four bits means in which state we were and upper four bits are
telling us into which state we are actually transiting. So if you want the real
state, you should always look at logical OR of both parts.

Algorithm idea (of function `speed_controller_simulate_state`):
 1. Join both parts of `speed_controller_current_state` and store it as `state`
 2. Take the output signal, interpret it as a set of actions
 3. Apply every action on the `state`, obtain `new_state`
 4. If `new_state` and `speed_controller_current_state` are different, our action has to change as well. So we use `state` as a lower part of current state and `new_state` as upper part.
 5. If `new_state` and `speed_controller_current_state are same four times in a row, we assume that the transition was completed and use `new_state` as both parts.

(When we said four times in a row, we actually meaned
 `current_config.transition_filter` times in a row -- ammount of filtering can
 be configured as a parameter.)

We also have something like "reverse simulation" in function
`uint16_t calculate_action(uint16_t desired_speed_state);`: We calculate action
based on current adn desired state. (This state has same format as speed commands in the
serial protocol -- see [README.md](README.md) for description.)

### Main loop and state machine

Associated files:
 - `main.c`
 - `sb_states.h`

We have a few tasks, which needs to be run periodically:

 - `uart_input_tick();`
   - Process serial input, needs to be run at least each 75 microseconds (given the baudrate 115200)
 - `uart_output_tick();`
   - Prepares next byte for sending. If we want to send whole packet as a single stream without delays, it has to be run at least each 75 microseconds
 - `manage_input_capture();`
   - Reads input from last input_capture and run it again. Needs to be run at least two times in a period (each 20 miliseconds).
 - `switch_state_serial();`, `select_action();`
   - Switch to different mode based on last parsed packet, and then select correct output based on all inputs and current mode.
   - Should be run at least once between Timer1 overflow is handled and a it is triggered again. (We can assume also each 20 ms)
 - `check_timer_overflow();`
   - Run the simulation, prepare next output packet and add 1 to few age counters for each inputs. Should be run at least twice a period (each 20 ms)

So we are interleaving all "slow" tasks with `uart_*_tick()` in the fixed
schedule. It is not needed in current version, because whole cycle is finished
in 54 to 63 microseconds.

#### Modes

We are starting in `SB_BOOT` mode, wait first three seconds and switch to
`SB_DEFAULT_STATE` (which is `SB_REMOTE_ONLY` by default). All the processing
of current mode is done inside `select_action()` function. Most modes have
quite straight-forward code.

Current mode is stored inside upper four bits of `uint8_t global_state`
variable. Lower four bits are reserved for states of the main mode. It is
mainly used for `SB_PAUSE` mode. We implement the input filtering using
submodes (something like button debouncing):

 - `0x70`: Normal running; if the controller throttle is pressed, transit to `0x71`
 - `0x71`: Remote action detected; if the controller is released, get back to `0x70`; if we are there for 5 cycles, switch automatically to `0x72`
 - `0x72`: Paused, button pressed, waiting for release; if the controller is released, switch to `0x73`
 - `0x73`: Remote released; if the controller is pressed again, get back to `0x72`; if we are there for 5 cycles, switch automatically to `0x74`
 - `0x74`: Proper pause, nothing is pressed anymore;

and so on... Read the comments in the code for description of other states.
