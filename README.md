# NM1240 Motor Demo Code (Keil V6)

NM1240 motor control demo code using FOC (Field-Oriented Control) algorithm with 3 Hall sensors and 2 shunt resistors for fan motor control.

## Project Structure

```
SampleCode/
├── Hard_Fault_Sample/     # Hard fault handler sample project
│   ├── IAR/               # IAR Embedded Workbench project files
│   ├── KEIL/              # Keil MDK project files
│   └── main.c
├── Template/              # Blank template project
│   ├── IAR/               # IAR Embedded Workbench project files
│   ├── Keil/              # Keil MDK project files
│   └── main.c
└── UserProj/              # Main motor control project
    ├── program/           # Source code
    │   ├── main.c                    # Main entry point
    │   ├── motor_FOC.c / .h          # FOC algorithm
    │   ├── motor_funtions.c          # Motor control functions
    │   ├── motor_functions.h         # Motor functions header
    │   ├── motor_six_step.c / .h     # Six-step commutation
    │   ├── PI_control.c / .h         # PI controller
    │   ├── system_initialize.C / .H  # System initialization
    │   ├── system_parameter.H        # System parameters
    │   ├── variable_initialize.c     # Variable initialization
    │   ├── variable_typedefine.h     # Type definitions
    │   ├── INT_ISR.C                 # Interrupt service routines
    │   ├── protocol.c / .h           # Communication protocol
    │   ├── DataFlash.c / .h          # Data flash operations
    │   ├── svpwm.h                   # SVPWM header
    │   ├── svpwm_v02.lib             # SVPWM library
    │   └── temp.c                    # Temperature handling
    ├── RTE/               # Run-Time Environment config
    ├── NM1240_1R_2R_6stp_Hall_FOC.uvprojx  # Keil V6 project
    └── Nu_Link_Driver.ini                  # Nu-Link debugger config
```

## Development Tools

- **Keil MDK V6** (ARM Compiler 6 / ARMCLANG)
- **IAR Embedded Workbench** (for Hard_Fault_Sample and Template)
- **Nu-Link** debugger

## MCU

- **Nuvoton NM1240** series (NM1244D48)
- Internal HIRC 60 MHz clock

## Features

- FOC (Field-Oriented Control) algorithm
- Six-step commutation mode
- 3 Hall sensor inputs (ECAP)
- 2 shunt resistor current sensing (ADC)
- PI speed controller
- UART communication protocol
- Data flash parameter storage
- SVPWM (Space Vector PWM)
