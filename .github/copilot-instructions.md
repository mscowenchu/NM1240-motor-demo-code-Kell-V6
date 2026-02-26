# Copilot Instructions for NM1240-motor-demo-code-Kell-V6

## Repository Overview

This repository contains embedded C firmware for the **Nuvoton NM1240** microcontroller (ARM Cortex-M0), demonstrating fan motor control using:
- **FOC (Field-Oriented Control)** with 3-phase Hall sensors and 2-shunt (2R) or 1-shunt (1R) current sensing
- **Six-Step commutation** (open-loop and closed-loop modes)

All source code is stored in the archive `SampleCode.7z`. Extract it before reading or modifying source files:
```
7z x SampleCode.7z
```

## Repository Structure (inside `SampleCode.7z`)

```
SampleCode/
├── UserProj/
│   ├── program/          # All C source and header files
│   │   ├── main.c                  # Entry point: system init, main loop
│   │   ├── system_parameter.H      # All compile-time configuration macros
│   │   ├── variable_typedefine.h   # Struct definitions (AMotor, AMotorSpec, etc.)
│   │   ├── motor_funtions.c/.h     # Core motor state machine and control loop (filename typo is in the original source)
│   │   ├── motor_FOC.c/.h          # FOC algorithm (Clark, Park, SVPWM)
│   │   ├── motor_six_step.c/.h     # Six-step commutation
│   │   ├── INT_ISR.C               # Interrupt service routines (Timer0, ECAP, ADC)
│   │   ├── system_initialize.C/.H  # MCU peripheral initialization
│   │   ├── PI_control.c/.h         # PI controller implementation
│   │   ├── protocol.c/.h           # UART communication protocol
│   │   ├── variable_initialize.c   # Motor parameter initialization
│   │   ├── DataFlash.c/.h          # Flash memory read/write
│   │   └── svpwm.h / svpwm_v02.lib # SVPWM lookup table library
│   ├── NM1240_1R_2R_6stp_Hall_FOC.uvprojx  # Keil MDK project file
│   ├── NM1240_1R_2R_6stp_Hall_FOC.uvoptx   # Keil MDK options file
│   └── obj/              # Build output (binary, map, object files)
├── Template/             # Bare project template (Keil + IAR)
└── Hard_Fault_Sample/    # Hard fault handler example (Keil + IAR)
```

## Build System

- **Primary IDE**: Keil MDK (µVision 5) — project file: `SampleCode/UserProj/NM1240_1R_2R_6stp_Hall_FOC.uvprojx`
- **Secondary IDE**: IAR Embedded Workbench — project files in `SampleCode/Template/IAR/` and `SampleCode/Hard_Fault_Sample/IAR/`
- **Compiler**: ARM Compiler 5 (armcc) targeting ARM Cortex-M0
- **Batch build**: `SampleCode/UserProj/NM1230_1R_2R_6stp_Hall_FOC.BAT` (note: file is named NM1230 but targets NM1240 hardware)
- There is **no automated CI pipeline**; builds require Keil MDK or IAR installed on Windows.

## Key Configuration (in `system_parameter.H`)

Select the control mode and hardware variant by modifying macros:

| Macro | Values | Description |
|---|---|---|
| `P_FOC_CONTROL` | 0/1 | Enable FOC mode |
| `P_SIX_STEP_CONTROL` | 0/1 | Enable six-step mode (mutually exclusive with FOC) |
| `P_2R_FOC` / `P_1R_FOC` | define/undef | Select 2-shunt or 1-shunt FOC variant |
| `SYSTEM_CLOCK` | 48000000/60000000 | MCU clock frequency in Hz |
| `C_MOTOR_PWM_FREQ` | e.g. 15000 | PWM frequency in Hz |
| `C_MOTOR_POLE` | e.g. 4 | Motor pole count |

## Coding Conventions

- **Fixed-point arithmetic**: Variables use Q15 (signed 16-bit, 1.15 format) and Q26 (1.26 format). Suffix `_Q15` or `_Q26` in names indicates the format.
- **Naming**: Variables follow the pattern `<type_prefix>_<scope>_<name>`, e.g. `i16_hall_angle` (int16), `u8_flag_error_record` (uint8).
- **Global motor state**: All motor state is held in the global `AMotor MotorA` struct (defined in `variable_typedefine.h`). Access sub-structs via `MotorA.spec`, `MotorA.info`, `MotorA.cmd`, `MotorA.pid`, `MotorA.other`.
- **Header extensions**: Some headers use `.H` and `.C` (uppercase) instead of `.h` and `.c`.
- **Inline functions**: Performance-critical helpers (e.g. `SIN()`, `COS()`, `Update_Hall_Angle()`) are declared `static __INLINE` in header files.
- **Comments**: Code comments are in English; parameter units are documented inline (e.g. `//Unit: rpm`).

## Important Notes

- This is **embedded firmware** with no host-runnable tests. Functional validation requires flashing to NM1240 hardware and measuring motor behavior.
- Modifying `system_parameter.H` macros changes hardware assumptions — ensure consistency between `P_FOC_CONTROL`/`P_SIX_STEP_CONTROL` and `P_1R_FOC`/`P_2R_FOC`.
- The SVPWM implementation is provided as a pre-compiled library (`svpwm_v02.lib`) — do not attempt to rebuild it.
- Interrupt handlers in `INT_ISR.C` are timing-critical; changes there may affect motor stability.
