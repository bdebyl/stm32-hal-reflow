# STM32 Reflow Oven Controller

## Project Overview

This is an STM32F051R4T6-based toaster oven controller for PCB reflow soldering. The system implements a custom PID controller with zero-cross detection for precise temperature control using solid-state relays (SSRs) to control heating elements.

## Hardware Configuration

### STM32F051R4T6 Microcontroller
- **CPU**: ARM Cortex-M0 at 48MHz (8MHz HSE × 6 PLL)
- **Package**: LQFP64
- **Flash**: 16KB
- **RAM**: 8KB

### Key Hardware Components

#### Temperature Sensing
- **Thermocouple Interface**: SPI1 (MAX31855 or similar)
  - CS1: PC5 (Chip Select)
  - SCK: PA5, MISO: PA6, MOSI: PA7
  - Periodic reading via TIM14 interrupt (100ms intervals)

#### AC Power Control
- **Zero-Cross Detection**: PA10 (GPIO_ZX_DET) - External interrupt
- **Zero-Cross Enable**: PA9 (GPIO_ZX_EN) - Always enabled
- **Solid State Relays**:
  - R1E: PC10 (Relay 1 Enable)
  - R2E: PC11 (Relay 2 Enable)
  - PWM control based on zero-cross timing (60 cycles max)

#### User Interface
- **20×2 Character LCD**:
  - Data Port: GPIOB (PB0-PB7)
  - Control Pins: RS (PB10), RW (PB9), E (PB8)
  - Inverted GPIO logic
- **Button**: PC8 (GPIO_BTN) - Starts reflow process
- **Rotary Encoder**: TIM3 encoder mode (currently unused in code)

## Software Architecture

### Core Modules

#### PID Controller (`pid.c/h`)
- **Implementation**: Based on Phil's Lab YouTube tutorial
- **Parameters**:
  - Kp = 40.0 (Proportional gain)
  - Ki = 1.6 (Integral gain)  
  - Kd = 0.2 (Derivative gain)
  - Sample time = 1 second
  - Output limits: 0-60 (matching zero-cross count)
- **Features**:
  - Anti-windup via dynamic integrator clamping
  - Derivative filtering with tau = 0.2s
  - 16-bit integer output for direct PWM control

#### LCD Driver (`lcd.c/h`)
- **Type**: HD44780-compatible parallel interface
- **Features**:
  - 20×2 character display support
  - Inverted GPIO logic handling
  - Cursor positioning and string writing
  - Character and instruction mode switching

#### Menu System (`menu.c/h`)
- **Status**: Partially implemented
- **Current**: Only reset/copyright display
- **Future**: Navigation for profile selection and manual temperature control

### Temperature Control System

#### Zero-Cross Detection & SSR Control
Located in `HAL_GPIO_EXTI_Callback()` at main.c:124:

1. **Zero-Cross Interrupt (PA10)**:
   - Updates thermocouple temperature from SPI buffer
   - Increments zero-cross counter (0-59, representing 60Hz AC cycles)
   - Runs PID calculation every 60 zero-crossings (1 second intervals)
   - Controls SSR timing based on PID output

2. **PWM Strategy**:
   - PID output (0-60) determines how many zero-cross cycles SSRs are ON
   - Both R1E and R2E controlled simultaneously
   - Provides phase-angle control for precise power regulation

#### Enhanced Reflow Profile System
Located in main.c:79-90 with improved ramp and hold capabilities:

```c
static ReflowProfile_TypeDef ReflowProfile[] = {
    // Preheat ramp: 25°C to 150°C over 90 seconds
    {.StartTemperature = 25, .TargetTemperature = 150, .RampTimeSeconds = 90, .HoldTimeSeconds = 0, .Type = REFLOW_PHASE_RAMP},
    // Preheat soak: Hold at 150°C for 60 seconds
    {.StartTemperature = 150, .TargetTemperature = 150, .RampTimeSeconds = 0, .HoldTimeSeconds = 60, .Type = REFLOW_PHASE_HOLD},
    // Ramp to reflow: 150°C to 217°C over 30 seconds
    {.StartTemperature = 150, .TargetTemperature = 217, .RampTimeSeconds = 30, .HoldTimeSeconds = 0, .Type = REFLOW_PHASE_RAMP},
    // Reflow peak: 217°C to 249°C over 30 seconds, hold for 10 seconds
    {.StartTemperature = 217, .TargetTemperature = 249, .RampTimeSeconds = 30, .HoldTimeSeconds = 10, .Type = REFLOW_PHASE_RAMP},
    // Cooling: Let natural cooling take over
    {.StartTemperature = 249, .TargetTemperature = 25, .RampTimeSeconds = 0, .HoldTimeSeconds = 0, .Type = REFLOW_PHASE_RAMP}
};
```

**Enhanced Profile Features**:
- **RAMP Phases**: Linear temperature transitions over specified time periods
- **HOLD Phases**: Maintain target temperature for specified duration (±3°C tolerance)
- **Temperature-based Progression**: Hold phases only advance when actual temperature reaches target
- **State Machine**: Tracks RAMPING/HOLDING/COMPLETE states for each phase
- **Intelligent Timer**: Hold timer only increments when temperature is within tolerance
- **Combined Phases**: Ramp phases can include hold time after reaching target

### Timer Configuration

- **TIM14**: 100ms thermocouple reading (10Hz)
- **TIM17**: 1-second reflow profile updates
- **TIM3**: Rotary encoder input (300 steps, currently unused)

### Interrupt Priorities
1. TIM14 (thermocouple): Priority 0 (highest)
2. SPI1: Priority 1
3. TIM17 (reflow timer): Priority 2
4. EXTI4_15 (button/zero-cross): Priority 3

**Note**: EXTI interrupts are enabled with a 500ms settling delay after initialization to prevent spurious zero-cross detection during startup that could erroneously trigger reflow processes.

## Current Display Output

The LCD shows:
- **Line 1**: "Temperature: XXX°C"
- **Line 2**: "Set: XXX SY" where:
  - XXX = Current setpoint temperature
  - S = State indicator: 'I' (Idle), 'R' (Ramping), 'H' (Holding)
  - Y = Current profile phase index

## Build System

- **Makefile-based** build system
- **Target**: `stm32-hal-reflow`
- **Toolchain**: ARM GCC
- **HAL Library**: STM32F0xx HAL Driver

## Future Enhancements

Based on the code structure, planned improvements include:

1. **Menu Navigation System**:
   - Profile selection via rotary encoder
   - Manual temperature control mode
   - Parameter tuning interface

2. **PID Tuning**:
   - Current parameters may need optimization
   - Adaptive tuning based on oven characteristics

3. **Enhanced Display**:
   - Real-time temperature graphing
   - Profile progress indication
   - Error/status messages

4. **Safety Features**:
   - Temperature limits and monitoring
   - Fault detection and recovery
   - Emergency shutdown capabilities

## File Structure

```
Core/
├── Src/
│   ├── main.c          # Main application and control logic
│   ├── pid.c           # PID controller implementation
│   ├── lcd.c           # LCD display driver
│   ├── menu.c          # Menu system (basic)
│   └── stm32f0xx_*.c   # HAL and system files
├── Inc/
│   ├── main.h          # Pin definitions and types
│   ├── pid.h           # PID controller interface
│   ├── lcd.h           # LCD driver interface
│   └── menu.h          # Menu system interface
Drivers/                # STM32 HAL libraries
Makefile               # Build configuration
*.ioc                  # STM32CubeMX configuration
```

## Development Notes

- **Author**: Bastian de Byl (bastian@bdebyl.net)
- **Date**: 2022
- **License**: BSD 3-Clause (STMicroelectronics components)
- **Hardware**: Custom PCB design for toaster oven integration
- **Safety**: Designed for controlled reflow soldering applications