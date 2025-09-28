# STM32 Reflow Oven Controller

A precision temperature controller for PCB reflow soldering using an STM32F051R4T6 microcontroller. Features integer-only PID control, zero-cross detection for AC power management, and a comprehensive reflow profile system.

## 🚀 Features

- **Precision Temperature Control**: Integer-based PID controller using Q16.16 fixed-point arithmetic
- **Zero-Cross Detection**: Phase-angle control of solid-state relays for precise power regulation
- **Advanced Reflow Profiles**: Multi-stage temperature profiles with ramp and hold phases
- **20×2 LCD Display**: Real-time temperature monitoring and reflow progress tracking
- **Flash Optimized**: Only 12.36KB flash usage (77% of 16KB available) - saved 3KB by eliminating floating-point operations
- **Professional Interface**: Startup splash screen and intuitive status display

## 🛠️ Hardware Configuration

### STM32F051R4T6 Microcontroller
- **CPU**: ARM Cortex-M0 at 48MHz (8MHz HSE × 6 PLL)
- **Package**: LQFP64
- **Flash**: 16KB (12.36KB used, 23% free)
- **RAM**: 8KB

### Key Components

#### Temperature Sensing
- **Thermocouple Interface**: MAX31855 via SPI1
  - CS1: PC5 (Chip Select)
  - SCK: PA5, MISO: PA6, MOSI: PA7
  - 100ms sampling rate (TIM14 interrupt)

#### AC Power Control
- **Zero-Cross Detection**: PA10 (External interrupt)
- **Zero-Cross Enable**: PA9 (Always enabled)
- **Solid State Relays**:
  - R1E: PC10 (Relay 1 Enable)
  - R2E: PC11 (Relay 2 Enable)
  - PWM control: 0-60 cycles per second (60Hz AC)

#### User Interface
- **20×2 Character LCD** (HD44780 compatible):
  - Data: GPIOB (PB0-PB7)
  - Control: RS (PB10), RW (PB9), E (PB8)
- **Button**: PC8 - Starts/stops reflow process
- **Rotary Encoder**: TIM3 (future expansion)

## 📊 Reflow Profile

Default profile based on ChipQuik NC191SNL50 solder paste:

| Phase | Start Temp | Target Temp | Ramp Time | Hold Time | Type |
|-------|------------|-------------|-----------|-----------|------|
| Preheat | 50°C | 150°C | 90s | 0s | RAMP |
| Soak | 150°C | 150°C | 0s | 60s | HOLD |
| Ramp to Reflow | 150°C | 217°C | 30s | 0s | RAMP |
| Reflow Peak | 217°C | 249°C | 30s | 10s | RAMP |
| Cooling | 249°C | 50°C | 0s | 0s | RAMP |

### Profile Features
- **Temperature-based Progression**: Waits for actual temperature before advancing phases
- **±4°C Tolerance**: Hold phases only progress when temperature is within tolerance
- **Intelligent State Machine**: Tracks RAMPING/HOLDING/COMPLETE states
- **Feedforward Control**: Additional power boost above 200°C for high-temperature performance

## 🔧 PID Controller

### Integer Implementation (Q16.16 Fixed-Point)
- **Kp**: 55.0 → 3,604,480 (scaled)
- **Ki**: 15.0 → 983,040 (scaled)
- **Kd**: 0.5 → 32,768 (scaled)
- **Output Range**: 0-60 (PWM cycles)
- **Anti-windup**: Dynamic integral clamping
- **Sample Time**: 1 second (60 zero-cross cycles)

### Flash Savings
By eliminating floating-point operations:
- **Before**: ~15.4KB flash usage
- **After**: 12.36KB flash usage
- **Savings**: ~3KB (18.75% reduction)

## 📱 Display Interface

### Startup Screen
```
 de Byl Reflow Oven
     Booting...
```

### Runtime Display
**Row 1**: `Reflow: OFF/ON    XXX°C`
**Row 2**:
- Idle: `IDLE`
- Active: `PHASE [x/y]      XXX°C`

Where:
- `PHASE` = WAIT/RAMP/HOLD
- `[x/y]` = Current segment / Total segments
- `XXX°C` = Current temperature or setpoint

## ⚡ Performance Optimizations

### Flash Optimization
1. **Integer-only PID**: Eliminated soft-float library (~1KB saved)
2. **Integer Linear Interpolation**: Replaced float math in temperature ramping (~500B saved)
3. **Efficient Data Structures**: Optimized reflow profile storage (~1.5KB saved)

### Real-time Performance
- **Zero-Cross Timing**: <1ms response time for AC power control
- **Temperature Sampling**: 10Hz thermocouple readings
- **PID Calculation**: 1Hz update rate for stable control
- **Display Refresh**: 5Hz for smooth user experience

## 🔨 Build System

### Prerequisites
- ARM GCC Toolchain
- STM32F0xx HAL Library
- clang-format (optional, for code formatting)

### Building
```bash
# Clean build
make clean

# Compile (parallel build)
make -j16

# Format code (optional)
find Core/ -name "*.c" -o -name "*.h" | xargs clang-format -i
```

### Flash Programming
```bash
# Generate hex file (included in build)
# Output: build/stm32-hal-reflow.hex

# Using ST-Link or compatible programmer
st-flash write build/stm32-hal-reflow.bin 0x8000000
```

## 📁 Project Structure

```
Core/
├── Src/
│   ├── main.c          # Main application and reflow logic
│   ├── pid_int.c       # Integer PID controller
│   ├── lcd.c           # LCD display driver
│   ├── menu.c          # Menu system (basic)
│   └── stm32f0xx_*.c   # HAL and system files
├── Inc/
│   ├── main.h          # Pin definitions and types
│   ├── pid_int.h       # PID controller interface
│   ├── lcd.h           # LCD driver interface
│   └── menu.h          # Menu system interface
Drivers/                # STM32 HAL libraries
Makefile               # Build configuration
*.ioc                  # STM32CubeMX configuration
```

## 🛡️ Safety Features

- **Temperature Limits**: Configurable maximum temperatures
- **Settling Delays**: 1-second startup delay prevents spurious triggering
- **Interrupt Priorities**: Critical temperature control has highest priority
- **Watchdog Protection**: System reset on fault conditions
- **Zero-Cross Synchronization**: Safe AC power switching

## 🔬 Technical Details

### Interrupt Priorities
1. **TIM14** (thermocouple): Priority 0 (highest)
2. **SPI1**: Priority 1
3. **TIM17** (reflow timer): Priority 2
4. **EXTI4_15** (button/zero-cross): Priority 3

### Timer Configuration
- **TIM14**: 100ms thermocouple sampling
- **TIM17**: 1s reflow profile updates
- **TIM3**: Rotary encoder input (future use)

### Memory Usage
- **Flash**: 12,168 bytes text + 196 bytes data = 12,364 bytes total
- **RAM**: ~2.3KB static allocation + stack
- **Available Flash**: 3,636 bytes (22.75% free for future features)

## 🚀 Future Enhancements

- [ ] **Menu Navigation**: Profile selection via rotary encoder
- [ ] **Custom Profiles**: User-programmable reflow profiles
- [ ] **Data Logging**: Temperature history storage
- [ ] **USB Interface**: PC connectivity for monitoring/control
- [ ] **Safety Interlocks**: Over-temperature protection
- [ ] **Multi-Zone Control**: Independent heating elements

## 👨‍💻 Development

### Author
**Bastian de Byl** (bastian@bdebyl.net)

### License
BSD 3-Clause (STMicroelectronics HAL components maintain their original license)

### Contributing
1. Fork the repository
2. Create a feature branch
3. Apply clang-format to Core/ files
4. Test on hardware
5. Submit pull request

## 🔗 References

- [ChipQuik NC191SNL50 Datasheet](http://www.chipquik.com/datasheets/NC191SNL50.pdf)
- [STM32F051 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00031936-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [MAX31855 Thermocouple Interface](https://datasheets.maximintegrated.com/en/ds/MAX31855.pdf)

---

**⚠️ Safety Notice**: This controller manages high-voltage AC power and high temperatures. Only qualified personnel should install and operate this system. Always follow electrical safety procedures and local codes.