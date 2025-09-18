# VESC Firmware Application Configuration

This firmware has been modified to support conditional compilation of applications to reduce firmware size.

## How to Configure Applications

### Option 1: Modify applications.mk (Recommended)

Edit `applications/applications.mk` and change the application enable flags:

```makefile
# Application configuration - define which apps to compile
# Set to 1 to enable, 0 to disable. This will significantly reduce firmware size.
APP_PPM_ENABLE ?= 0          # Disable PPM
APP_ADC_ENABLE ?= 0          # Disable ADC  
APP_STEN_ENABLE ?= 0         # Disable STEN
APP_UART_ENABLE ?= 0         # Disable UART
APP_NUNCHUK_ENABLE ?= 0      # Disable Nunchuk
APP_PAS_ENABLE ?= 0          # Disable PAS
APP_CUSTOM_ENABLE ?= 1       # Keep Custom (orthopus) enabled
APP_DPV_ENABLE ?= 0          # Disable DPV
APP_SKYPUFF_ENABLE ?= 0      # Disable Skypuff
APP_CUSTOM_TEMPLATE_ENABLE ?= 0  # Disable template
```

### Option 2: Command Line Override

You can override the settings from the command line when building:

```bash
make APP_PPM_ENABLE=0 APP_ADC_ENABLE=0 APP_UART_ENABLE=0 [your_target]
```

## Current Configuration (for Orthopus Users)

The default configuration is optimized for users who only need the orthopus application:

- ✅ **APP_CUSTOM_ENABLE = 1** (orthopus applications)
- ❌ **APP_PPM_ENABLE = 0** (PPM/RC input)
- ❌ **APP_ADC_ENABLE = 0** (Analog input)
- ❌ **APP_UART_ENABLE = 0** (UART communication)
- ❌ **APP_NUNCHUK_ENABLE = 0** (Wii Nunchuk)
- ❌ **APP_PAS_ENABLE = 0** (Pedal Assist)
- ❌ **APP_STEN_ENABLE = 0** (STEN specific)
- ❌ **APP_DPV_ENABLE = 0** (DPV specific)
- ❌ **APP_SKYPUFF_ENABLE = 0** (Skypuff specific)
- ❌ **APP_CUSTOM_TEMPLATE_ENABLE = 0** (Development template)

## Firmware Size Reduction

Disabling unused applications can significantly reduce firmware size:

- Each disabled application saves typically 2-10KB of flash memory
- With all non-orthopus applications disabled, you can save 20-50KB or more
- This leaves more space for your orthopus application logic

## Compatibility with Upstream

This modification is designed to be compatible with upstream VESC firmware:

- All original functionality is preserved when applications are enabled
- No source code is removed, only conditionally compiled
- Easy to merge upstream changes
- Can be reverted by setting all APP_*_ENABLE to 1

## Application Descriptions

| Application | Description | Typical Use Case |
|-------------|-------------|------------------|
| PPM | RC/PPM input control | RC controllers, traditional ESCs |
| ADC | Analog input control | Potentiometer/throttle control |
| UART | UART communication | External control systems |
| Nunchuk | Wii Nunchuk input | Gaming controller input |
| PAS | Pedal Assist System | E-bike pedal assistance |
| Custom | Custom applications | Orthopus and other custom apps |

## Troubleshooting

If you encounter build errors:

1. **Linker errors**: Make sure `app_stubs.c` is included in the build
2. **Missing functions**: Check that the application configuration matches your needs
3. **Runtime errors**: Ensure you have at least one application enabled

## Reverting to Full Build

To restore all applications (original behavior):

```makefile
APP_PPM_ENABLE ?= 1
APP_ADC_ENABLE ?= 1
APP_STEN_ENABLE ?= 1
APP_UART_ENABLE ?= 1
APP_NUNCHUK_ENABLE ?= 1
APP_PAS_ENABLE ?= 1
APP_CUSTOM_ENABLE ?= 1
APP_DPV_ENABLE ?= 1
APP_SKYPUFF_ENABLE ?= 1
APP_CUSTOM_TEMPLATE_ENABLE ?= 1
```
