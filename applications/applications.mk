# Application configuration - define which apps to compile
# Set to 1 to enable, 0 to disable. This will significantly reduce firmware size.
APP_PPM_ENABLE ?= 0
APP_ADC_ENABLE ?= 0
APP_STEN_ENABLE ?= 0
APP_UART_ENABLE ?= 0
APP_NUNCHUK_ENABLE ?= 0
APP_PAS_ENABLE ?= 0
APP_CUSTOM_ENABLE ?= 1
APP_DPV_ENABLE ?= 0
APP_SKYPUFF_ENABLE ?= 0
APP_CUSTOM_TEMPLATE_ENABLE ?= 0

# Pass defines to compiler
APPFLAGS = -DAPP_PPM_ENABLE=$(APP_PPM_ENABLE) \
           -DAPP_ADC_ENABLE=$(APP_ADC_ENABLE) \
           -DAPP_STEN_ENABLE=$(APP_STEN_ENABLE) \
           -DAPP_UART_ENABLE=$(APP_UART_ENABLE) \
           -DAPP_NUNCHUK_ENABLE=$(APP_NUNCHUK_ENABLE) \
           -DAPP_PAS_ENABLE=$(APP_PAS_ENABLE) \
           -DAPP_CUSTOM_ENABLE=$(APP_CUSTOM_ENABLE) \
           -DAPP_DPV_ENABLE=$(APP_DPV_ENABLE) \
           -DAPP_SKYPUFF_ENABLE=$(APP_SKYPUFF_ENABLE) \
           -DAPP_CUSTOM_TEMPLATE_ENABLE=$(APP_CUSTOM_TEMPLATE_ENABLE)

# Start with the main app.c file (always needed)
APPSRC = applications/app.c \
         applications/app_stubs.c

# Conditionally add application source files based on configuration
ifeq ($(APP_PPM_ENABLE), 1)
APPSRC += applications/app_ppm.c
endif

ifeq ($(APP_ADC_ENABLE), 1)
APPSRC += applications/app_adc.c
endif

ifeq ($(APP_STEN_ENABLE), 1)
APPSRC += applications/app_sten.c
endif

ifeq ($(APP_UART_ENABLE), 1)
APPSRC += applications/app_uartcomm.c
endif

ifeq ($(APP_NUNCHUK_ENABLE), 1)
APPSRC += applications/app_nunchuk.c
endif

ifeq ($(APP_PAS_ENABLE), 1)
APPSRC += applications/app_pas.c
endif

ifeq ($(APP_CUSTOM_ENABLE), 1)
APPSRC += applications/app_custom.c
endif

# Add specialized applications if enabled
ifeq ($(APP_DPV_ENABLE), 1)
APPSRC += applications/app_dpv.c
endif

ifeq ($(APP_SKYPUFF_ENABLE), 1)
APPSRC += applications/app_skypuff.c
endif

ifeq ($(APP_CUSTOM_TEMPLATE_ENABLE), 1)
APPSRC += applications/app_custom_template.c
endif

APPINC = applications
