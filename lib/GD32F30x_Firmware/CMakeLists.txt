set(SOURCES
    GD32F30x_standard_peripheral/Source/gd32f30x_adc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_bkp.c
    GD32F30x_standard_peripheral/Source/gd32f30x_can.c
    GD32F30x_standard_peripheral/Source/gd32f30x_crc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_ctc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_dac.c
    GD32F30x_standard_peripheral/Source/gd32f30x_dbg.c
    GD32F30x_standard_peripheral/Source/gd32f30x_dma.c
    GD32F30x_standard_peripheral/Source/gd32f30x_enet.c
    GD32F30x_standard_peripheral/Source/gd32f30x_exmc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_exti.c
    GD32F30x_standard_peripheral/Source/gd32f30x_fmc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_fwdgt.c
    GD32F30x_standard_peripheral/Source/gd32f30x_gpio.c
    GD32F30x_standard_peripheral/Source/gd32f30x_i2c.c
    GD32F30x_standard_peripheral/Source/gd32f30x_misc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_pmu.c
    GD32F30x_standard_peripheral/Source/gd32f30x_rcu.c
    GD32F30x_standard_peripheral/Source/gd32f30x_rtc.c
    GD32F30x_standard_peripheral/Source/gd32f30x_sdio.c
    GD32F30x_standard_peripheral/Source/gd32f30x_spi.c
    GD32F30x_standard_peripheral/Source/gd32f30x_timer.c
    GD32F30x_standard_peripheral/Source/gd32f30x_usart.c
    GD32F30x_standard_peripheral/Source/gd32f30x_wwdgt.c
)

set(TARGET gd32f30x)

add_library(${TARGET} OBJECT
    ${SOURCES}
)

# target_compile_definitions(${TARGET}
#     PUBLIC
# )

target_include_directories(${TARGET} 
    PUBLIC
        CMSIS/GD/GD32F30x/Include
        CMSIS/
        GD32F30x_standard_peripheral/Include
        config/
)