set(SOURCES
    startup/tx_initialize_low_level.S
    src/battery.c
    src/digitalIO.c
    src/display.c
    src/eeprom.c
    src/gd32f30x_it.c
    src/imu.c
    src/main.c
    src/motors.c
    src/systick.c
    src/system_gd32f30x.c
    src/newlib_nano.c
)

add_executable(gd32_openfirmwaremower
    startup/startup_gd32f30x_hd.S
    ${SOURCES}
)

target_link_libraries(gd32_openfirmwaremower
    azrtos::threadx
    gd32f30x
)

target_include_directories(gd32_openfirmwaremower
    PUBLIC
        include/
)

set_target_linker(gd32_openfirmwaremower  ${CMAKE_CURRENT_SOURCE_DIR}/startup/GD32F303ZE.ld)

post_build(gd32_openfirmwaremower)