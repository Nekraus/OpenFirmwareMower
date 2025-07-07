# Define ThreadX user configuration
set(TX_USER_FILE "${CMAKE_CURRENT_LIST_DIR}/threadx/tx_user.h" CACHE STRING "Enable TX user configuration")

add_subdirectory(threadx_core)
add_subdirectory(GD32F30x_Firmware)