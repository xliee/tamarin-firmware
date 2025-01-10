pico_generate_pio_header(tamarin_firmware ${CMAKE_CURRENT_LIST_DIR}/lightning_tx.pio)
pico_generate_pio_header(tamarin_firmware ${CMAKE_CURRENT_LIST_DIR}/lightning_rx.pio)
pico_generate_pio_header(tamarin_firmware ${CMAKE_CURRENT_LIST_DIR}/probe.pio)


set(SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/tamarin_probe.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/usb_descriptors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/util.c
    ${CMAKE_CURRENT_SOURCE_DIR}/crc.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SOURCE_FILES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})