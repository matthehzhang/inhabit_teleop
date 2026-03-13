# Select firmware variant: "internal_adc" (default, original) or "mcp3008"
# Override at build time:  idf.py -DFIRMWARE_VARIANT=mcp3008 build
set(FIRMWARE_VARIANT "internal_adc" CACHE STRING "Firmware variant: internal_adc | mcp3008")

if(FIRMWARE_VARIANT STREQUAL "mcp3008")
    set(MAIN_SRC "main_mcp3008.c")
else()
    set(MAIN_SRC "main.c")
endif()

idf_component_register(
    SRCS ${MAIN_SRC}
    INCLUDE_DIRS "."
)
