# No hardware dependencies -- only FreeRTOS, for portMUX_TYPE and the
# critical-section macros. Deliberately kept driver-free so flight logic,
# telemetry, and SD logging can all share it without dragging in
# peripheral drivers they don't use.
idf_component_register(
    SRCS "ring_buffer.c"
    INCLUDE_DIRS "."
    REQUIRES freertos
)
