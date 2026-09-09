# Thin fan-out to SD and telemetry. flight_logic is deliberately NOT a
# dependency -- it consumes current sensor state via its own task, not
# queued records through this router. See data_router.h.
idf_component_register(
    SRCS "data_router.c"
    INCLUDE_DIRS "."
    REQUIRES sensor_records sd_logger telemetry
)
