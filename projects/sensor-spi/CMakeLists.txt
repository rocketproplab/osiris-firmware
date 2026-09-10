cmake_minimum_required(VERSION 3.22)

# Named individually rather than pointing at ../../components as a whole: some
# of the legacy components there register source files that no longer exist,
# and any one of them failing to configure takes the whole build down.
set(EXTRA_COMPONENT_DIRS
	"${CMAKE_CURRENT_LIST_DIR}/../../components/ai_spi"
	"${CMAKE_CURRENT_LIST_DIR}/../../components/ai_lsm6dsox"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor-spi)
