# build the sensor library
SET(SENSOR_SRCS
	src/ScannerParam.cpp
	src/RangeScanner.cpp
	src/PfR2000LaserScanner.cpp
	src/HokuyoLaserScanner.cpp
	src/SickSafetyLaserScanner.cpp
	src/Sick581LaserScanner.cpp
	src/laser_t.c
	src/SensorFamily.cpp
)

add_library(sensor_lib ${SENSOR_SRCS})
add_dependencies(sensor_lib pf_r2000_driver hokuyo_uam05lp_driver sick_nanoscan3_driver maths_lib lcm_lib)
target_link_libraries(sensor_lib pf_r2000_driver hokuyo_uam05lp_driver sick_nanoscan3_driver maths_lib lcm_lib)

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set_target_properties(sensor_lib PROPERTIES OUTPUT_NAME "sensor")

