# build the Hokuyo UAM-05LP driver library
set(HOKUYO_UAM05LP_SRCS
  	samples/cpp/Connection_information.cpp
  	samples/cpp/get_distance_intensity.cpp
	src/ticks.cpp
	src/urg_connection.c
	src/urg_debug.c
	src/Urg_driver.cpp
	src/urg_ring_buffer.c
	src/urg_sensor.c
	src/urg_serial.c
	#src/urg_serial_linux.c
	src/urg_serial_utils.c
	#src/urg_serial_utils_linux.c
	src/urg_tcpclient.c
	src/urg_utils.c
)

add_library(hokuyo_uam05lp_driver ${HOKUYO_UAM05LP_SRCS})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


