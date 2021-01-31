# build the Pepperl+Fuchs R2000 driver library
#set(CMAKE_CXX_FLAGS "-std=c++11")
#set(PF_R2000_INCLUDE_DIRS 
#	include
#)

#include_directories(
#	${PF_R2000_INCLUDE_DIRS}
#)

SET(PF_R2000_SRCS
  	src/r2000_driver.cpp
  	src/http_command_interface.cpp
  	src/scan_data_receiver.cpp
	ndt_com.cpp
)

add_library(pf_r2000_driver ${PF_R2000_SRCS})
target_link_libraries(pf_r2000_driver ${Boost_LIBRARIES})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


