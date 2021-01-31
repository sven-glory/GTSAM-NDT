# build the sick nanoscan3 driver library
aux_source_directory(src/cola2 COLA2_SRCS)
aux_source_directory(src/communication COMMUNICATION_SRCS)
aux_source_directory(src/data_processing DATA_PROCESS_SRCS)
aux_source_directory(src/datastructure DATASTRUCT_SRCS)

set(SICK_NANOSCAN3_SRCS
  	${COLA2_SRCS}
  	${COMMUNICATION_SRCS}
  	${DATA_PROCESS_SRCS}
	${DATASTRUCT_SRCS}
	src/SickSafetyscannersBase.cpp
)

add_library(sick_nanoscan3_driver ${SICK_NANOSCAN3_SRCS})
target_link_libraries(sick_nanoscan3_driver ${Boost_LIBRARIES})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


