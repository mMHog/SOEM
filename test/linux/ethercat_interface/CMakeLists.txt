
set(SOURCES ethercat_interface.c)
add_executable(ethercat_interface ${SOURCES})
target_link_libraries(ethercat_interface soem m)
install(TARGETS ethercat_interface DESTINATION bin)

include_directories(./include)