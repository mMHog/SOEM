
set(SOURCES pid_interface.c)
add_executable(pid_interface ${SOURCES})
target_link_libraries(pid_interface soem m)
install(TARGETS pid_interface DESTINATION bin)

include_directories(./include)