
set(SOURCES pid_interface.c)
add_executable(pid_interface ${SOURCES})
target_link_libraries(pid_interface soem m)
install(TARGETS pid_interface DESTINATION bin)

set(SOURCES pid_test.c)
add_executable(pid_test ${SOURCES})
target_link_libraries(pid_test soem m)
install(TARGETS pid_test DESTINATION bin)

set(SOURCES mems_server.c)
add_executable(mems_server ${SOURCES})
install(TARGETS mems_server DESTINATION bin)

set(SOURCES mems_client.c)
add_executable(mems_client ${SOURCES})
install(TARGETS mems_client DESTINATION bin)

include_directories(./include)