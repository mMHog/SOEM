
set(SOURCES pid_interface.c)
add_executable(pid_interface ${SOURCES})
target_link_libraries(pid_interface soem m)
install(TARGETS pid_interface DESTINATION bin)

set(SOURCES inter_interface.c)
add_executable(inter_interface ${SOURCES})
target_link_libraries(inter_interface soem m)
install(TARGETS inter_interface DESTINATION bin)

set(SOURCES pid_test.c)
add_executable(pid_test ${SOURCES})
target_link_libraries(pid_test soem m)
install(TARGETS pid_test DESTINATION bin)

set(SOURCES mems_server.c)
add_executable(mems_server ${SOURCES})
install(TARGETS mems_server DESTINATION bin)

set(SOURCES welder_client.c)
add_executable(welder_client ${SOURCES})
install(TARGETS welder_client DESTINATION bin)

set(SOURCES mems_client.c)
add_executable(mems_client ${SOURCES})
install(TARGETS mems_client DESTINATION bin)

set(SOURCES load_data.c)
add_executable(load_data ${SOURCES})
target_link_libraries(load_data soem m)
install(TARGETS load_data DESTINATION bin)

set(SOURCES random.c)
add_executable(random ${SOURCES})
target_link_libraries(random soem m)
install(TARGETS random DESTINATION bin)

include_directories(./include)