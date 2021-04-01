
set(SOURCES dcsync_test.c)
add_executable(dcsync_test ${SOURCES})
target_link_libraries(dcsync_test soem m)
install(TARGETS dcsync_test DESTINATION bin)

set(SOURCES data_proccess.c)
add_executable(data_proccess ${SOURCES})
target_link_libraries(data_proccess soem m)
install(TARGETS data_proccess DESTINATION bin)

include_directories(./include)