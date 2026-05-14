
include_directories(./include)

set(SOURCES controller.c)
add_executable(controller ${SOURCES})
target_link_libraries(controller soem m)
install(TARGETS controller DESTINATION bin)

set(SOURCES api_test.c
        include/slave_sbt908.h
)
add_executable(api_test ${SOURCES})
target_link_libraries(api_test soem m)
install(TARGETS api_test DESTINATION bin)