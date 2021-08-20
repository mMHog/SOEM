
set(SOURCES welder.c)
add_executable(welder ${SOURCES})
target_link_libraries(welder soem m)
install(TARGETS welder DESTINATION bin)

include_directories(./include)
