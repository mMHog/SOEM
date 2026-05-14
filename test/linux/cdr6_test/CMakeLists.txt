
set(SOURCES cdr6_test.c)
add_executable(cdr6_test ${SOURCES})
target_link_libraries(cdr6_test soem)
install(TARGETS cdr6_test DESTINATION bin)
