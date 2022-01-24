set(SOURCES elmo.c)
add_executable(elmo ${SOURCES})
target_link_libraries(elmo soem m)
install(TARGETS elmo DESTINATION bin)

set(SOURCES elmo_client.c)
add_executable(elmo_client ${SOURCES})
install(TARGETS elmo_client DESTINATION bin)

set(SOURCES api.c)
add_executable(api ${SOURCES})
install(TARGETS api DESTINATION bin)

include_directories(./include)
