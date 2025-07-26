
file(GLOB MC_LIBS_SIM "${CMAKE_SOURCE_DIR}/sim/*.cpp")

foreach(src ${MC_LIBS_SIM})
	message(VERBOSE "${src}")
endforeach()

add_library(mc-sim STATIC ${MC_LIBS_SIM})

target_include_directories(mc-sim PUBLIC
	${MC_INCLUDE_DIRS}
)

target_link_libraries(mc-sim PUBLIC mc-control)



