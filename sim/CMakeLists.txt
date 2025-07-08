
file(GLOB MC_LIBS_SIM "${CMAKE_SOURCE_DIR}/sim/*.cpp")

foreach(src ${MC_LIBS_SIM})
	message(VERBOSE "${src}")
endforeach()

add_library(libs_sim STATIC ${MC_LIBS_SIM})

target_include_directories(libs_sim PUBLIC
	${MC_INCLUDE_DIRS}
)

target_link_libraries(libs_sim PUBLIC mc-control)



