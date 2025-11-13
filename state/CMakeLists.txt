
#### Build - state/ ####

list(APPEND MC_LIBS_STATE
	${CMAKE_SOURCE_DIR}/state/mcstate.cpp
)

if(MC_STATE_TYPE STREQUAL "SIM")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/sim/state_sim.cpp
	)
elseif(MC_STATE_TYPE STREQUAL "COMPLEMENTARY")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/complementary/state_complementary.cpp
	)
elseif(MC_STATE_TYPE STREQUAL "MADGWICK")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/madgwick/state_madgwick.cpp
	)
elseif(MC_STATE_TYPE STREQUAL "EKF")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/ekf/ekf.cpp
		${CMAKE_SOURCE_DIR}/state/ekf/ekf_predict.c
		${CMAKE_SOURCE_DIR}/state/ekf/ekf_correct.c
		#${CMAKE_SOURCE_DIR}/state/ekf/ekf_predict_mx.c
		#${CMAKE_SOURCE_DIR}/state/ekf/ekf_correct_mx.c
	)
elseif(MC_STATE_TYPE STREQUAL "NONE")
	list(APPEND MC_LIBS_STATE
		${CMAKE_SOURCE_DIR}/state/none/state_none.cpp
	)
endif()

add_library(mc-state STATIC ${MC_LIBS_STATE})

target_include_directories(mc-state PUBLIC
	${MC_INCLUDE_DIRS}
)

