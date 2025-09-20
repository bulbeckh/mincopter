
#pragma once

#include <AP_Math.h>

/* @brief State struct containing full system state. This struct is passed to ahrs and inertial_nav */
typedef struct {
	double _position[3];
	double _velocity[3];
	// TODO We should be able to select the internal data type used by Quaternion class (i.e. float, double)
	Quaternion _attitude;

	// Angular velocity and inertial frame accelerations
	Vector3f _omega;
	Vector3f _accel;

	// Euler angle representation
	Vector3f _euler;

	// Euler rates representation
	Vector3f _euler_rates;

	// TODO Add bias states
} MCStateData;



