
#include "AHRS_sim.h"
#include "AP_Math.h"

#include "gz_interface.cpp"
extern GZ_Interface gz_interface;

void AHRS_sim::update()
{
	/* Update the <roll,pitch,yaw>_sensor variables as well as the _accel_ef variable */

	// Update angle variables in deg*100
	roll_sensor  = (int32_t)(degrees(gz_interface.last_sensor_state.wldAbdyA_eul_x)*100.0f);
	pitch_sensor = (int32_t)(degrees(gz_interface.last_sensor_state.wldAbdyA_eul_y)*100.0f);
	yaw_sensor   = (int32_t)(degrees(gz_interface.last_sensor_state.wldAbdyA_eul_z)*100.0f);

	_body_dcm_matrix.from_euler(gz_interface.last_sensor_state.wldAbdyA_eul_x,
			gz_interface.last_sensor_state.wldAbdyA_eul_y,
			gz_interface.last_sensor_state.wldAbdyA_eul_z);

	// Update the accel_ef vector
	
	// In body frame here
	Vector3f accel_temp = mincopter.ins.get_accel();

	_body_dcm_matrix.rotate(accel_temp);

	// After rotation, we assign
	_accel_ef = accel_temp;

	return;
}

void AHRS_sim::reset(bool recover_eulers)
{
	return;
}
