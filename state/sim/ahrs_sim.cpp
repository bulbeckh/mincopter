
#include "ahrs_sim.h"

#include "AP_Math.h"

#include "mcinstance.h"
extern MCInstance mincopter;

void AHRS_sim::_ahrs_init_internal(void)
{
	/* Don't need to do anything */
	return;
}

void AHRS_sim::ahrs_update()
{
	// TODO Needs to be updated for new ahrs interface
	// We need to retrieve the orientation and then update the attitude quaternion (_state._attitude)
	
	/*
	// Call the ins.update method otherwise it won't measure state
	mincopter.ins.update();

	// Update the <roll,pitch,yaw>_sensor variables as well as the _accel_ef variable

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

	float error_rp = get_error_rp();
	float error_yaw = get_error_yaw();

	//simlog.write_ahrs_state(roll_sensor, pitch_sensor, yaw_sensor, _accel_ef.x, _accel_ef.y, _accel_ef.z, error_rp, error_yaw);
	
	*/

	_ahrs_state->_attitude.from_euler(
			mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_x,
			mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_y,
			mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_z
			);

	_ahrs_state->_euler.x = mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_x;
	_ahrs_state->_euler.y = mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_y;
	_ahrs_state->_euler.z = mincopter.hal.sim->last_sensor_state.wldAbdyA_eul_z;

	_ahrs_state->_euler_rates.x = mincopter.hal.sim->last_sensor_state.euler_rate_x;
	_ahrs_state->_euler_rates.y = mincopter.hal.sim->last_sensor_state.euler_rate_y;
	_ahrs_state->_euler_rates.z = mincopter.hal.sim->last_sensor_state.euler_rate_z;
			
	return;
}

