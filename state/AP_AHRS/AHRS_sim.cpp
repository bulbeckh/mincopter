
#include "AHRS_sim.h"

#include "AP_Math.h"

#include "mcinstance.h"
extern MCInstance mincopter;

#include "gz_interface.h"
extern GZ_Interface gz_interface;

#include "simulation_logger.h"
extern SimulationLogger simlog;

#include "mcinstance.h"
extern MCInstance mincopter;

void AHRS_sim::update()
{
	// Call the ins.update method otherwise it won't measure state
	mincopter.ins.update();

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

#ifdef TARGET_ARCH_LINUX
	float error_rp = get_error_rp();
	float error_yaw = get_error_yaw();
	simlog.write_ahrs_state(roll_sensor, pitch_sensor, yaw_sensor, _accel_ef.x, _accel_ef.y, _accel_ef.z, error_rp, error_yaw);
#endif

	return;
}

void AHRS_sim::reset(bool recover_eulers)
{
	return;
}

const Vector3f AHRS_sim::get_gyro(void) const
{
	return mincopter.ins.get_gyro();
}

void AHRS_sim::reset_attitude(const float &roll, const float &pitch, const float &yaw)
{
	roll_sensor = (int32_t)(degrees(roll)*100.0f);
	pitch_sensor = (int32_t)(degrees(pitch)*100.0f);
	yaw_sensor = (int32_t)(degrees(yaw)*100.0f);

	return;
}

