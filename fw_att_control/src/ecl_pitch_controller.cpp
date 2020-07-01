#include <fw_att_control/ecl_pitch_controller.hpp>

namespace ariitk::fw_att_control {

float ECL_PitchController::controlAttitude(const ECL_ControlData &ctl_data) {
    /* Do not calculate control signal with bad inputs */
    if (!(ISFINITE(ctl_data.pitch_setpoint) &&
	      ISFINITE(ctl_data.roll) &&
	      ISFINITE(ctl_data.pitch) &&
	      ISFINITE(ctl_data.airspeed))) {

		std::cout<<("not controlling pitch");
		return rate_setpoint_;
	}

    /*Apply P controller : rate setpoint from current error and time constant*/
    float pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;
    rate_setpoint_ = pitch_error/tc_;

    return rate_setpoint_;
}

float ECL_PitchController::controlBodyRate(const ECL_ControlData &ctl_data) {

    /* Do not calculate control signal with bad inputs */
	if (!(ISFINITE(ctl_data.roll) &&
	      ISFINITE(ctl_data.pitch) &&
	      ISFINITE(ctl_data.body_y_rate) &&
	      ISFINITE(ctl_data.body_z_rate) &&
	      ISFINITE(ctl_data.yaw_rate_setpoint) &&
	      ISFINITE(ctl_data.airspeed_min) &&
	      ISFINITE(ctl_data.airspeed_max) &&
	      ISFINITE(ctl_data.scaler))) {

		return math::constrain(last_output_, -1.0f, 1.0f);
	}

    uint64_t dt_micros = hrt_elapsed_time(&last_run_);
    last_run_ = hrt_absolute_time();
    float dt = (float)dt_micros*1e-6f;

    /* lock integral for long intervals */
    bool lock_integrator = ctl_data.lock_integrator;

    if (dt_micros > 500000) {
		lock_integrator = true;
	}

    rate_error_ = bodyrate_setpoint_ - ctl_data.body_y_rate;

    if(!lock_integrator && k_i_>0.0f) {
        float id = rate_error_*dt*ctl_data.scaler;

		// anti-windup: do not allow integrator to increase if actuator is at limit
        if(last_output_ <-1.0f) {
            //only allow motion to center: increase value
            id = math::max(id, 0.0f);
        }
        else if(last_output_>1.0f) {
            //only allow motion to center: decrease value
            id = math::min(id, 0.0f);
        }

        integrator_ = math::constrain(integrator_ + id*k_i_, -integrator_max_, integrator_max_);
    }

    /* Apply PI rate controller and store non-limited output */
    last_output_ = bodyrate_setpoint_*k_ff_*ctl_data.scaler + 
                   rate_error_*k_p_*ctl_data.scaler*ctl_data.scaler + 
                   integrator_; //scaler is proportional to 1/airspeed

    return math::constrain(last_output_, -1.0f, 1.0f);
}

float ECL_PitchController::controlEulerRate(const ECL_ControlData &ctl_data) {
    /* Transform setpoint to body angular rates (jacobian) */
    bodyrate_setpoint_ = cosf(ctl_data.roll)*rate_setpoint_ + 
                         cosf(ctl_data.pitch)*sinf(ctl_data.roll)*ctl_data.yaw_rate_setpoint;
    setBodyRateSetpoint(bodyrate_setpoint_); 
    return controlBodyRate(ctl_data);
}

}