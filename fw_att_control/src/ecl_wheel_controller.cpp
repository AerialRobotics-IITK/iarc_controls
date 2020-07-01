#include <fw_att_control/ecl_wheel_controller.hpp>

namespace ariitk::fw_att_control {

float ECL_WheelController::controlBodyRate(const ECL_ControlData &ctl_data) {
    // Do not calculate control signal with bad inputs
    if (!(ISFINITE(ctl_data.body_z_rate) &&
	      ISFINITE(ctl_data.groundspeed) &&
	      ISFINITE(ctl_data.groundspeed_scaler))) {
		return math::constrain(last_output_, -1.0f, 1.0f);
	}

    uint64_t dt_micros = hrt_elapsed_time(&last_run_);
    last_run_ = hrt_absolute_time();
    float dt = (float)dt_micros * 1e-6f;

    bool lock_integrator = ctl_data.lock_integrator;
    if(dt_micros > 500000) {
        lock_integrator = true;
    }

    // Input conditioning
    float min_speed = 1.0f;

    // body angular rate error
    rate_error_ = rate_setpoint_ - ctl_data.body_z_rate; 

    if(!lock_integrator && k_i_ > 0.0f && ctl_data.groundspeed > min_speed) {
        float id = rate_error_ * dt * ctl_data.groundspeed_scaler;

        if(last_output_ < -1.0f) {
            id = math::max(id, 0.0f);
        }
        else if(last_output_ > 1.0f) {
            id = math::min(id, 0.0f);
        }

        integrator_ = math::constrain(integrator_ + id * k_i_, -integrator_max_, integrator_max_);
    }

    last_output_ = rate_setpoint_ * k_ff_ * ctl_data.groundspeed_scaler + 
                  ctl_data.groundspeed_scaler * ctl_data.groundspeed_scaler * (rate_error_ * k_p_ + integrator_);
    
    return math::constrain(last_output_, -1.0f, 1.0f);
}

float ECL_WheelController::controlAttitude(const ECL_ControlData &ctl_data) {

    /* Do not calculate control signal with bad inputs */
	if (!(ISFINITE(ctl_data.yaw_setpoint) &&
	      ISFINITE(ctl_data.yaw))) {
		return rate_setpoint_;
	}

    float yaw_error = math::wrapPi(ctl_data.yaw_setpoint - ctl_data.yaw);

    rate_setpoint_ = yaw_error/tc_;

    if(max_rate_ > 0.01f) {
        if(rate_setpoint_ > 0.0f) {
            rate_setpoint_ = (rate_setpoint_ > max_rate_) ? max_rate_ : rate_setpoint_;
        }
        else {
            rate_setpoint_ = (rate_setpoint_ < -max_rate_) ? -max_rate_ : rate_setpoint_;
        }
    }

    return rate_setpoint_;

}

float controlEulerRate(const ECL_ControlData &ctl_data) {
    (void) ctl_data;
    return 0;
}

}


