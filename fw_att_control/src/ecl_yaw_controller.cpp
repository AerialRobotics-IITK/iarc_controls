#include <fw_att_control/ecl_yaw_controller.hpp>

namespace ariitk::fw_att_control {

float ECL_YawController::controlAttitude(const ECL_ControlData &ctl_data) {
    switch(coordinated_method_) {
        case COORD_METHOD_OPEN :
            return controlAttitudeIMPLOpenLoop(ctl_data);
        case COORD_METHOD_CLOSEACC:
            return controlAttitudeIMPLAccClosedLoop(ctl_data);
        
        default:
            static hrt_abstime last_print = 0;

            if(rt_elapsed_time(&last_print) > 5e6) {
                std::cout<<"invalid param setting ";
                last_print = hrt_absolute_time();
            }
    }
}

float ECL_YawController::controlAttitudeIMPLOpenLoop(const ECL_ControlData &ctl_data) {
    /* Do not calculate control signal with bad inputs */
	if (!(ISFINITE(ctl_data.roll) &&
	      ISFINITE(ctl_data.pitch) &&
	      ISFINITE(ctl_data.roll_rate_setpoint) &&
	      ISFINITE(ctl_data.pitch_rate_setpoint))) {

		return rate_setpoint_;
	}

    float constrained_roll;
    bool inverted = false;

    /* roll is used as feedforward term and inverted flight needs to be considered */
    if(fabsf(ctl_data.roll) < math::radians(90.0f)) {
        constrained_roll = math::constrain(ctl_data.roll, math::radians(-80.0f), math::radians(80.0f));
    }
    else {
        inverted = true;

        // inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
		//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
        if(ctl_data.roll > 0.0f) {
            //Right hemisphere
            constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));
        }
        else {
            //Left Hemisphere
            constrained_roll = math::constrain(ctl_data.roll, math::radians(-180.0f), math::radians(-100.0f));
        }
    }

    constrained_roll = math::constrain(constrained_roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));

    if(!inverted) {
        // Calculate desired yaw rate from coordinated turn constraint / (no side forces)
        rate_setpoint_ = tanf(constrained_roll) * cosf(ctl_data.pitch) * CONSTANTS_ONE_G;
        rate_setpoint /= (ctl_data.airspeed < ctl_data.airspeed_min ? ctl_data.airspeed_min : ctl_data.airspeed);
    }

    if(!ISFINITE(rate_setpoint_)) {
        std::cout<<"Yaw rate setpoint not finite";
        rate_setpoint_ = 0.0f;
    }
    return rate_setpoint_;
}

float ECL_YawController::controlBodyRate(const ECL_ControlData &ctl_data) {
    /* Do not calculate control signal with bad inputs */
	if (!(ISFINITE(ctl_data.roll) && ISFINITE(ctl_data.pitch) && ISFINITE(ctl_data.body_y_rate) &&
	      ISFINITE(ctl_data.body_z_rate) && ISFINITE(ctl_data.pitch_rate_setpoint) &&
	      ISFINITE(ctl_data.airspeed_min) && ISFINITE(ctl_data.airspeed_max) &&
	      ISFINITE(ctl_data.scaler))) {
		return math::constrain(last_output_, -1.0f, 1.0f);
	}

    uint64_t = dt_micros = hrt_elapsed_time(&last_run_);
    last_run_ = hrt_absolute_time();
    float dt = (float)dt_micros * 1e-6f;

    bool lock_integrator = ctl_data.lock_integrator;

    if(dt_micros > 500000) {
        lock_integrator = true;
    }

    //input conditioning
    float airspeed = ctl_data.airspeed;

    if(!ISFINITE(airspeed)) {
        airspeed = 0.5f * (ctl_data.airspeed_min + ctl_data.airspeed_max);
    }
    else if(airspeed < ctl_data.airspeed_min) {
        airspeed = ctl_data.airspeed_min;
    }

    //Close the acceleration loop if _coordinated_method wants this: change body_rate setpoint
    if(coordinated_method_ == COORD_METHOD_CLOSEACC) {
        // lateral acceleration needs to go into integrator with a gain
    }

    rate_error_ = bodyrate_setpoint_ - ctl_data.body_z_rate;

    if(!lock_integrator && k_i_ > 0.0f && airspeed > 0.5f * ctl_data.airspeed_min) {
        float id = rate_error_ * dt;

        if(last_output_ < -1.0f) {
            id = math::max(id, 0.0f);
        }
        else if(last_output_ > 1.0f) {
            id = math::min(id, 0.0f);
        }

        last_output_ = (bodyrate_setpoint_ * k_ff_ + rate_error_ * k_p_ + integrator) * ctl_data.scaler * ctl_data.scaler;
        return math::constrain(last_output_, -1.0f, 1.0f);
    }
}

float ECL_YawController::controlAttitudeIMPLAccClosedLoop(const ECL_ControlData &ctl_data) {
    (void)ctl_data; //unused
    return 0.0f;
}

float ECL_YawController::controlEulerRate(const ECL_ControlData &ctl_data) {
    bodyrate_setpoint_ = -sinf(ctl_data.roll) * ctl_data.pitch_rate_setpoint + 
                          cosf(ctl_data.roll) * cosf(ctl_data.pitch) * rate_setpoint_;
    
    setBodyRateSetpoint(bodyrate_setpoint_);

    return controlBodyRate(ctl_data);
}
}