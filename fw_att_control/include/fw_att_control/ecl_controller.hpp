#pragma once

#include <stdint.h>
#include <fw_att_control/defines.hpp>
#include <mathlib/math.hpp>

namespace ariitk::fw_att_control {
struct ECL_ControlData {
    float roll;
    float pitch;
    float yaw;
    float body_x_rate;
    float body_y_rate;
    float body_z_rate;
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;
    float roll_rate_setpoint;
    float pitch_rate_setpoint;
    float yaw_rate_setpoint;
    float airspeed_min;
    float airspeed_max;
    float airspeed;
    float scaler;
    float groundspeed;
    float groundspeed_scaler;
    bool lock_integrator;
};

class ECL_Controller {

    public:
        ECL_Controller();
        virtual ~ECL_Controller() = default;

        virtual float controlAttitude(const ECL_ControlData &ctl_data ) = 0;
        virtual float controlEulerRate(const ECL_ControlData &ctl_data) = 0;
        virtual float controlBodyRate(const ECL_ControlData &ctl_data) = 0;

        // Setters 
        void setTimeConstant(float time_constant);
        void setKp(float k_p);
        void setKi(float k_i);
        void setKff(float k_ff);
        void setIntegratorMax(float max);
        void setMaxRate(float max_rate);
        void setBodyRateSetpoint( float rate);

        //Getters
        float getRateError();
        float getDesiredRate();
        float getDesiredBodyRate();
        float getIntegrator();

        void resetIntegrator();

    protected:
        uint64_t last_run_;
        float tc_;
        float k_p_;
        float k_i_;
        float k_ff_;
        float integrator_max_;
        float max_rate_;
        float last_output_;
        float integrator_;
        float rate_error_;
        float rate_setpoint_;
        float bodyrate_setpoint_;
        float constrainAirspeed(float airspeed, float minspeed, float maxspeed);
};
}