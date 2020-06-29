#include <fw_att_control/ecl_controller.hpp>

namespace ariitk::fw_att_control {

ECL_Controller::ECL_Controller() : 
    last_run_(0),
    tc_(0.1f),
    k_p_(0.0f),
    k_i_(0.0f),
    k_ff_(0.0f),
    integrator_max_(0.0f),
    max_rate_(0.0f),
    last_output_(0.0f),
    integrator_(0.0f),
    rate_error_(0.0f),
    rate_setpoint_(0.0f),
    body_rate_setpoint(0.0f) {

}

void ECL_Controller::resetIntegrator() {
    integrator_ = 0.0f;
}

void ECL_Controller::setTimeConstant(float time_constant) {
    if(time_constant>0.1f && time_constant<3.0f) tc_ =  time_constant;
}

void ECL_Controller::setKp(float k_p) {
    k_p_ = k_p;
}

void ECL_Controller::setKi(float k_i) {
    k_i_ = k_i;
}

void ECL_Controller::setKff(float k_ff) {
    k_ff_ = k_ff;
}

void ECL_Controller::setIntegratorMax(float max) {
    integrator_max_ = max;
}

void ECL_Controller::setMaxRate(float max_rate) {
    max_rate = max_rate;
}

void ECL_Controller::setBodyRateSetpoint(float rate) {
    bodyrate_setpoint_ = math::constrain(rate, -max_rate_, max_rate_);
}

float ECL_Controller::getRateError() {
    return rate_error_;
}

float ECL_CONTROLLER::getDesiredRate() {
    return rate_setpoint_;
}

float ECL_Controller::getDesiredBodyRate() {
    return bodyrate_setpoint_;
}

float ECL_Controller::getIntegrator() {
    return integrator_;
}

float ECL_Controller::constrainAirspeed(float airspeed, float minspeed, float maxspeed) {
    float airspeed_result = airspeed;

    if(!ISFINITE(airspeed)) {
        airspeed_result = 0.5f*(minspeed + maxspeed);
    }
    else if(airspeed < minspeed) {
        airspeed_result = minspeed;
    }

    return airspeed_result;
}

}