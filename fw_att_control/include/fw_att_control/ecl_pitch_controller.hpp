#pragma once

#include <mathlib/math.hpp>
#include "ecl_controller.hpp"

namespace ariitk::fw_att_control {

class ECL_PitchController : public ECL_Controller {
    public:
        ECL_PitchController() = default;
        ~ECL_PitchController() = default;

        float controlAttitude(const ECL_ControlData &ctl_data) override;
        float controlEulerRate(const ECL_ControlData &ctl_data) override;
        float controlBodyRate(const ECL_ControlData &ctl_data) override;

        // Additional Setters 
        void setMaxRatePos(float max_rate_pos) {
            max_rate_ = max_rate_pos;
        }

        void setMaxRateNeg(float max_rate_neg) {
            max_rate_neg_ = max_rate_neg;
        }

        void setBodyRateSetpoint(float rate) {
            bodyrate_setpoint_ = math::constrain(rate, -max_rate_neg_, max_rate_);
        }

        void setRollff(float roll_ff) {
            roll_ff_ = roll_ff;
        }

    protected:
        float max_rate_neg_{0.0f};
        float roll_ff_{0.0f};
};
}