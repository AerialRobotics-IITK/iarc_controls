#pragma once 

#include "ecl_controller.hpp"

namespace ariitk::fw_att_control {
class ECL_RollController : public ECL_Controller {
    public:
        ECL_RollController() = default;
        ~ECL_RollController() = default;

        float controlAttitude(const ECL_ControlData &ctl_data) override;
	    float controlEulerRate(const ECL_ControlData &ctl_data) override;
	    float controlBodyRate(const ECL_ControlData &ctl_data) override;
};
}