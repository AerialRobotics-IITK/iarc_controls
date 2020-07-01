#include "ecl_controller.hpp"

namespace ariitk::fw_att_control {

static constexpr float CONSTANTS_ONE_G = 9.80665f;						// m/s^2

class ECL_YawController : public ECL_Controller {
    public:
    ECL_YawController() = default;
    ~ECL_YawController() = default;

    float controlAttitude(const ECL_ControlData &ctl_data) override;
    float controlEulerRate(const ECL_ControlData &ctl_data) override;
    float controlBodyRate(const ECL_ControlData &ctl_data) override;

    // Additional Setters 
    void setCoordinatedMinSpeed(float coordinated_min_speed) {
        coordinated_min_speed_ = coordinated_min_speed;
    } 

    void setCoordinatedMethod(int32_t coordinated_method) {
        coordinated_method_ = coordinated_method;
    }

    enum {
        COORD_METHOD_OPEN = 0,
        COORD_METHOD_CLOSEACC = 1
    };

    protected:
    float coordinated_min_speed_{1.0f};
    float max_rate_{0.0f};
    int32_t coordinated_method_{COORD_METHOD_OPEN};

    float controlAttitudeIMPLOpenLoop(const ECL_ControlData &ctl_data);
    float controlAttitudeIMPLAccClosedLoop(const ECL_ControlData &ctl_data);
};

}