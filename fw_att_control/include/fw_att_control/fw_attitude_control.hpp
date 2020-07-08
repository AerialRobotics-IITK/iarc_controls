#pragma once

#include <ecl_controllers/ecl_pitch_controller.hpp>
#include <ecl_controllers/ecl_roll_controller.hpp>
#include <ecl_controllers/ecl_wheel_controller.hpp>
#include <ecl_controllers/ecl_yaw_controller.hpp>

#include <msg/vehicle_attitude.h>
#include <msg/vehicle_attitude_setpoint.h>
#include <msg/battery_status.h>
#include <msg/vehicle_local_position.h>
#include <msg/manual_control_setpoint.h>
#include <msg/parameter_update.h>
#include <msg/vehicle_rates_setpoint.h>
#include <msg/vehicle_control_mode.h>
#include <msg/vehicle_land_detected.h>
#include <msg/vehicle_status.h>
#include <msg/vehicle_angular_velocity.h>

#include <ros/ros.h>

namespace ariitk::fw_att_control {

class FixedwingAttitudeControl {
    public:
    FixedwingAttitudeControl(bool vtol = false);
    ~FixedwingAttitudeControl();
    void init(ros::NodeHandle& nh);

    private:
    void run();
    ros::Subscriber att_sub_; //vehicle attitude
    ros::Subscriber att_sp_sub_; //vehicle attitude setpoint
    ros::Subscriber battery_status_sub_; //batter status

    // nav_msgs/Odometry or custom?
    ros::Subscriber local_pos_sub_; //local position subscription

    ros::Subscriber manual_control_setpoint_sub_; //notification of manual control updates
    ros::Subscriber parameter_update_sub_; // notification of parameter updates
    ros::Subscriber rates_sp_sub_; //vehicle rates setpoint
    ros::Subscriber vcontrol_mode_sub_; //vehicle status subscription
    ros::Subscriber vehicle_land_detected_sub_; // vehicle land detected subscription
    ros::Subscriber vehicle_status_sub_; // vehicle status 
    ros::Subscriber vehicle_rates_sub_; //vehicle angular velocity subscription

    ECL_RollController roll_ctrl_;
    ECL_PitchController pitch_ctrl_;
    ECL_YawController yaw_ctrl_;
    ECL_WheelController wheel_ctrl;

    void controlFlaps(const float dt);

    int parametersUpdate(); //Need to figure this out

    void vehicleControlModePoll();
    void vehicleManualPoll();
    void vehicleAttitudeSetpointPoll();
    void vehicleRatesSetpointPoll();
    void vehicleLandDetectedPoll();

    float getAirspeedAndUpdateScaling();


};

}