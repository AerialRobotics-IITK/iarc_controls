#include <agent_state_machine/agent_state_machine_base.hpp>

namespace ariitk::agent_state_machine {

void StateMachineBase::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("verbose/transition", verbose_);
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("land_height", land_height_);

    has_payload = true;
    mast_detected = false;

    odom_sub_ = nh.subscribe("odometry", 1, &StateMachineBase::odometryCallback, this);
    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
}

void StateMachineBase::takeoff(const Takeoff& cmd) {
    echo("Taking off!");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, hover_height_);
}

void StateMachineBase::findMast(const Search& cmd) {
    echo("Searching for Mast...");
    mast_detected = true;

    while (ros::ok() && !mast_detected) {
        // TODO: Implement search routine to look for mast
    }
}

void StateMachineBase::detachBlock(const RemoveBlock& cmd) {
    echo("Removing block on mast...");
    mast_detected = false;

    while (ros::ok() && mast_detected) {
        // TOOD: Behaviour to yank existing block out
    }
}

void StateMachineBase::attachBlock(const PlaceBlock& cmd) {
    echo("Placing block on mast...");
    has_payload = false;

    while (ros::ok() && has_payload) {
        // TODO: Behaviour to place our communication block
    }
}

void StateMachineBase::hover(const Hold& cmd) {
    echo("In Position Hold");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, cmd.hold_height);
}

void StateMachineBase::land(const Terminate& cmd) {
    echo("Landing!");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, land_height_);
}

void StateMachineBase::publishPoseCommand(const double& x, const double& y, const double& z) {
    geometry_msgs::PoseStamped cmd_msg;

    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.pose.position.x = x;
    cmd_msg.pose.position.y = y;
    cmd_msg.pose.position.z = z;

    cmd_pose_pub_.publish(cmd_msg);
}

}  // namespace ariitk::agent_state_machine
