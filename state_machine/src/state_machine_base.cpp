#include <geometry_msgs/PoseStamped.h>
#include <state_machine/state_machine_base.hpp>

namespace ariitk::state_machine {

void FSMBase::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("verbose/transition", verbose_);
    nh_private.getParam("hover_height", hover_height_);
    nh_private.getParam("land_height", land_height_);

    has_payload = true;
    mast_detected = false;

    odom_sub_ = nh.subscribe("odometry", 1, &FSMBase::odometryCallback, this);
    cmd_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
}

void FSMBase::takeoff(const Takeoff& cmd) {
    echo("Taking off!");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, hover_height_);
}

void FSMBase::findMast(const Search& cmd) {
    echo("Searching for Mast...");
    mast_detected = true;

    while (ros::ok() && !mast_detected) {
        // TODO: Implement search routine to look for mast
    }
}

void FSMBase::detachBlock(const RemoveBlock& cmd) {
    echo("Removing block on mast...");
    mast_detected = false;

    while (ros::ok() && mast_detected) {
        // TOOD: Behaviour to yank existing block out
    }
}

void FSMBase::attachBlock(const PlaceBlock& cmd) {
    echo("Placing block on mast...");
    has_payload = false;

    while (ros::ok() && has_payload) {
        // TODO: Behaviour to place our communication block
    }
}

void FSMBase::hover(const Hold& cmd) {  // TODO: take hover height from command
    echo("In Position Hold");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, hover_height_);
}

void FSMBase::land(const Terminate& cmd) {
    echo("Landing!");
    publishPoseCommand(mav_pose_.position.x, mav_pose_.position.y, land_height_);
}

void FSMBase::publishPoseCommand(const double& x, const double& y, const double& z) {
    geometry_msgs::PoseStamped cmd_msg;

    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.pose.position.x = x;
    cmd_msg.pose.position.y = y;
    cmd_msg.pose.position.z = z;

    cmd_pose_pub_.publish(cmd_msg);
}

}  // namespace ariitk::state_machine
