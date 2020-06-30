#include <state_machine/state_machine.hpp>

#include <future>
#include <std_msgs/String.h>

namespace ariitk::state_machine {

FiniteStateMachine::FiniteStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    nh_private.getParam("poll_rate", poll_rate_);
    nh_private.getParam("verbose/state", verbose_);

    machine_.start();
    machine_.init(nh, nh_private);

    state_pub_ = nh_private.advertise<std_msgs::String>("curr_state", 1);

    // Fly!
    machine_.process_event(Takeoff());
}

void FiniteStateMachine::spin() {
    // TODO: try enqueue for a cleaner impl
    auto state_publish_thread = std::async(std::launch::async, [this] { publishCurrState(); });

    // First, look for the mast
    machine_.process_event(Search());  // exits when mast is found
    machine_.process_event(Hold());
    // Second, remove the existing block
    machine_.process_event(RemoveBlock());  // exits once block is removed
    machine_.process_event(Hold());
    // Now, attach payload in place of the block
    machine_.process_event(PlaceBlock());  // exits once block is in place
    machine_.process_event(Hold());
    // Die!
    machine_.process_event(Terminate());

    machine_.stop();
}

void FiniteStateMachine::publishCurrState() {
    ros::Rate loop_rate(poll_rate_);

    std_msgs::String state_msg;
    while (ros::ok()) {
        ros::spinOnce();
        state_msg.data = state_names[machine_.current_state()[0]];
        state_pub_.publish(state_msg);
        loop_rate.sleep();
    }
}

}  // namespace ariitk::state_machine
