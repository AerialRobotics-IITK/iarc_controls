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
    auto state_publish_thread = std::async(std::launch::async, [this] { publishCurrState(); });

    // First, look for the mast
    performTask<Search>();  // exits when mast is found
    // Second, remove the existing block
    performTask<RemoveBlock>();  // exits once block is removed
    // Now, attach payload in place of the block
    performTask<PlaceBlock>();  // exits once block is in place
    // Die!
    machine_.process_event(Terminate());

    machine_.stop();
}

template<class Event>
void FiniteStateMachine::performTask() {
    // since every task must come back to hover, these two calls are always together
    machine_.process_event(Event());
    machine_.process_event(Hold(curr_height));  // hover at the current height by default
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
