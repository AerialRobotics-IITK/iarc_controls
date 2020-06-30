#include <state_machine/state_machine.hpp>

using namespace ariitk::state_machine;

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_machine_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    FiniteStateMachine fsm(nh, nh_private);
    fsm.spin();
}
