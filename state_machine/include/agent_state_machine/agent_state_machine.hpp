#pragma once

#include <agent_state_machine/agent_state_machine_base.hpp>

namespace ariitk::agent_state_machine {

class AgentStateMachine : public StateMachineBase {
  public:
    AgentStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

  private:
    void publishCurrState();
    template<class Event>
    void performTask();

    StateMachineBackend machine_;

    ros::Publisher state_pub_;

    double poll_rate_;
    bool verbose_;
};

}  // namespace ariitk::agent_state_machine
