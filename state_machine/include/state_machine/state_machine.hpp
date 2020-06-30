#pragma once

#include <state_machine/state_machine_base.hpp>

namespace ariitk::state_machine {

class FiniteStateMachine : public FSMBase {
  public:
    FiniteStateMachine(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

  private:
    void publishCurrState();

    FSMBackend machine_;

    ros::Publisher state_pub_;

    double poll_rate_;
    bool verbose_;
};

}  // namespace ariitk::state_machine
