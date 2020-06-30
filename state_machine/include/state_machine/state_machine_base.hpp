#pragma once

#include <nav_msgs/Odometry.h>
#include <state_machine/state_machine_def.hpp>

namespace ariitk::state_machine {

class FSMBase : public FSMDef<FSMBase> {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void spin();

    // State names
    // TODO: Test if the order of names is correct
    std::vector<std::string> state_names = {"Rest", "Hover", "Explore", "Detach", "Attach"};

    // State definitions
    struct Rest : public State<Rest> {};
    struct Hover : public State<Hover> {};
    struct Detach : public State<Detach> {};
    struct Attach : public State<Attach> {};
    struct Explore : public State<Explore> {};

    typedef Rest initial_state;

    // Transition events
    struct Takeoff : public Command {};
    struct Search : public Command {};
    struct RemoveBlock : public Command {};
    struct PlaceBlock : public Command {};
    struct Hold : public Command {};
    struct Terminate : public Command {};

    // Guard variables
    bool has_payload;
    bool mast_detected;

    // Transition Guards
    // TODO: Cleaner way?
    template<class Event>
    bool isMastVisible(const Event& cmd) {
        return mast_detected;
    }
    bool needMastSearch(const Search& cmd) {
        return !mast_detected;
    }
    bool hasNoPayload(const Hold& cmd) {
        return !has_payload;
    }
    bool canAttachBlock(const PlaceBlock& cmd) {
        return (has_payload && mast_detected);
    }

    // Transition actions
    void takeoff(const Takeoff& cmd);
    void findMast(const Search& cmd);
    void hover(const Hold& cmd);
    void detachBlock(const RemoveBlock& cmd);
    void attachBlock(const PlaceBlock& cmd);
    void land(const Terminate& cmd);

    // clang-format off
    struct transition_table
        : boost::mpl::vector<
              //      Type     Start          Event          Next            Action                Guard
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                     a_row<    Rest     ,  Takeoff      ,  Hover    ,  &FSMBase::takeoff                                    >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                       row<    Hover    ,  Search       ,  Explore  ,  &FSMBase::findMast     ,  &FSMBase::needMastSearch   >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                       row<    Explore  ,  Hold         ,  Hover    ,  &FSMBase::hover        ,  &FSMBase::isMastVisible    >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                       row<    Hover    ,  RemoveBlock  ,  Detach   ,  &FSMBase::detachBlock  ,  &FSMBase::isMastVisible    >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                     a_row<    Detach   ,  Hold         ,  Hover    ,  &FSMBase::hover                                      >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                       row<    Hover    ,  PlaceBlock   ,  Attach   ,  &FSMBase::attachBlock  ,  &FSMBase::canAttachBlock   >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                       row<    Attach   ,  Hold         ,  Hover    ,  &FSMBase::hover        ,  &FSMBase::hasNoPayload     >,
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
                     a_row<    Hover    ,  Terminate    ,  Rest     ,  &FSMBase::land                                       >
              // +++ ------ + --------- + ------------- + --------- + ----------------------- + -------------------------- +++
              > {};
    // clang-format on

  private:
    void publishPoseCommand(const double& x, const double& y, const double& z);
    void odometryCallback(const nav_msgs::Odometry& odom) {
        mav_pose_ = odom.pose.pose;
    }

    geometry_msgs::Pose mav_pose_;

    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pose_pub_;

    bool verbose_;

    double hover_height_;
    double land_height_;
};

}  // namespace ariitk::state_machine
