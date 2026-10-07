#ifndef WHEELTEC_NAV2__FINAL_IMU_SPIN_ACTION_HPP_
#define WHEELTEC_NAV2__FINAL_IMU_SPIN_ACTION_HPP_

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/spin.hpp"

namespace wheeltec_nav2
{

class FinalImuSpinAction
: public nav2_behavior_tree::BtActionNode<nav2_msgs::action::Spin>
{
public:
  FinalImuSpinAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & configuration);

  void on_tick() override;

  static BT::PortsList providedPorts();
};

}  // namespace wheeltec_nav2

#endif  // WHEELTEC_NAV2__FINAL_IMU_SPIN_ACTION_HPP_
