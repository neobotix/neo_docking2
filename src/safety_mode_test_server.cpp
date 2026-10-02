/*********************************************************************
MIT License

Copyright (c) 2026 Neobotix GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*********************************************************************/

#include <cstdint>
#include <memory>

#include "neo_actions2/action/relay_board_set_safety_mode.hpp"
#include "neo_msgs2/msg/safety_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class SafetyModeTestServer : public rclcpp::Node
{
public:
  using SetSafetyMode = neo_actions2::action::RelayBoardSetSafetyMode;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SetSafetyMode>;

  SafetyModeTestServer()
  : Node("safety_mode_test_server")
  {
    action_server_ = rclcpp_action::create_server<SetSafetyMode>(
      this,
      "set_safety_mode",
      [this](
        const rclcpp_action::GoalUUID &,
        const std::shared_ptr<const SetSafetyMode::Goal> goal)
      {
        RCLCPP_INFO(
          get_logger(), "Safety mode goal: mode=%u, station=%u",
          static_cast<unsigned int>(goal->set_safety_mode.mode),
          static_cast<unsigned int>(goal->station));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>)
      {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [](const std::shared_ptr<GoalHandle> goal_handle)
      {
        const uint8_t mode = goal_handle->get_goal()->set_safety_mode.mode;
        auto result = std::make_shared<SetSafetyMode::Result>();
        result->success =
        mode == neo_msgs2::msg::SafetyMode::SM_APPROACHING ||
        mode == neo_msgs2::msg::SafetyMode::SM_DEPARTING;

        if (result->success) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
      });

    RCLCPP_INFO(get_logger(), "Test action 'set_safety_mode' is ready");
  }

private:
  rclcpp_action::Server<SetSafetyMode>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyModeTestServer>());
  rclcpp::shutdown();
  return 0;
}
