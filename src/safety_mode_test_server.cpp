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

#include "neo_msgs2/msg/safety_mode.hpp"
#include "neo_srvs2/srv/relay_board_set_safety_mode.hpp"
#include "rclcpp/rclcpp.hpp"

class SafetyModeTestServer : public rclcpp::Node
{
public:
  using SetSafetyMode = neo_srvs2::srv::RelayBoardSetSafetyMode;

  SafetyModeTestServer()
  : Node("safety_mode_test_server")
  {
    service_ = create_service<SetSafetyMode>(
      "set_safety_mode",
      [this](
        const std::shared_ptr<SetSafetyMode::Request> request,
        std::shared_ptr<SetSafetyMode::Response> response)
      {
        const uint8_t mode = request->set_safety_mode.mode;
        response->success =
          mode == neo_msgs2::msg::SafetyMode::SM_APPROACHING ||
          mode == neo_msgs2::msg::SafetyMode::SM_DEPARTING;

        RCLCPP_INFO(
          get_logger(), "Safety mode request: mode=%u, station=%u, success=%s",
          static_cast<unsigned int>(mode),
          static_cast<unsigned int>(request->station),
          response->success ? "true" : "false");
      });

    RCLCPP_INFO(get_logger(), "Test service 'set_safety_mode' is ready");
  }

private:
  rclcpp::Service<SetSafetyMode>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyModeTestServer>());
  rclcpp::shutdown();
  return 0;
}
