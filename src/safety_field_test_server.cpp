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

#include "neo_srvs2/srv/set_safety_field.hpp"
#include "rclcpp/rclcpp.hpp"

class SafetyFieldTestServer : public rclcpp::Node
{
public:
  using SetSafetyField = neo_srvs2::srv::SetSafetyField;

  SafetyFieldTestServer()
  : Node("safety_field_test_server")
  {
    service_ = create_service<SetSafetyField>(
      "set_safety_field",
      [this](
        const std::shared_ptr<SetSafetyField::Request> request,
        std::shared_ptr<SetSafetyField::Response> response)
      {
        const uint32_t field_id = request->field_id;
        response->success = field_id == 0 || field_id == 8;

        RCLCPP_INFO(
          get_logger(), "Safety field request: field_id=%u, success=%s",
          field_id,
          response->success ? "true" : "false");
      });

    RCLCPP_INFO(get_logger(), "Test service 'set_safety_field' is ready");
  }

private:
  rclcpp::Service<SetSafetyField>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyFieldTestServer>());
  rclcpp::shutdown();
  return 0;
}
