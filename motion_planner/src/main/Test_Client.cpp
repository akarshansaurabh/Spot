#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/srv/task_test.hpp" // Adjust the path according to your setup

using namespace std::chrono_literals;

class ClientNode : public rclcpp::Node
{
public:
    ClientNode() : Node("client_node")
    {
        client_ = this->create_client<custom_interfaces::srv::TaskTest>("/test_service");

        auto request = std::make_shared<custom_interfaces::srv::TaskTest::Request>();
        request->test = 1;

        // Wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Service call succeeded.");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Service call failed.");
        }
    }

private:
    rclcpp::Client<custom_interfaces::srv::TaskTest>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientNode>());
    rclcpp::shutdown();
    return 0;
}
