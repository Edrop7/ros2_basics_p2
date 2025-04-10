#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("server_services_quiz_node")
  {

    srv_ = create_service<Spin>("rotate", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(
      const std::shared_ptr<Spin::Request> request,
      const std::shared_ptr<Spin::Response> response
      ) {
        auto message = geometry_msgs::msg::Twist();
    
        if (request->direction == "right")
        {   
            // Send velocities to move the robot to the right
            message.angular.z = -1 * request->angular_velocity;
            publisher_->publish(message);

            // wait the specified amount of time and stop
            rclcpp::sleep_for(std::chrono::seconds(request->time));
            message.angular.z = 0.0;
            publisher_->publish(message);

            // Set the response success variable to true
            response->success = true;
        }
        else if (request->direction == "left")
        {
            // Send velocities to move the robot to the left
            message.angular.z = request->angular_velocity;
            publisher_->publish(message);

            // wait the specified amount of time and stop
            rclcpp::sleep_for(std::chrono::seconds(request->time));
            message.angular.z = 0.0;
            publisher_->publish(message);

            // Set the response success variable to false
            response->success = true;
        }
        else {
            response->success = false;
        }
                
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}