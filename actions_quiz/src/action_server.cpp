#include <functional>
#include <memory>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "actions_quiz_msg/action/distance.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp" 

class MyActionServer : public rclcpp::Node
{
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Distance>;

  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("server_action_quiz_node", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
      this,
      "distance_as",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

    subscriber_ =  this->create_subscription<nav_msgs::msg::Odometry>
    (
        "/odom", 
        10,
        std::bind
        (
                    &MyActionServer::odomCallback, 
                    this, 
                    std::placeholders::_1
        )
    );

    publisher_ = this->create_publisher<std_msgs::msg::Float32>
    (
        "/total_distance", 
        10
    );
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  float current_velocity = 0.0;
  float current_distance = 0.0;
  std::mutex velocity_mutex_;
  std::mutex distance_mutex_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Distance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    {
      std::lock_guard<std::mutex> lock(distance_mutex_);
      this->current_distance = 0.0;
    }

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto result = std::make_shared<Distance::Result>();

    rclcpp::Rate loop_rate(10);

    for (int i = 0; (i < (goal->seconds * 10)) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        result->total_dist = 0;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      float velocity_copy;
      {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        velocity_copy = this->current_velocity;
      }

      float new_distance;
      {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        this->current_distance += (abs(velocity_copy) * 0.1);
        new_distance = this->current_distance;
      }

      feedback->current_dist = new_distance;

      std_msgs::msg::Float32 distance_msg;
      distance_msg.data = new_distance;
      publisher_->publish(distance_msg);

      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      float final_distance;
      {
        std::lock_guard<std::mutex> lock(distance_mutex_);
        final_distance = this->current_distance;
      }

      result->status = true;
      result->total_dist = final_distance;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // store current velocity (used to find distance)
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        this->current_velocity = msg->twist.twist.linear.x;
    }
};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}