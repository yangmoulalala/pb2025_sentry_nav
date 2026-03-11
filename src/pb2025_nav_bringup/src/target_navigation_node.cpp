#include "target_navigation_node.hpp"

TargetNavigationNode::TargetNavigationNode(const rclcpp::NodeOptions &options)
    : Node("target_navigation_node", options)
{
  // 初始化参数
  target_received_ = false;
  last_game_progress_ = -1;
  
  // 创建客户端用于清理代价地图
  local_costmap_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/red_standard_robot1/local_costmap/clear_entirely");
  global_costmap_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/red_standard_robot1/global_costmap/clear_entirely");
  
  
  // 创建Action客户端
  navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "/red_standard_robot1/navigate_to_pose");
  
  // 等待Action服务器可用
  if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }
  
  // 创建订阅者
  target_position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/red_standard_robot1/target_position", 10,
      std::bind(&TargetNavigationNode::targetPositionCallback, this, std::placeholders::_1));
  
  referee_sub_ = this->create_subscription<rm_interfaces::msg::Referee>(
      "/red_standard_robot1/referee", 10,
      std::bind(&TargetNavigationNode::refereeCallback, this, std::placeholders::_1));
  
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/red_standard_robot1/odometry", 10,
      std::bind(&TargetNavigationNode::odometryCallback, this, std::placeholders::_1));
  
  // 创建定时器，用于定期检查是否需要发送新的目标点
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5000),  // 500ms = 2Hz
      std::bind(&TargetNavigationNode::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "Target navigation node initialized");
}

void TargetNavigationNode::targetPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_position_ = *msg;
  target_received_ = true;
  new_target_available_ = true;
}

void TargetNavigationNode::refereeCallback(const rm_interfaces::msg::Referee::SharedPtr msg)
{
  // 检查game_progress是否从其他状态变为2
  if (last_game_progress_ != 2 && msg->game_progress == 2) {
    RCLCPP_INFO(this->get_logger(), "Game progress changed to 2, clearing costmaps");
    clearCostmaps();
  }
  last_game_progress_ = msg->game_progress;
}

void TargetNavigationNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  current_odometry_ = *msg;
}

void TargetNavigationNode::timerCallback()
{
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!target_received_) return;

    // 计算新目标与上次发送目标的欧氏距离
    double dist = std::hypot(target_position_.x - last_sent_target_.x, 
                             target_position_.y - last_sent_target_.y);

    // 发送条件：有新目标 且 (距离变化显著 或 当前没在导航)
    if (new_target_available_ && (dist > TARGET_DIST_THRESHOLD || !navigation_in_progress_)) {
        sendNavigationGoal();
        new_target_available_ = false;
        last_sent_target_ = target_position_;
    }
}

void TargetNavigationNode::sendNavigationGoal()
{
    // 如果当前有任务在跑，可以考虑是否取消或直接让新 Goal 抢占
    // Nav2 默认支持抢占，所以我们直接发送
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position = target_position_;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = [this](auto handle) {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            navigation_in_progress_ = false;
        } else {
            this->current_goal_handle_ = handle;
            navigation_in_progress_ = true;
        }
    };

    send_goal_options.result_callback = std::bind(&TargetNavigationNode::resultCallback, this, std::placeholders::_1);

    navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
}

void TargetNavigationNode::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
  }
}

void TargetNavigationNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    navigation_in_progress_ = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation reached!");
            retry_count_ = 0;
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Navigation aborted! Potential obstacle. Clearing costmaps...");
            clearCostmaps(); // 失败后自动清理代价地图
            
            if (retry_count_ < MAX_RETRIES) {
                retry_count_++;
                // 标记为新目标可用，让下一次 timer 触发重发
                new_target_available_ = true; 
                RCLCPP_INFO(this->get_logger(), "Retrying goal... (%d/%d)", retry_count_, MAX_RETRIES);
            }
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            break;

        default:
            break;
    }
}

void TargetNavigationNode::clearCostmaps()
{
  // 清理局部代价地图
  auto local_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto local_result = local_costmap_clear_client_->async_send_request(local_request);
  
  // 清理全局代价地图
  auto global_request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto global_result = global_costmap_clear_client_->async_send_request(global_request);
  
  RCLCPP_INFO(this->get_logger(), "Sent requests to clear both local and global costmaps");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}