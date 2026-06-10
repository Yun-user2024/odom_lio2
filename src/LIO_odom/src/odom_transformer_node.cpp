#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomTransformerNode : public rclcpp::Node
{
public:
  OdomTransformerNode()
  : Node("lio_odom_transformer")
  {
    declare_parameter<std::string>("input_odom_topic", "/Odometry");
    declare_parameter<std::string>("output_odom_topic", "/lio_odom");
    declare_parameter<std::string>("input_parent_frame", "camera_init");
    declare_parameter<std::string>("input_child_frame", "body");
    declare_parameter<std::string>("output_parent_frame", "odom");
    declare_parameter<std::string>("output_child_frame", "base_link");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<std::vector<double>>("base_to_livox.translation", {0.1, 0.0, 0.11});
    declare_parameter<std::vector<double>>("base_to_livox.rpy", {-0.034, 0.56395, 0.0});
    declare_parameter<std::vector<double>>("livox_to_body.translation", {-0.012255, -0.016787, 0.050150});
    declare_parameter<std::vector<double>>("livox_to_body.rpy", {0.0032322401494167274, -0.017154571846166722, 0.003817108262238392});
    declare_parameter<bool>("debug.enable", true);
    declare_parameter<double>("debug.log_period_sec", 1.0);

    input_odom_topic_ = get_parameter("input_odom_topic").as_string();
    output_odom_topic_ = get_parameter("output_odom_topic").as_string();
    input_parent_frame_ = get_parameter("input_parent_frame").as_string();
    input_child_frame_ = get_parameter("input_child_frame").as_string();
    output_parent_frame_ = get_parameter("output_parent_frame").as_string();
    output_child_frame_ = get_parameter("output_child_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    debug_enable_ = get_parameter("debug.enable").as_bool();
    debug_log_period_sec_ = get_parameter("debug.log_period_sec").as_double();

    const auto base_to_livox_translation = get_parameter("base_to_livox.translation").as_double_array();
    const auto base_to_livox_rpy = get_parameter("base_to_livox.rpy").as_double_array();
    const auto livox_to_body_translation = get_parameter("livox_to_body.translation").as_double_array();
    const auto livox_to_body_rpy = get_parameter("livox_to_body.rpy").as_double_array();

    validate_vector_size(base_to_livox_translation, "base_to_livox.translation");
    validate_vector_size(base_to_livox_rpy, "base_to_livox.rpy");
    validate_vector_size(livox_to_body_translation, "livox_to_body.translation");
    validate_vector_size(livox_to_body_rpy, "livox_to_body.rpy");

    base_to_livox_ = make_transform(base_to_livox_translation, base_to_livox_rpy);
    livox_to_body_ = make_transform(livox_to_body_translation, livox_to_body_rpy);
    input_child_to_output_child_ = livox_to_body_.inverse() * base_to_livox_.inverse();

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::QoS(20));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_,
      rclcpp::QoS(20),
      std::bind(&OdomTransformerNode::odom_callback, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(get_logger(), "Subscribed raw odom: %s", input_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing transformed odom: %s", output_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Transforming %s->%s into %s->%s",
      input_parent_frame_.c_str(), input_child_frame_.c_str(),
      output_parent_frame_.c_str(), output_child_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Debug logging: %s, period: %.2f sec",
      debug_enable_ ? "enabled" : "disabled", debug_log_period_sec_);
  }

private:
  static void validate_vector_size(const std::vector<double> & values, const std::string & name)
  {
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly 3 values");
    }
  }

  static tf2::Transform make_transform(
    const std::vector<double> & translation,
    const std::vector<double> & rpy)
  {
    tf2::Quaternion quaternion;
    quaternion.setRPY(rpy[0], rpy[1], rpy[2]);
    quaternion.normalize();

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(translation[0], translation[1], translation[2]));
    transform.setRotation(quaternion);
    return transform;
  }

  static tf2::Transform odom_msg_to_tf(const nav_msgs::msg::Odometry & msg)
  {
    tf2::Quaternion quaternion(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);
    quaternion.normalize();

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z));
    transform.setRotation(quaternion);
    return transform;
  }

  static void tf_to_pose(const tf2::Transform & transform, geometry_msgs::msg::Pose & pose)
  {
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation = tf2::toMsg(transform.getRotation());
  }

  static bool is_finite_pose(const geometry_msgs::msg::Pose & pose)
  {
    return std::isfinite(pose.position.x) &&
           std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  static double quaternion_norm(const geometry_msgs::msg::Quaternion & quaternion)
  {
    return std::sqrt(
      quaternion.x * quaternion.x +
      quaternion.y * quaternion.y +
      quaternion.z * quaternion.z +
      quaternion.w * quaternion.w);
  }

  void maybe_log_debug(
    const nav_msgs::msg::Odometry & input,
    const nav_msgs::msg::Odometry & output,
    const tf2::Transform & output_parent_to_base)
  {
    if (!debug_enable_) {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    if (last_debug_log_time_.nanoseconds() != 0) {
      const double elapsed = (now - last_debug_log_time_).seconds();
      if (elapsed < debug_log_period_sec_) {
        return;
      }
    }
    last_debug_log_time_ = now;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(output_parent_to_base.getRotation()).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
      get_logger(),
      "odom debug: raw_count=%zu, published_count=%zu, raw_frame=%s->%s, out_frame=%s->%s, raw_xyz=[%.3f %.3f %.3f], out_xyz=[%.3f %.3f %.3f], out_rpy=[%.3f %.3f %.3f]",
      raw_odom_count_,
      published_odom_count_,
      input.header.frame_id.c_str(),
      input.child_frame_id.c_str(),
      output.header.frame_id.c_str(),
      output.child_frame_id.c_str(),
      input.pose.pose.position.x,
      input.pose.pose.position.y,
      input.pose.pose.position.z,
      output.pose.pose.position.x,
      output.pose.pose.position.y,
      output.pose.pose.position.z,
      roll,
      pitch,
      yaw);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    ++raw_odom_count_;

    if (!is_finite_pose(msg->pose.pose)) {
      ++invalid_odom_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received invalid raw odom pose: count=%zu, invalid_count=%zu",
        raw_odom_count_,
        invalid_odom_count_);
      return;
    }

    const double raw_quaternion_norm = quaternion_norm(msg->pose.pose.orientation);
    if (raw_quaternion_norm < 1e-6) {
      ++invalid_odom_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received near-zero raw odom quaternion: count=%zu, invalid_count=%zu, norm=%.9f",
        raw_odom_count_,
        invalid_odom_count_,
        raw_quaternion_norm);
      return;
    }

    const tf2::Transform input_parent_to_child = odom_msg_to_tf(*msg);
    const tf2::Transform output_parent_to_base = input_parent_to_child * input_child_to_output_child_;

    nav_msgs::msg::Odometry output = *msg;
    output.header.frame_id = output_parent_frame_;
    output.child_frame_id = output_child_frame_;
    tf_to_pose(output_parent_to_base, output.pose.pose);

    if (!is_finite_pose(output.pose.pose)) {
      ++invalid_odom_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Computed invalid output odom pose: raw_count=%zu, invalid_count=%zu",
        raw_odom_count_,
        invalid_odom_count_);
      return;
    }

    odom_publisher_->publish(output);
    ++published_odom_count_;

    maybe_log_debug(*msg, output, output_parent_to_base);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = msg->header.stamp;
      transform_msg.header.frame_id = output_parent_frame_;
      transform_msg.child_frame_id = output_child_frame_;
      transform_msg.transform.translation.x = output.pose.pose.position.x;
      transform_msg.transform.translation.y = output.pose.pose.position.y;
      transform_msg.transform.translation.z = output.pose.pose.position.z;
      transform_msg.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform_msg);
    }
  }

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string input_parent_frame_;
  std::string input_child_frame_;
  std::string output_parent_frame_;
  std::string output_child_frame_;
  bool publish_tf_;
  bool debug_enable_;
  double debug_log_period_sec_;
  std::size_t raw_odom_count_ = 0;
  std::size_t published_odom_count_ = 0;
  std::size_t invalid_odom_count_ = 0;
  rclcpp::Time last_debug_log_time_{0, 0, RCL_ROS_TIME};

  tf2::Transform base_to_livox_;
  tf2::Transform livox_to_body_;
  tf2::Transform input_child_to_output_child_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTransformerNode>());
  rclcpp::shutdown();
  return 0;
}
