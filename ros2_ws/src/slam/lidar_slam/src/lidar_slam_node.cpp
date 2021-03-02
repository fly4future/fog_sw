#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <iostream>

class LidarSlamNode : public rclcpp::Node
{
public:
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  LidarSlamNode() : Node("lidar_slam"),
  {
    declare_parameter<string>("pointcloud_topic", "/pointcloud");
    declare_parameter<string>("base_frame_id", "base_link");
    declare_parameter<string>("map_frame_id", "map");

    const std::string cloud_topic = get_parameter("cloud_topic").as_string();
    base_frame_ = get_parameter("base_frame_id").as_string();
    map_frame_ = get_parameter("map_frame_id").as_string();

    RCLCPP_INFO(get_logger()) << "Listening to images from topic '" << cloud_topic << "'";
  }


private:

  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_subscriber_;
  std::string base_frame_, map_frame_;


  void GrabPointCloud2(const CloudMsg::SharedPtr msg)
  {
    std::cout << "GrabImage from frame " << msg->header.frame_id << std::endl;
    cv_bridge::CvImagePtr m_cvImPtr;

    try
    {
      m_cvImPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat cam2map = slam_->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);

    if (cam2map.empty())
    {
      return;
    }
    std::cout << cam2map << std::endl;

    auto camera2map_transform = cvMatToTransform(cam2map);

  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LidarSlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
