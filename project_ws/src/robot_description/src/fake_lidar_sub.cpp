#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <limits>
#include <algorithm>

class FakeLidarSubscriber : public rclcpp::Node
{
public:
    FakeLidarSubscriber() : Node("fake_lidar_sub")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&FakeLidarSubscriber::scan_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Fake LIDAR subscriber started");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Find minimum range in the scan
        auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
        if (min_it != msg->ranges.end()) {
            RCLCPP_INFO(this->get_logger(), "Min range: %.2f m", *min_it);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeLidarSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}