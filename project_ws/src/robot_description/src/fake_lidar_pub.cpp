#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

class FakeLidarPublisher : public rclcpp::Node
{
public:
    FakeLidarPublisher() : Node("fake_lidar_pub")
    {
        // Create publisher for laser scan data
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        
        // Create timer to publish at 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FakeLidarPublisher::publish_laser_scan, this));
        
        RCLCPP_INFO(this->get_logger(), "Fake LIDAR publisher started");
    }

private:
    void publish_laser_scan()
    {
        auto scan = sensor_msgs::msg::LaserScan();
        
        // Set header
        scan.header.stamp = this->now();
        scan.header.frame_id = "rplidar_link";
        
        // Set scan parameters (360 degree scan)
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = M_PI / 180.0; // 1 degree resolution
        scan.time_increment = 0.0;
        scan.scan_time = 0.1; // 10 Hz
        scan.range_min = 0.12;
        scan.range_max = 6.0;
        
        // Generate fake ranges (360 points)
        int num_readings = 360;
        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);
        
        for (int i = 0; i < num_readings; ++i)
        {
            // Create a simple pattern: walls at 3m distance with some variation
            double angle = scan.angle_min + i * scan.angle_increment;
            double range = 3.0 + 0.5 * sin(4 * angle) + 0.2 * sin(8 * angle);
            
            // Add some noise
            range += 0.05 * ((double)rand() / RAND_MAX - 0.5);
            
            // Clamp to valid range
            if (range < scan.range_min) range = scan.range_min;
            if (range > scan.range_max) range = scan.range_max;
            
            scan.ranges[i] = range;
            scan.intensities[i] = 100.0; // Fake intensity value
        }
        
        publisher_->publish(scan);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeLidarPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}