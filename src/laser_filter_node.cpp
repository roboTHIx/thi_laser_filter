
#include <iostream> 
#include <rclcpp/rclcpp.hpp> 

// #include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>


//node class
class RangeFilter : public rclcpp::Node
{
public:
  RangeFilter() : Node("scan2cloud")
  {
    //create parameter for QOS pub/sub
    this->declare_parameter("qos_scan_pub", std::string("sensor_data"));
    this->declare_parameter("qos_scan_sub", std::string("sensor_data"));
    this->declare_parameter("min_range", 0.4);


    auto qos_scan_out_str = this->get_parameter("qos_scan_pub").as_string();
    auto qos_scan_str = this->get_parameter("qos_scan_sub").as_string();
    _min_range = this->get_parameter("min_range").as_double();
  
    RCLCPP_INFO(this->get_logger(), "###########################################################");
    RCLCPP_INFO(this->get_logger(), "##  Parameter  ##");

    auto qos_scan_out = rclcpp::QoS(10); //default
    if(qos_scan_out_str == "sensor_data")
    {
      qos_scan_out = rclcpp::SensorDataQoS();
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Cloud Pub: sensor_data");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Cloud Pub: default");
    }
    auto qos_scan = rclcpp::QoS(10); //default
    if(qos_scan_str == "sensor_data")
    {
      qos_scan = rclcpp::SensorDataQoS();
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Scan Sub: sensor_data");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "QOS Profile for Scan Sub: default");
    }

    RCLCPP_INFO(this->get_logger(), "Min Range: %f", _min_range);
    RCLCPP_INFO(this->get_logger(), "###########################################################");


    //create publisher
    _pub_scan = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", qos_scan_out);
    //create subscriber
    _sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", qos_scan, std::bind(&RangeFilter::sub_scan_callback, this, std::placeholders::_1));
  }

private:
  void sub_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    //create new message
    sensor_msgs::msg::LaserScan scan_filtered = *msg;
    //filter
    for(std::size_t i = 0; i < msg->ranges.size(); i++)
    {
      if(msg->ranges[i] < _min_range)
      {
        scan_filtered.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    //publish
    _pub_scan->publish(scan_filtered);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _pub_scan;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _sub_scan;

  double _min_range = 0.0;

  //projector
  // laser_geometry::LaserProjection _projector;
};



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RangeFilter>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
