#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using std::placeholders::_1;

using namespace std;
using namespace GeographicLib;

class GpsSubscriber : public rclcpp::Node
{
  public:
    GpsSubscriber()
    : Node("gps_local_node") 
    {  
      subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/summit/fix", 10, std::bind(&GpsSubscriber::gps_callback, this, _1));
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

  private:
    
    LocalCartesian proj;
    double x{0.0};
    double y{0.0};
    double z{0.0}; 
    bool reset_{true};
    
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps)
    {
      if (gps->status.status == -1)
      {
        RCLCPP_WARN(this->get_logger(), "GPS status not fix yet");
        return;
      }

      else
      {
        if (reset_)
        {
          proj.Reset(gps->latitude, gps->longitude, gps->altitude);
          reset_ = false;
          RCLCPP_INFO(this->get_logger(), "setting origin to 1st GPS fix");
        }

        proj.Forward(gps->latitude, gps->longitude, gps->altitude, x, y, z);

        RCLCPP_INFO(this->get_logger(), "x : %f, y: %f, z: %f", x, y, z);

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = gps->header.stamp;
        transformStamped.header.frame_id = "enu";
        transformStamped.child_frame_id = "gps_antenna";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        transformStamped.transform.rotation.w = 1.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;     
        tf_broadcaster_->sendTransform(transformStamped);
      }
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsSubscriber>());
  rclcpp::shutdown();
  return 0;
}