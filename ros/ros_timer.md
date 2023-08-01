```c++
#include <ros/ros.h>
#include <std_msgs/Float64.h>

class TemperatureSensor {
public:
  TemperatureSensor(ros::NodeHandle *nh) {
    // Initialize temperature and ROS publisher
    temperature = 0.0;
    temperaturePublisher =
      nh->advertise<std_msgs::Float64>("/temperature", 10);
  }

  double readTemperatureSensorData() {
    // Code to read data from your sensor
    // Return measured temperature
    return 30.0;
  }

  void publishTemperature() {
    std_msgs::Float64 msg;
    msg.data = readTemperatureSensorData();
    temperaturePublisher.publish(msg);
  }

private:
  double temperature;
  ros::Publisher temperaturePublisher;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "your_sensor_node");
  ros::NodeHandle nh;

  // Create an instance of Temperature sensor
  TemperatureSensor temperatureSensor(&nh);

  // Create a ROS timer for reading data
  // the callback function for the Timer must
  // be bound with std::bind or boost::bind
  ros::Timer timerReadTemperature =
    nh.createTimer(ros::Duration(1.0 / 10.0),
        std::bind(&TemperatureSensor::readTemperatureSensorData, temperatureSensor));

  // Create a ROS timer for publishing temperature
  ros::Timer timerPublishTemperature =
    nh.createTimer(ros::Duration(1.0 / 10.0),
        std::bind(&TemperatureSensor::publishTemperature, temperatureSensor));

  // We can now use spin, or do whatever 
  // we want in this node
  ros::spin();
}
```





```c++
class ParamTest {
 public:
  ParamTest() = default;
  void operator()(dynamic_param::TestConfig& config, uint32_t level) {
    pub_ = nh_.advertise<dynamic_param::TestConfig>("/message_topic", 10);
    pub_.publish(config);
  }
 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

};
```

