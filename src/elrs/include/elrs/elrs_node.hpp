#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <iostream> // for cerr and endl
#include <string.h> // string function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <algorithm> // for std::clamp

#include "telemetry_interfaces/srv/get_attitude.hpp"
#include "telemetry_interfaces/srv/get_battery_voltage.hpp"
#include "util.hpp"
#include "constants.hpp"

class ELRSNode : public rclcpp::Node {
private:
	// usb writer
	int fd;
	// last time receiving update from controls
	std::chrono::steady_clock::time_point last_rc_update_;
	// timer to periodically send channels to ELRS module
	rclcpp::TimerBase::SharedPtr loop_timer_;
	int rcChannels_[CRSF_MAX_CHANNEL];
	// ros2 subscriber to receive rc channels from controls
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr channels_sub_;
	// telemetry services
	rclcpp::Service<telemetry_interfaces::srv::GetAttitude>::SharedPtr attitude_srv_;
	rclcpp::Service<telemetry_interfaces::srv::GetBatteryVoltage>::SharedPtr battery_voltage_srv_;
	// current packet to send
	uint8_t crsfPacket_[CRSF_PACKET_SIZE];
	// telemetry read buffer TODO: check size
	uint8_t readBuf_[64];

	bool drone_connected_ = false;
	float current_pitch_ = 0.0;
	float current_roll_ = 0.0;
	float current_yaw_ = 0.0;
	float battery_voltage_ = 0.0;

	void SetDefaultRcChannels();

	void ProcessCrsfByte(uint8_t byte);
	bool ParseCrsfPacket(uint8_t* buffer, size_t length);

	void SerialLoop(); 

	void RcChannelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg); 

public:
    ELRSNode();
};