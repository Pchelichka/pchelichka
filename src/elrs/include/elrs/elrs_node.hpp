#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <iostream> // for cerr and endl
#include <string.h> // string function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <algorithm> // for std::clamp

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
	// current packet to send
	uint8_t crsfPacket_[CRSF_PACKET_SIZE];
	// telemetry read buffer TODO: check size
	uint8_t readBuf_[64];

	void SetDefaultRcChannels();

	void SerialLoop(); 

	void RcChannelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg); 

public:
    ELRSNode();
};