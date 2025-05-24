#include "elrs/elrs_node.hpp"

// set default values to rcChannels
void ELRSNode::SetDefaultRcChannels() {
	for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
		rcChannels[i] = RC_MID;
	}
	rcChannels[THROTTLE] = RC_MIN;
	rcChannels[AUX1] = RC_MIN;
	rcChannels[AUX2] = RC_MIN;
	rcChannels[AUX3] = RC_MAX;
}

// Periodically communicates over serial with the ELRS module 
void ELRSNode::SerialLoop() {
	// set failsafe value
	// auto now = std::chrono::steady_clock::now();
	// if (rcChannels[AUX1] == RC_MAX || (now - last_rc_update_ > FAILSAFE_TIMEOUT)) {
	// 	RCLCPP_WARN(this->get_logger(), "Failsafe activated - no new RC data received");
	// 	setDefaultRcChannels();
	// }

	CrsfPrepareChannelsPacket(crsfPacket, rcChannels);
	// RCLCPP_INFO(this->get_logger(), "Writing to serial...");
	write(fd, crsfPacket, CRSF_PACKET_SIZE);           
}

void ELRSNode::RcChannelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
	if (msg->data.size() != CRSF_MAX_CHANNEL) {
		RCLCPP_WARN(this->get_logger(), "Received RC channel data with wrong size: %zu", msg->data.size());
		return;
	}

	for (size_t i = 0; i < CRSF_MAX_CHANNEL; ++i) {
		rcChannels[i] = msg->data[i];
	}
}

// Initilaize ROS2 node, open serial to module and set up timer and topic subscriber
ELRSNode::ELRSNode() : Node("elrs_node") {
	RCLCPP_INFO(this->get_logger(), "Starting ELRS interface node!");
	SetDefaultRcChannels();
	const char* port = "/dev/ttyUSB0";  // Adjust for your device

	// 1. Open the serial port
	fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		std::cerr << "Failed to open port: " << strerror(errno) << std::endl;
		rclcpp::shutdown();
	}

	// 2. Configure the port
	termios tty{};

	if (tcgetattr(fd, &tty) != 0) {
		std::cerr << "Error getting termios: " << strerror(errno) << std::endl;
		rclcpp::shutdown();
	}

	cfsetospeed(&tty, BAUDRATE);
	cfsetispeed(&tty, BAUDRATE);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
	tty.c_cflag &= ~(PARENB | PARODD);  // shut off parity
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		std::cerr << "Error setting termios: " << strerror(errno) << std::endl;
		rclcpp::shutdown();
	}
	RCLCPP_INFO(this->get_logger(), "Starting the loop...");
	channels_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
	"rc_channels", 10, 
	[this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
		this->RcChannelsCallback(msg);
	});
	loop_timer_ = this->create_wall_timer(
		CRSF_TIME_NEEDED_PER_FRAME_US,
		std::bind(&ELRSNode::SerialLoop, this)
	);
};