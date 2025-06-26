#include "elrs/elrs_node.hpp"

// set default values to rcChannels
void ELRSNode::SetDefaultRcChannels() {
	for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
		rcChannels_[i] = RC_MID;
	}
	rcChannels_[THROTTLE] = RC_MIN;
	rcChannels_[AUX1] = RC_MIN;
	rcChannels_[AUX2] = RC_MIN;
	rcChannels_[AUX3] = RC_MAX;
}

// handle each incoming byte from the ELRS module
void ELRSNode::ProcessCrsfByte(uint8_t byte) {
	static uint8_t crsf_frame_buffer[CRSF_FRAME_SIZE_MAX * 2];
	static uint8_t crsf_frame_buffer_pos  = 0;
	// record incoming byte in buffer
	crsf_frame_buffer[crsf_frame_buffer_pos++] = byte;
	if (crsf_frame_buffer_pos > 3) {
		uint8_t expected_frame_len = crsf_frame_buffer[CRSF_FRAME_LENGTH_ADDRESS];
		uint8_t expected_frame_type = crsf_frame_buffer[CRSF_FRAME_TYPE_ADDRESS];
		int full_frame_length = expected_frame_len + 2;
		if (expected_frame_type == CRSF_PACKET_TYPE_RADIO_ID) {
			full_frame_length = expected_frame_len + 5;
		}
		if (full_frame_length > CRSF_FRAME_SIZE_MAX) {
			RCLCPP_WARN(NODE_LOGGER, "Frame error: size if too large = %d", full_frame_length);
			crsf_frame_buffer_pos = 0;
			return;
		} else if (crsf_frame_buffer_pos >= full_frame_length) {
			if (ParseCrsfPacket(crsf_frame_buffer, crsf_frame_buffer_pos)) {
				crsf_frame_buffer_pos -= full_frame_length; // subtract frame size from write pointer
				memcpy(crsf_frame_buffer, &crsf_frame_buffer[full_frame_length], crsf_frame_buffer_pos); // put the remaining bytes into beginning	
			} else {
				crsf_frame_buffer_pos = 0;
			}
		}
	}
}

// parse a full CRSF packet to extract its data
bool ELRSNode::ParseCrsfPacket(uint8_t* buffer, size_t buffer_length) {
	const uint8_t type = buffer[2];
	const uint8_t* payload = &buffer[3];
	if (CheckCrc(buffer, buffer_length)) {
		switch (type) {
			case CRSF_PACKET_TYPE_LINK_STATISTICS:
			{
				int8_t rssi1 = payload[0] >= 128 ? payload[0] - 256 : payload[0];
				int8_t rssi2 = payload[1] >= 128 ? payload[1] - 256 : payload[1];
				uint8_t lq = payload[2];
				uint8_t mode = payload[5];
				RCLCPP_INFO(NODE_LOGGER, "Telemetry: RSSI=%d/%d LQ=%d:%d", rssi1, rssi2, mode, lq);
				break;
			}
			case CRSF_PACKET_TYPE_ATTITUDE:
				current_pitch_ = ((int16_t)((payload[0] << 8) | payload[1])) / 10000.0f;
				current_roll_  = ((int16_t)((payload[2] << 8) | payload[3])) / 10000.0f;
				current_yaw_   = ((int16_t)((payload[4] << 8) | payload[5])) / 10000.0f;

        		RCLCPP_INFO(NODE_LOGGER, "Attitude: Roll=%.2f rad, Pitch=%.2f rad, Yaw=%.2f rad", current_roll_, current_pitch_, current_yaw_);
				break;
			default:
				RCLCPP_INFO(NODE_LOGGER, "Received packet type 0x%02X", type);
				break;
		}
		return true;
	} else {
		RCLCPP_WARN(NODE_LOGGER, "CRC error, packet type: 0x%02X", type);
		return false;
	}
}

// Periodically communicates over serial with the ELRS module 
void ELRSNode::SerialLoop() {
	// set failsafe value
	// auto now = std::chrono::steady_clock::now();
	// if (rcChannels[AUX1] == RC_MAX || (now - last_rc_update_ > FAILSAFE_TIMEOUT)) {
	// 	RCLCPP_WARN(this->get_logger(), "Failsafe activated - no new RC data received");
	// 	setDefaultRcChannels();
	// }
	CrsfPrepareChannelsPacket(crsfPacket_, rcChannels_);
	write(fd, crsfPacket_, CRSF_PACKET_SIZE);           

	int numBytes = read(fd, readBuf_, sizeof(readBuf_));

	if (numBytes > 0) {
		for (int i = 0; i < numBytes; ++i) {
			ProcessCrsfByte(readBuf_[i]);
		}
	}
}

void ELRSNode::RcChannelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
	if (msg->data.size() != CRSF_MAX_CHANNEL) {
		RCLCPP_WARN(this->get_logger(), "Received RC channel data with wrong size: %zu", msg->data.size());
		return;
	}

	for (size_t i = 0; i < CRSF_MAX_CHANNEL; ++i) {
		rcChannels_[i] = msg->data[i];
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
	tty.c_cc[VTIME] = 0;            // no read timeout

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
	// receicing channels from controller
	channels_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
	"rc_channels", 10, 
	[this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
		this->RcChannelsCallback(msg);
	});
	// interaction with ELRS module
	loop_timer_ = this->create_wall_timer(
		CRSF_TIME_NEEDED_PER_FRAME_US,
		std::bind(&ELRSNode::SerialLoop, this)
	);
	// service definitions
	attitude_srv_ = this->create_service<telemetry_interfaces::srv::GetAttitude>(
		"get_attitude",
		[this](const std::shared_ptr<telemetry_interfaces::srv::GetAttitude::Request> request,
				std::shared_ptr<telemetry_interfaces::srv::GetAttitude::Response> response)
		{
			(void)request;  // unused
			response->success = drone_connected_;
			response->pitch_radians = current_pitch_;
			response->roll_radians = current_roll_;
			response->yaw_radians = current_yaw_;
		});

	battery_voltage_srv_ = this->create_service<telemetry_interfaces::srv::GetBatteryVoltage>(
		"get_battery_voltage",
		[this](const std::shared_ptr<telemetry_interfaces::srv::GetBatteryVoltage::Request> request,
				std::shared_ptr<telemetry_interfaces::srv::GetBatteryVoltage::Response> response)
		{
			(void)request;
			response->success = drone_connected_;
			response->voltage = battery_voltage_;
		});
};