#include <iostream> // for cerr and endl
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <algorithm> // for std::clamp
#include <chrono> // for the timer

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


const uint8_t RADIO_ADDRESS = 0xEA;
const uint8_t ADDR_MODULE = 0xEE;  //  Crossfire transmitter
const uint8_t TYPE_CHANNELS = 0x16;

// internal crsf variables
constexpr int RC_MIN = 988;
constexpr int RC_MID = 1500;
constexpr int RC_MAX = 2012;

constexpr int CRSF_MIN = 172;
constexpr int CRSF_MAX = 1811;
const speed_t BAUDRATE = B921600;             // Crossfire/ELRS typically uses 400000
constexpr std::chrono::microseconds CRSF_TIME_NEEDED_PER_FRAME_US(1100);
constexpr std::chrono::milliseconds FAILSAFE_TIMEOUT(1000);
//   Hz     ms
//   25     40000
//   50     20000
//   100    10000
//   250    4000
//   500    2000 (???)
const int CRSF_TIME_BETWEEN_FRAMES_US = 4000; // 4 ms 250Hz
const int CRSF_MAX_CHANNEL = 16;
const int CRSF_FRAME_SIZE_MAX = 64;
const int SERIAL_BAUDRATE = 400000;
const int CRSF_MSP_RX_BUF_SIZE = 128;
const int CRSF_MSP_TX_BUF_SIZE = 128;
const int CRSF_PAYLOAD_SIZE_MAX   = 60;
const int CRSF_PACKET_LENGTH = 22;
const int CRSF_PACKET_SIZE  = 26;
const int CRSF_FRAME_LENGTH = 24;   // length of type + payload + = crc;
const int CRSF_CHANNEL_BITS = 11;

int map_rc_to_crsf(int rc_value) {

    // Clamp input to expected RC range
    rc_value = std::clamp(rc_value, RC_MIN, RC_MAX);

    // Scale linearly from RC to CRSF
    float scale = (float)(CRSF_MAX - CRSF_MIN) / (RC_MAX - RC_MIN);
    return static_cast<int>((rc_value - RC_MIN) * scale + CRSF_MIN);
}

enum chan_order{
   ROLL,
   PITCH,
   THROTTLE,
   YAW,
   AUX1,  // (CH5)  ARM switch for Expresslrs
   AUX2,  // (CH6)  angel / airmode change
   AUX3,  // (CH7)  flip after crash
   AUX4,  // (CH8)
   AUX5,  // (CH9)
   AUX6,  // (CH10)
   AUX7,  // (CH11)
   AUX8,  // (CH12)
   AUX9,  // (CH12)
   AUX10,  // (CH12)
   AUX11,  // (CH12)
   AUX12,  // (CH12) // telemetry
};


class ELRSNode : public rclcpp::Node {
private:
	int fd;
	std::chrono::steady_clock::time_point last_rc_update_;
	rclcpp::TimerBase::SharedPtr loop_timer;
	int rcChannels[CRSF_MAX_CHANNEL];
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr channels_sub;
	uint8_t crsfPacket[CRSF_PACKET_SIZE];
	// crc implementation from CRSF protocol document rev7
	static constexpr std::array<uint8_t, 256>  crsf_crc8tab = {
	0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
	0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
	0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
	0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
	0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
	0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
	0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
	0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
	0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
	0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
	0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
	0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
	0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
	0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
	0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
	0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};


	uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
		uint8_t crc = 0;
		for (uint8_t i=0; i < len; i++) {
			crc = crsf_crc8tab[crc ^ *ptr++];
		}
		return crc;
	}

	void crsfPreparePacket(uint8_t packet[]){
		static int channels[CRSF_MAX_CHANNEL];
		/*
			* Map 1000-2000 with middle at 1500 chanel values to
			* 172-1811(CRSF_CHANNEL_MIN-CRSF_CHANNEL_MAX) with middle at 991(CRSF_CHANNEL_MID) S.BUS(andCRSF) protocol requires
			*/
		for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
			channels[i] = map_rc_to_crsf(rcChannels[i]);
		}   
		// packet[0] = UART_SYNC; //Header
		packet[0] = ADDR_MODULE; //Header
		packet[1] = 24;   // length of type (24) + payload + crc
		packet[2] = TYPE_CHANNELS;
		
		packet[3] = (uint8_t) (channels[0] & 0x07FF);
		packet[4] = (uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
		packet[5] = (uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
		packet[6] = (uint8_t) ((channels[2] & 0x07FF)>>2);
		packet[7] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
		packet[8] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
		packet[9] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
		packet[10] = (uint8_t) ((channels[5] & 0x07FF)>>1);
		packet[11] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
		packet[12] = (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
		packet[13] = (uint8_t) ((channels[7] & 0x07FF)>>3);
		packet[14] = (uint8_t) ((channels[8] & 0x07FF));
		packet[15] = (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
		packet[16] = (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6); 
		packet[17] = (uint8_t) ((channels[10] & 0x07FF)>>2);
		packet[18] = (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
		packet[19] = (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
		packet[20] = (uint8_t) ((channels[12] & 0x07FF)>>4  | (channels[13] & 0x07FF)<<7);
		packet[21] = (uint8_t) ((channels[13] & 0x07FF)>>1);
		packet[22] = (uint8_t) ((channels[13] & 0x07FF)>>9  | (channels[14] & 0x07FF)<<2);
		packet[23] = (uint8_t) ((channels[14] & 0x07FF)>>6  | (channels[15] & 0x07FF)<<5);
		packet[24] = (uint8_t) ((channels[15] & 0x07FF)>>3);
		packet[25] = crsf_crc8(&packet[2], CRSF_PACKET_SIZE-3); //CRC
	}

	void setDefaultRcChannels() {
		for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
			rcChannels[i] = RC_MID;
		}
		rcChannels[THROTTLE] = RC_MIN;
		rcChannels[AUX1] = RC_MIN;
		rcChannels[AUX2] = RC_MIN;
		rcChannels[AUX3] = RC_MAX;
	}

	void serialLoop() {
		// set failsafe value
		auto now = std::chrono::steady_clock::now();
		// if (rcChannels[AUX1] == RC_MAX || (now - last_rc_update_ > FAILSAFE_TIMEOUT)) {
		// 	RCLCPP_WARN(this->get_logger(), "Failsafe activated - no new RC data received");
		// 	setDefaultRcChannels();
		// }

		RCLCPP_WARN(this->get_logger(), "AUX3: %d", rcChannels[AUX3]);
		crsfPreparePacket(crsfPacket);
        // RCLCPP_INFO(this->get_logger(), "Writing to serial...");
		write(fd, crsfPacket, CRSF_PACKET_SIZE);           
	}

	void rcChannelsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
		if (msg->data.size() != CRSF_MAX_CHANNEL) {
			RCLCPP_WARN(this->get_logger(), "Received RC channel data with wrong size: %zu", msg->data.size());
			return;
		}

		for (size_t i = 0; i < CRSF_MAX_CHANNEL; ++i) {
			rcChannels[i] = msg->data[i];
		}
	}

public:
    ELRSNode() : Node("elrs_node") {
        RCLCPP_INFO(this->get_logger(), "Starting ELRS interface node!");
		setDefaultRcChannels();
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
		channels_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
		"rc_channels", 10, 
		[this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
			this->rcChannelsCallback(msg);
		});
		loop_timer = this->create_wall_timer(
			CRSF_TIME_NEEDED_PER_FRAME_US,
			std::bind(&ELRSNode::serialLoop, this)
		);
    };
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ELRSNode>());
    rclcpp::shutdown();
    return 0;
}