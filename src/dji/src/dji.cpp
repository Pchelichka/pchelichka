#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb.h>
#include <err.h>
#include <chrono>

void die(const std::string& message, int code) {
	RCLCPP_FATAL(rclcpp::get_logger("dji_node"), "%s %d", message.c_str(), code);
	rclcpp::shutdown();  
}

struct ctx {
	std::chrono::system_clock::time_point last;
	int bytes;
};

class DJINode : public rclcpp::Node
{
private:
	libusb_device_handle *dev_handle_ = nullptr;
	rclcpp::TimerBase::SharedPtr timer_;
	ctx myctx = {std::chrono::system_clock::now(), 0};
	struct libusb_transfer *xfr = libusb_alloc_transfer(0);
	std::vector<uint8_t> buf;
	void poll_usb_events() {
		timeval tv{0, 1000};  // 1 ms timeout
		int res = libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
		if (res != LIBUSB_SUCCESS) {
			die("libusb_handle_events returned error: ", res);
		}
	}
public:
	DJINode()
		: Node("dji_node")
	{
		buf.resize(512);
		int err = 0;
		RCLCPP_INFO(this->get_logger(), "DJI Node has been initialized.");
		if ((err = libusb_init(NULL)) < 0) { die("Initialize fail: ", err); }
		RCLCPP_INFO(this->get_logger(), "Opening device...");
		dev_handle_ = libusb_open_device_with_vid_pid(NULL, 0x2ca3, 0x001f);
		if (!dev_handle_) { die("Opening device failed: ", 1); }
		int reset_result = libusb_reset_device(dev_handle_);
		if (reset_result != LIBUSB_SUCCESS) {
			RCLCPP_WARN(this->get_logger(), "Device reset failed: %s", libusb_error_name(reset_result));
		} else {
			RCLCPP_INFO(this->get_logger(), "Device successfully reset.");
		}

		RCLCPP_INFO(this->get_logger(), "Claiming interface...");
		if ((err = libusb_claim_interface(dev_handle_, 3)) < 0) { die("claim interface fail ", err); }

		RCLCPP_INFO(this->get_logger(), "Sending magic packet...");
		int tx = 0;
		uint8_t data[] = {0x52, 0x4d, 0x56, 0x54};
		if ((err = libusb_bulk_transfer(dev_handle_, 0x03, data, 4, &tx, 500)) < 0) {
			RCLCPP_ERROR(this->get_logger(), "ERROR: No data transmitted to device: %d", err); //don't exit
		}

		libusb_fill_bulk_transfer(xfr, dev_handle_, 0x84, buf.data(), buf.size(), [](struct libusb_transfer *xfer){
			// fwrite(xfer->buffer, sizeof(uint8_t), xfer->actual_length, stdout); // todo change to pipe
			ctx* c = (ctx*) xfer->user_data;
			if (std::chrono::duration_cast<std::chrono::milliseconds>((std::chrono::system_clock::now() - c->last)).count() > (c->bytes? 100 : 2000)) {
				auto now = std::chrono::system_clock::now();
				auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now - c->last).count();
				seconds = std::max<int64_t>(seconds, 1); // prevent divide-by-zero
				RCLCPP_INFO(rclcpp::get_logger("dji_node"), "rx %ld kb/s", (c->bytes / seconds) / 1000);
				c->last = std::chrono::system_clock::now();
				c->bytes = 0;
			}
			c->bytes += xfer->actual_length;
			int err;
			if ((err = libusb_submit_transfer(xfer)) < 0) { die("Reading init failed: ", err); }
		}, &myctx, 100);
		if ((err = libusb_submit_transfer(xfr)) < 0) { die("read init fail ", err); }

		timer_ = this->create_wall_timer(std::chrono::microseconds(10), std::bind(&DJINode::poll_usb_events, this));
	}

	~DJINode()
	{
		RCLCPP_INFO(this->get_logger(), "DJI Node is being destroyed.");
		timer_->cancel();
		if (dev_handle_) {
			libusb_release_interface(dev_handle_, 3);
			libusb_close(dev_handle_);
		}
		libusb_exit(nullptr);
	}
};

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DJINode>());
	rclcpp::shutdown();
	return 0;
}
