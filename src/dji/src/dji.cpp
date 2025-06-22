#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb.h>
#include <err.h>
#include <sys/timeb.h>

struct ctx {
	double last;
	int bytes;
};

class DJINode : public rclcpp::Node
{
private:
	double now() {
		struct timeb timebuffer;
		ftime(&timebuffer);
		return timebuffer.time+ ((double)timebuffer.millitm)/1000;
	}
	void die(const std::string& message, int code) {
		RCLCPP_FATAL(this->get_logger(), "%s %d", message.c_str(), code);
		rclcpp::shutdown();  
		std::exit(EXIT_FAILURE); 
	}
public:
	DJINode()
		: Node("dji_node")
	{
		int err = 0;
		RCLCPP_INFO(this->get_logger(), "DJI Node has been initialized.");
		if ((err = libusb_init(NULL)) < 0) { return die("Initialize fail: ", err); }
		RCLCPP_INFO(this->get_logger(), "Opening device...");
		libusb_device_handle *dh = libusb_open_device_with_vid_pid(NULL, 0x2ca3, 0x1f);
		if (!dh) { return die("Opening device failed: ", 1); }

		RCLCPP_INFO(this->get_logger(), "Claiming interface...");
		if ((err = libusb_claim_interface(dh, 3)) < 0) { return die("claim interface fail ", err); }

		RCLCPP_INFO(this->get_logger(), "Sending magic packet...");
		int tx = 0;
		uint8_t data[] = {0x52, 0x4d, 0x56, 0x54};
		if ((err = libusb_bulk_transfer(dh, 0x03, data, 4, &tx, 500)) < 0) {
			RCLCPP_ERROR("ERROR: No data transmitted to device: %d", err); //don't exit
		}

		ctx myctx = {now(), 0};
		struct libusb_transfer *xfr = libusb_alloc_transfer(0);
		vector<uint8_t> buf(512);
		libusb_fill_bulk_transfer(xfr, dh, 0x84, buf.data(), buf.size(), [](struct libusb_transfer *xfer){
			fwrite(xfer->buffer, sizeof(uint8_t), xfer->actual_length, stdout); // todo change to pipe
			ctx* c = (ctx*) xfer->user_data;
			if ((now() - c->last) > (c->bytes? 0.1 : 2)) {
				RCLCPP_INFO("rx %d kb/s", (c->bytes / (now() - c->last))/1000);
				c->last = now();
				c->bytes = 0;
			}
			c->bytes += xfer->actual_length;
			int err;
			if ((err = libusb_submit_transfer(xfer)) < 0) { exit(die("Reading init failed: ", err)); }
		}, &myctx, 100);
		if ((err = libusb_submit_transfer(xfr)) < 0) { return die("read init fail ", err); }

		while(1) {
			if (libusb_handle_events(NULL) != LIBUSB_SUCCESS) break;
		}
	}

	~DJINode()
	{
		RCLCPP_INFO(this->get_logger(), "DJI Node is being destroyed.");
		libusb_exit(NULL);
	}
};

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ELRSNode>());
	rclcpp::shutdown();
	return 0;
}
