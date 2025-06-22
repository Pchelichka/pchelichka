#include "elrs/util.hpp"


void ProcessCrsfByte(uint8_t byte) {
	static uint8_t crsf_frame_buffer[CRSF_FRAME_SIZE_MAX * 2];
	static uint8_t crsf_frame_buffer_pos  = 0;
	static uint8_t crsfPayloadLength = 0;
	static uint8_t crsfType = 0;
	// record incoming byte in buffer
	crsf_frame_buffer[crsf_frame_buffer_pos++] = byte;
	if (crsf_frame_buffer_pos > 3) {
		uint8_t expected_frame_len = crsf_frame_buffer[CRSF_FRAME_LENGTH_ADDRESS];
		uint8_t expected_frame_type = crsf_frame_buffer[CRSF_FRAME_TYPE_ADDRESS];
		// printf("Type: 0x%02X\n", expected_frame_type);
		int full_frame_length = expected_frame_len + 2;
		if (expected_frame_type == CRSF_PACKET_TYPE_RADIO_ID) {
			full_frame_length = expected_frame_len + 5;
		}
		if (full_frame_length > CRSF_FRAME_SIZE_MAX) {
			printf("Frame error: size if too large = %d\n", full_frame_length);
			return;
		} else if (crsf_frame_buffer_pos >= full_frame_length) {
			ParseCrsfPacket(crsf_frame_buffer, crsf_frame_buffer_pos);
			crsf_frame_buffer_pos -= full_frame_length; // subtract frame size from write pointer
			memcpy(crsf_frame_buffer, &crsf_frame_buffer[full_frame_length], crsf_frame_buffer_pos); // put the remaining bytes into beginning	
		}
	}
}

void ParseCrsfPacket(uint8_t* buffer, size_t buffer_length) {
	const uint8_t length = buffer[1];
	const uint8_t type = buffer[2];
	const uint8_t* payload = &buffer[3];
	const uint8_t expected_crc = buffer[length + 1];
	if (CrsfCrc8(&buffer[2], length - 1)) {
		switch (type) {
			case CRSF_PACKET_TYPE_LINK_STATISTICS:
				RCLCPP_INFO(rclcpp::get_logger("elrs_node"), "Telemetry: RSSI=%d LQ=%d\n", payload[0], payload[2]);
				break;

			case CRSF_PACKET_TYPE_BATTERY_SENSOR:
			{
				uint16_t voltage_mv = payload[0] | (payload[1] << 8);
				uint16_t current_ma = payload[2] | (payload[3] << 8);
				
				RCLCPP_INFO(rclcpp::get_logger("elrs_node"), "Battery: %u mV, %u mA", voltage_mv, current_ma);
				break;
			}
			case CRSF_PACKET_TYPE_ATTITUDE:
				RCLCPP_INFO(rclcpp::get_logger("elrs_node"), "%d", length);
				break;
			default:
				RCLCPP_INFO(rclcpp::get_logger("elrs_node"), "Received packet type 0x%02X", type);
				break;
		}
	} else {
		printf("CRC error\n");
	}
}

bool CheckCrc(uint8_t* buffer, size_t length) {
	uint8_t crc = CrsfCrc8(&buffer[2], length - 3); // exclude addr, len, crc
	printf("Calculated: 0x%02X -- Actual 0x%02X\n", crc, buffer[length - 1]);
	return crc == buffer[length - 1];
}

// Map RC range (RC_MIN-RC_MAX) to the values accepted by CRSF(CRSF_MIN-CRSF_MAX)
int MapRcToCrsf(int rc_value) {
	// Clamp input to expected RC range
	rc_value = std::clamp(rc_value, RC_MIN, RC_MAX);

	// Scale linearly from RC to CRSF
	float scale = (float)(CRSF_MAX - CRSF_MIN) / (RC_MAX - RC_MIN);
	return static_cast<int>((rc_value - RC_MIN) * scale + CRSF_MIN);
}

// Calculate the last(verification) byte of the CRSF packet
uint8_t CrsfCrc8(const uint8_t *ptr, uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i=0; i < len; i++) {
		crc = crsf_crc8tab[crc ^ *ptr++];
	}
	return crc;
}

// Convert channel values into a valid CRSF packet
void CrsfPrepareChannelsPacket(uint8_t packet[], int rcChannels[]){
	static int channels[CRSF_MAX_CHANNEL];
	/*
		* Map 1000-2000 with middle at 1500 chanel values to
		* 172-1811(CRSF_CHANNEL_MIN-CRSF_CHANNEL_MAX) with middle at 991(CRSF_CHANNEL_MID) S.BUS(andCRSF) protocol requires
		*/
	for (uint8_t i = 0; i < CRSF_MAX_CHANNEL; i++) {
		channels[i] = MapRcToCrsf(rcChannels[i]);
	}   

	packet[0] = CRSF_ADDR_MODULE; //Header
	packet[1] = 24;   // length of type (24) + payload + crc
	packet[2] = CRSF_PACKET_TYPE_RC_CHANNELS_PACKED;
	
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
	packet[25] = CrsfCrc8(&packet[2], CRSF_PACKET_SIZE-3); //CRC
}