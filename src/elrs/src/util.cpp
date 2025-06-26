#include "elrs/util.hpp"

// handle each incoming byte from the ELRS module
void ProcessCrsfByte(uint8_t byte) {
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
bool ParseCrsfPacket(uint8_t* buffer, size_t buffer_length) {
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
			{
				float pitch = ((int16_t)((payload[0] << 8) | payload[1])) / 10000.0f;
				float roll  = ((int16_t)((payload[2] << 8) | payload[3])) / 10000.0f;
				float yaw   = ((int16_t)((payload[4] << 8) | payload[5])) / 10000.0f;

        		RCLCPP_INFO(NODE_LOGGER, "Attitude: Roll=%.2f rad, Pitch=%.2f rad, Yaw=%.2f rad", roll, pitch, yaw);
				break;
			}
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

// verify crc of packet
bool CheckCrc(uint8_t* buffer, size_t length) {
	uint8_t crc = CrsfCrc8(&buffer[2], length - 3); // exclude addr, len, crc
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