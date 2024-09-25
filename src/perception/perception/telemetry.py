from telemetry_interfaces.srv import GetPitch, GetRoll, GetYaw, GetBatteryVoltage 
import serial
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import argparse
from enum import IntEnum

CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def crc8_dvb_s2(crc, a) -> int:
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b):
    return b - 256 if b >= 128 else b

def packCrsfToBytes(channels) -> bytes:
    # channels is in CRSF format! (0-1984)
    # Values are packed little-endianish such that bits BA987654321 -> 87654321, 00000BA9
    # 11 bits per channel x 16 channels = 22 bytes
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    result = bytearray()
    destShift = 0
    newVal = 0
    for ch in channels:
        # Put the low bits in any remaining dest capacity
        newVal |= (ch << destShift) & 0xff
        result.append(newVal)

        # Shift the high bits down and place them into the next dest byte
        srcBitsLeft = 11 - 8 + destShift
        newVal = ch >> (11 - srcBitsLeft)
        # When there's at least a full byte remaining, consume that as well
        if srcBitsLeft >= 8:
            result.append(newVal & 0xff)
            newVal >>= 8
            srcBitsLeft -= 8

        # Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft

    return result

def channelsCrsfToChannelsPacket(channels) -> bytes:
    result = bytearray([CRSF_SYNC, 24, PacketsTypes.RC_CHANNELS_PACKED]) # 24 is packet length
    result += packCrsfToBytes(channels)
    result.append(crc8_data(result[2:]))
    return result

class TelemetryService(Node): 
    def __init__(self):
        super().__init__('minimal_service')
        parser = argparse.ArgumentParser()
        parser.add_argument('-P', '--port', default='/dev/ttyACM1', required=False)
        parser.add_argument('-b', '--baud', default=921600, required=False)
        parser.add_argument('-t', '--tx', required=False, default=False, action='store_true',
                            help='Enable sending CHANNELS_PACKED every 20ms (all channels 1500us)')
        self.args, _ = parser.parse_known_args()
        self.input = bytearray()
        self.serial = serial.Serial(self.args.port, self.args.baud, timeout=2)
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.battery_voltage = 0
        self.callback_group = ReentrantCallbackGroup()
        self.pitch_srv = self.create_service(GetPitch, 'get_pitch', self.get_pitch, callback_group=self.callback_group)
        self.roll_srv = self.create_service(GetRoll, 'get_roll', self.get_roll, callback_group=self.callback_group)
        self.yaw_srv = self.create_service(GetYaw, 'get_yaw', self.get_yaw, callback_group=self.callback_group)
        self.battery_srv = self.create_service(GetBatteryVoltage, 'get_battery_voltage', self.get_battery_voltage, callback_group=self.callback_group)
        self.timer_action = self.create_timer(0.0001, self.timer_callback)

    def get_pitch(self, request, response):
        response.success = True
        response.radians = self.pitch
        response.degrees = math.degrees(self.pitch)
        return response

    def get_roll(self, request, response):
        response.success = True
        response.radians = self.roll
        response.degrees = math.degrees(self.roll)
        return response

    def get_yaw(self, request, response):
        response.success = True
        response.radians = self.yaw
        response.degrees = math.degrees(self.yaw)
        return response

    def get_battery_voltage(self, request, response):
        response.success = True
        response.voltage = self.battery_voltage
        return response
    
    def timer_callback(self):
        if self.serial.in_waiting > 0:
            self.input.extend(self.serial.read(self.serial.in_waiting))
        else:
            if self.args.tx:
                self.serial.write(channelsCrsfToChannelsPacket([992 for ch in range(16)]))
            time.sleep(0.020)

        while len(self.input) > 2:
            # This simple parser works with malformed CRSF streams
            # it does not check the first byte for SYNC_BYTE, but
            # instead just looks for anything where the packet length
            # is 4-64 bytes, and the CRC validates
            expected_len = self.input[1] + 2
            if expected_len > 64 or expected_len < 4:
                self.input = bytearray()
            elif len(self.input) >= expected_len:
                single = self.input[:expected_len] # copy out this whole packet
                self.input = self.input[expected_len:] # and remove it from the buffer

                if not crsf_validate_frame(single): # single[-1] != crc:
                    packet = ' '.join(map(hex, single))
                    print(f"crc error: {packet}")
                else:
                    self.handleCrsfPacket(single[2], single)
            else:
                break
    def handleCrsfPacket(self, ptype, data):
        if ptype == PacketsTypes.RADIO_ID and data[5] == 0x10:
            #print(f"OTX sync")
            pass
        elif ptype == PacketsTypes.LINK_STATISTICS:
            rssi1 = signed_byte(data[3])
            rssi2 = signed_byte(data[4])
            lq = data[5]
            snr = signed_byte(data[6])
            antenna = data[7]
            mode = data[8]
            power = data[9]
            # telemetry strength
            downlink_rssi = signed_byte(data[10])
            downlink_lq = data[11]
            downlink_snr = signed_byte(data[12])
            print(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03} mode={mode}") # ant={antenna} snr={snr} power={power} drssi={downlink_rssi} dlq={downlink_lq} dsnr={downlink_snr}")
        elif ptype == PacketsTypes.ATTITUDE:
            self.pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
            self.roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
            self.yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
            print(f"Attitude: Pitch={self.pitch:0.2f} Roll={self.roll:0.2f} Yaw={self.yaw:0.2f} (rad)")
        elif ptype == PacketsTypes.FLIGHT_MODE:
            packet = ''.join(map(chr, data[3:-2]))
            print(f"Flight Mode: {packet}")
        elif ptype == PacketsTypes.BATTERY_SENSOR:
            self.battery_voltage = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
            curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
            mah = data[7] << 16 | data[8] << 7 | data[9]
            pct = data[10]
            print(f"Battery: {self.battery_voltage:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
        elif ptype == PacketsTypes.BARO_ALT:
            print(f"BaroAlt: ")
        elif ptype == PacketsTypes.DEVICE_INFO:
            packet = ' '.join(map(hex, data))
            print(f"Device Info: {packet}")
        elif data[2] == PacketsTypes.GPS:
            lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
            lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
            gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
            hdg =  int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
            alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
            sats = data[17]
            print(f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}")
        elif ptype == PacketsTypes.VARIO:
            vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
            print(f"VSpd: {vspd:0.1f}m/s")
        elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
            #print(f"Channels: (data)")
            pass
        else:
            packet = ' '.join(map(hex, data))
            print(f"Unknown 0x{ptype:02x}: {packet}")

def main(args=None):
    rclpy.init(args=args)

    telemtry_service = TelemetryService()


    executor = MultiThreadedExecutor()
    executor.add_node(telemtry_service)

    executor.spin()
            
    rclpy.shutdown()


if __name__ == '__main__':
    main()