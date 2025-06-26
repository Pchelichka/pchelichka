#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <fcntl.h>
#include <dirent.h>
#include <algorithm>
#include <string.h> // memcpy

#include "constants.hpp"

std::string FindSerialPortByVidPid(const std::string& vid, const std::string& pid);
int MapRcToCrsf(int rc_value);
uint8_t CrsfCrc8(const uint8_t* ptr, uint8_t len);
void CrsfPrepareChannelsPacket(uint8_t packet[], int rcChannels[]);
void ProcessCrsfByte(uint8_t byte);
bool ParseCrsfPacket(uint8_t* buffer, size_t length);
bool CheckCrc(uint8_t* buffer, size_t length);