#include "osc/pro_link_keep_alive.h"

#include <cstring>

namespace BeatAnalyzer {
namespace OSC {

namespace {
constexpr uint8_t kMagic[10] = { 'Q', 's', 'p', 't', '1', 'W', 'm', 'J', 'O', 'L' };
constexpr uint8_t kTypeKeepAlive = 0x06;
constexpr uint8_t kDeviceTypePlayer = 0x01;
constexpr int kNameSize = 20;

constexpr int kOffType = 0x0a;
constexpr int kOffName = 0x0c;
constexpr int kOffSubtype = 0x20;     // 01 02
constexpr int kOffLength = 0x22;      // 2 bytes BE: the packet's own length
constexpr int kOffDeviceNumber = 0x24;
constexpr int kOffDeviceType = 0x25;
constexpr int kOffMac = 0x26;
constexpr int kOffIp = 0x2c;
constexpr int kOffPeers = 0x30;       // 01 00 00 00
constexpr int kOffDeviceTypeAgain = 0x34;
} // namespace

int buildKeepAlive(uint8_t* buf, const char* name, uint8_t deviceNumber,
                   const uint8_t (&mac)[6], uint32_t ipNetworkOrder)
{
    std::memset(buf, 0, kKeepAliveSize);
    std::memcpy(buf, kMagic, sizeof(kMagic));
    buf[kOffType] = kTypeKeepAlive;
    std::strncpy(reinterpret_cast<char*>(buf + kOffName), name, kNameSize);
    buf[kOffSubtype] = 0x01;
    buf[kOffSubtype + 1] = 0x02;
    buf[kOffLength] = 0x00;
    buf[kOffLength + 1] = kKeepAliveSize;
    buf[kOffDeviceNumber] = deviceNumber;
    buf[kOffDeviceType] = kDeviceTypePlayer;
    std::memcpy(buf + kOffMac, mac, sizeof(mac));
    std::memcpy(buf + kOffIp, &ipNetworkOrder, 4);
    buf[kOffPeers] = 0x01;
    buf[kOffDeviceTypeAgain] = kDeviceTypePlayer;
    return kKeepAliveSize;
}

std::string keepAliveDeviceName(const uint8_t* data)
{
    const char* name = reinterpret_cast<const char*>(data + kOffName);
    return std::string(name, strnlen(name, kNameSize));
}

uint8_t keepAliveDeviceNumber(const uint8_t* data)
{
    return data[kOffDeviceNumber];
}

} // namespace OSC
} // namespace BeatAnalyzer
