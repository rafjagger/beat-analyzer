#pragma once

#include <cstdint>
#include <string>

namespace BeatAnalyzer {
namespace OSC {

// The keep-alive a player sends on port 50000 every 1.5 s to stay on the
// DJ network, laid out as the public protocol descriptions give it: Deep
// Symmetry's "DJ Link Ecosystem Analysis" (Device Announcements,
// https://djl-analysis.deepsymmetry.org/) and prolink-connect's announce
// packet. Written from those descriptions, not from either project's code.
constexpr int kKeepAliveSize = 0x36;

// Writes the keep-alive into buf (kKeepAliveSize bytes) and returns its
// length. The name is cut at 20 bytes; ipNetworkOrder as in sockaddr_in.
int buildKeepAlive(uint8_t* buf, const char* name, uint8_t deviceNumber,
                   const uint8_t (&mac)[6], uint32_t ipNetworkOrder);

// What a received keep-alive (at least kKeepAliveSize bytes) says about its sender.
std::string keepAliveDeviceName(const uint8_t* data);
uint8_t keepAliveDeviceNumber(const uint8_t* data);

} // namespace OSC
} // namespace BeatAnalyzer
