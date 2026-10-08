// The keep-alive beat-analyzer announces itself with on the DJ network
// (beat-analyzer#4): every byte where the public protocol descriptions put it
// -- Deep Symmetry's "DJ Link Ecosystem Analysis" (Device Announcements) and
// prolink-connect's announce packet. StemDeck pins the same layout in
// ProLinkPackets.TheKeepAliveMatchesTheLayout.
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "../include/osc/pro_link_keep_alive.h"

using namespace BeatAnalyzer::OSC;

static const uint8_t kMac[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
static const uint8_t kIp[4] = { 192, 168, 8, 10 };

static void build(uint8_t (&buf)[kKeepAliveSize])
{
    uint32_t ip;
    std::memcpy(&ip, kIp, 4);
    const int len = buildKeepAlive(buf, "beat-analyzer", 7, kMac, ip);
    assert(len == kKeepAliveSize);
}

static void test_the_keep_alive_matches_the_layout()
{
    uint8_t k[kKeepAliveSize];
    std::memset(k, 0xee, sizeof(k));
    build(k);

    assert(kKeepAliveSize == 0x36);
    assert(std::memcmp(k, "Qspt1WmJOL", 10) == 0);
    assert(k[0x0a] == 0x06 && k[0x0b] == 0x00);
    assert(std::memcmp(k + 0x0c, "beat-analyzer", 13) == 0);
    for (int i = 0x0c + 13; i < 0x20; ++i)
        assert(k[i] == 0x00);                       // name padded to 20 bytes
    assert(k[0x20] == 0x01 && k[0x21] == 0x02);
    assert(k[0x22] == 0x00 && k[0x23] == 0x36);     // the packet's own length
    assert(k[0x24] == 7);                           // device number
    assert(k[0x25] == 0x01);                        // device type: player
    assert(std::memcmp(k + 0x26, kMac, 6) == 0);
    assert(std::memcmp(k + 0x2c, kIp, 4) == 0);
    assert(k[0x30] == 0x01 && k[0x31] == 0x00 && k[0x32] == 0x00 && k[0x33] == 0x00);
    assert(k[0x34] == 0x01);                        // device type again
    assert(k[0x35] == 0x00);
    std::cout << "  ✓ every byte where the layout puts it" << std::endl;
}

static void test_a_long_name_is_cut_at_twenty_bytes()
{
    uint8_t k[kKeepAliveSize];
    uint32_t ip = 0;
    buildKeepAlive(k, "a-name-much-longer-than-twenty-bytes", 7, kMac, ip);
    assert(std::memcmp(k + 0x0c, "a-name-much-longer-t", 20) == 0);
    assert(k[0x20] == 0x01);
    std::cout << "  ✓ a long name stops before 0x20" << std::endl;
}

static void test_a_received_keep_alive_names_its_device()
{
    uint8_t k[kKeepAliveSize];
    build(k);
    assert(keepAliveDeviceName(k) == "beat-analyzer");
    assert(keepAliveDeviceNumber(k) == 7);
    std::cout << "  ✓ the name is read where it is written" << std::endl;
}

int main()
{
    std::cout << "Pro DJ Link keep-alive" << std::endl;
    test_the_keep_alive_matches_the_layout();
    test_a_long_name_is_cut_at_twenty_bytes();
    test_a_received_keep_alive_names_its_device();
    return 0;
}
