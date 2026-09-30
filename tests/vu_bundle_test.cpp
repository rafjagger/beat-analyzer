// Forty VU channels do not fit one 512-byte bundle; they go out as several,
// each within the queue's slot, together carrying every channel once.
#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "../include/osc/osc_sender.h"

using BeatAnalyzer::OSC::OscSender;

static int readInt32(const char* p)
{
    return (static_cast<unsigned char>(p[0]) << 24) | (static_cast<unsigned char>(p[1]) << 16)
         | (static_cast<unsigned char>(p[2]) << 8) | static_cast<unsigned char>(p[3]);
}

// How many messages a serialised bundle holds.
static int elementsIn(const char* buf, int len)
{
    int pos = 16, n = 0;
    while (pos + 4 <= len) {
        pos += 4 + readInt32(buf + pos);
        ++n;
    }
    return n;
}

static void test_forty_channels_go_out_in_three_bundles()
{
    auto const chunks = OscSender::vuBundleChunks(40);
    assert(chunks.size() == 3);
    assert(chunks[0] == std::make_pair(0, 16));
    assert(chunks[1] == std::make_pair(16, 16));
    assert(chunks[2] == std::make_pair(32, 8));
    std::cout << "  ✓ 40 channels: 16 + 16 + 8" << std::endl;
}

static void test_every_bundle_fits_and_carries_its_channels()
{
    std::vector<std::string> paths;
    std::vector<float> peaks(40, 0.5f), rms(40, 0.25f);
    for (int i = 0; i < 40; ++i) paths.push_back("/vu/" + std::to_string(i));

    int total = 0;
    for (auto const& [first, count] : OscSender::vuBundleChunks(40)) {
        char buf[512];
        int len = OscSender::serializeBundle(buf, sizeof(buf), paths.data() + first,
                                             peaks.data() + first, rms.data() + first, count);
        assert(len > 0 && len <= 512);
        assert(elementsIn(buf, len) == count);
        total += count;
    }
    assert(total == 40);
    std::cout << "  ✓ each bundle fits 512 bytes, 40 messages in all" << std::endl;
}

static void test_twelve_channels_stay_one_bundle()
{
    auto const chunks = OscSender::vuBundleChunks(12);
    assert(chunks.size() == 1 && chunks[0] == std::make_pair(0, 12));
    assert(OscSender::vuBundleChunks(0).empty());
    std::cout << "  ✓ 12 channels: one bundle, as before" << std::endl;
}

int main()
{
    std::cout << "VU bundle tests" << std::endl;
    test_forty_channels_go_out_in_three_bundles();
    test_every_bundle_fits_and_carries_its_channels();
    test_twelve_channels_stay_one_bundle();
    return 0;
}
