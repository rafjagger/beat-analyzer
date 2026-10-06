// Forty VU channels go out as one bundle per block of ten, each within the
// queue's 512-byte slot, together carrying every channel once.
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

// One bundle per block of ten (inputs, Main, Booth, stereo): each arrives as
// one consistent picture of its block -- Main's sub and tops together.
static void test_forty_channels_go_out_one_bundle_per_block()
{
    auto const chunks = OscSender::vuBundleChunks(40);
    assert(chunks.size() == 4);
    for (int b = 0; b < 4; ++b)
        assert(chunks[static_cast<size_t>(b)] == std::make_pair(b * 10, 10));
    std::cout << "  ✓ 40 channels: four blocks of ten" << std::endl;
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

static void test_a_short_last_block_is_its_own_bundle()
{
    auto const chunks = OscSender::vuBundleChunks(12);
    assert(chunks.size() == 2);
    assert(chunks[0] == std::make_pair(0, 10) && chunks[1] == std::make_pair(10, 2));
    assert(OscSender::vuBundleChunks(0).empty());
    std::cout << "  ✓ 12 channels: ten, then two" << std::endl;
}

// The forty plus the 8 stem meters (issue a3-system#71): the stems are a
// fifth bundle of their own and fit like the others.
static void test_a_bundle_of_forty_eight_fits()
{
    std::vector<std::string> paths;
    std::vector<float> peaks(48, 0.5f), rms(48, 0.25f);
    for (int i = 0; i < 48; ++i) paths.push_back("/vu/" + std::to_string(i + 1));

    auto const chunks = OscSender::vuBundleChunks(48);
    assert(chunks.size() == 5);
    assert(chunks[4] == std::make_pair(40, 8));
    int total = 0;
    for (auto const& [first, count] : chunks) {
        char buf[512];
        int len = OscSender::serializeBundle(buf, sizeof(buf), paths.data() + first,
                                             peaks.data() + first, rms.data() + first, count);
        assert(len > 0 && len <= 512);
        assert(elementsIn(buf, len) == count);
        total += count;
    }
    assert(total == 48);
    std::cout << "  ✓ 48 channels: the stems are a fifth bundle that fits" << std::endl;
}

// The stereo channel meters (/vu/51-66, spec stereo-channel-meters) go out
// as two bundles of eight: all four channels' pre-fader sides together, then
// all four post-fader -- a block each, like Main's sub and tops.
static void test_the_stereo_channel_meters_are_two_bundles_of_eight()
{
    auto const chunks = OscSender::vuBundleChunks(56);
    assert(chunks.size() == 6);
    assert(chunks[4] == std::make_pair(40, 8));
    assert(chunks[5] == std::make_pair(48, 8));

    std::vector<std::string> paths;
    std::vector<float> peaks(56, 0.5f), rms(56, 0.25f);
    for (int i = 0; i < 56; ++i) paths.push_back("/vu/" + std::to_string(i < 40 ? i + 1 : i + 11));
    int total = 0;
    for (auto const& [first, count] : chunks) {
        char buf[512];
        int len = OscSender::serializeBundle(buf, sizeof(buf), paths.data() + first,
                                             peaks.data() + first, rms.data() + first, count);
        assert(len > 0 && len <= 512);
        assert(elementsIn(buf, len) == count);
        total += count;
    }
    assert(total == 56);
    std::cout << "  ✓ 56 channels: the stereo meters are two bundles of eight" << std::endl;
}

int main()
{
    std::cout << "VU bundle tests" << std::endl;
    test_a_bundle_of_forty_eight_fits();
    test_the_stereo_channel_meters_are_two_bundles_of_eight();
    test_forty_channels_go_out_one_bundle_per_block();
    test_every_bundle_fits_and_carries_its_channels();
    test_a_short_last_block_is_its_own_bundle();
    return 0;
}
