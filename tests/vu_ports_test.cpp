// The VU inputs are named after what REAPER sends them (its outputs 31-70
// and 51-66, see the A3 Core manual's channel map), so a patchbay reads right and a
// wrong cable shows at a glance.
#include <cassert>
#include <iostream>
#include <set>
#include <string>

#include "../include/audio/vu_ports.h"

using namespace BeatAnalyzer::Audio;

static void test_the_map_names_forty_inputs()
{
    assert(kVuMapChannels == 40);
    std::set<std::string> names;
    for (int i = 0; i < kVuMapChannels; ++i)
        names.insert(vuPortName(i));
    assert(names.size() == 40);
    std::cout << "  ✓ forty distinct names" << std::endl;
}

static void test_each_block_by_its_reaper_output()
{
    // 31-34 inputs pre-fader, 35-38 post-fader, 39-40 free
    assert(vuPortName(0) == "vu_in1_pre");
    assert(vuPortName(3) == "vu_in4_pre");
    assert(vuPortName(4) == "vu_in1_post");
    assert(vuPortName(7) == "vu_in4_post");
    assert(vuPortName(8) == "vu_free39");
    assert(vuPortName(9) == "vu_free40");
    // 41-50 Main, 51-60 Booth: sub, then nine tops
    assert(vuPortName(10) == "vu_main_sub");
    assert(vuPortName(11) == "vu_main_top1");
    assert(vuPortName(19) == "vu_main_top9");
    assert(vuPortName(20) == "vu_booth_sub");
    assert(vuPortName(29) == "vu_booth_top9");
    // 61-70 stereo: phones, rec, aux, free
    assert(vuPortName(30) == "vu_phones_L");
    assert(vuPortName(31) == "vu_phones_R");
    assert(vuPortName(32) == "vu_rec_L");
    assert(vuPortName(35) == "vu_aux_R");
    assert(vuPortName(36) == "vu_free67");
    assert(vuPortName(39) == "vu_free70");
    std::cout << "  ✓ names follow REAPER's VU outputs" << std::endl;
}

// The stereo channel meters (spec stereo-channel-meters, 2026-10-06) follow
// the forty as inputs 40-55: REAPER outs 51-66, sent as /vu/51-66.
static void test_the_stereo_channel_meters_follow_the_forty()
{
    assert(kVuInputs == 56);
    assert(vuPortName(40) == "vu_in1_pre_L");
    assert(vuPortName(41) == "vu_in1_pre_R");
    assert(vuPortName(46) == "vu_in4_pre_L");
    assert(vuPortName(47) == "vu_in4_pre_R");
    assert(vuPortName(48) == "vu_in1_post_L");
    assert(vuPortName(55) == "vu_in4_post_R");
    std::set<std::string> names;
    for (int i = 0; i < kVuInputs; ++i)
        names.insert(vuPortName(i));
    assert(names.size() == 56);
    std::cout << "  ✓ inputs 40-55: vu_in1_pre_L .. vu_in4_post_R" << std::endl;
}

static void test_the_stereo_channel_meters_are_sent_as_51_to_66()
{
    assert(vuOscPath("/vu/{n}", 40) == "/vu/51");  // vu_in1_pre_L
    assert(vuOscPath("/vu/{n}", 47) == "/vu/58");  // vu_in4_pre_R
    assert(vuOscPath("/vu/{n}", 48) == "/vu/59");  // vu_in1_post_L
    assert(vuOscPath("/vu/{n}", 55) == "/vu/66");  // vu_in4_post_R
    std::cout << "  ✓ /vu/51 .. /vu/66" << std::endl;
}

static void test_beyond_the_map_ports_are_numbered()
{
    assert(vuPortName(kVuInputs) == "vu_57");
    std::cout << "  ✓ beyond the map: vu_N" << std::endl;
}

// The meters are held in arrays of kMaxVuChannels; a count from the .env
// above that wrote past their end.
static void test_the_count_is_held_to_what_fits()
{
    assert(kMaxVuChannels >= kVuMapChannels);
    assert(clampVuChannels(40) == 40);
    assert(clampVuChannels(1000) == kMaxVuChannels);
    assert(clampVuChannels(-3) == 0);
    std::cout << "  ✓ the channel count is clamped" << std::endl;
}

// The OSC counts from 1, like the channel map (2026-09-30): /vu/N is VU
// channel N, fed from REAPER out 30 + N.
static void test_the_osc_address_counts_from_one()
{
    assert(vuOscPath("/vu/{n}", 0) == "/vu/1");    // vu_in1_pre, REAPER out 31
    assert(vuOscPath("/vu/{n}", 10) == "/vu/11");  // vu_main_sub, REAPER out 41
    assert(vuOscPath("/vu/{n}", 39) == "/vu/40");  // vu_free70, REAPER out 70
    std::cout << "  ✓ /vu/1 .. /vu/40" << std::endl;
}

// /vu/41-48 are StemDeck's own stem meters since spec stemdeck-remote
// (2026-10-01), 49-50 its AUX bus: whatever NUM_VU_CHANNELS says, the
// analyzer's inputs skip them, so it never speaks on StemDeck's addresses.
static void test_the_vu_inputs_never_reach_stemdecks_addresses()
{
    assert(vuChannelsSent(64) == kVuInputs);
    assert(vuChannelsSent(56) == 56);
    assert(vuChannelsSent(40) == 40);
    for (int i = 0; i < vuChannelsSent(kMaxVuChannels); ++i) {
        const auto path = vuOscPath("/vu/{n}", i);
        for (int n = 41; n <= 50; ++n)
            assert(path != "/vu/" + std::to_string(n));
    }
    assert(vuChannelsSent(12) == 12);
    assert(vuChannelsSent(0) == 0);
    std::cout << "  ✓ never /vu/41-50: those are StemDeck's" << std::endl;
}

int main()
{
    std::cout << "VU port tests" << std::endl;
    test_the_vu_inputs_never_reach_stemdecks_addresses();
    test_the_map_names_forty_inputs();
    test_each_block_by_its_reaper_output();
    test_the_stereo_channel_meters_follow_the_forty();
    test_the_stereo_channel_meters_are_sent_as_51_to_66();
    test_beyond_the_map_ports_are_numbered();
    test_the_count_is_held_to_what_fits();
    test_the_osc_address_counts_from_one();
    return 0;
}
