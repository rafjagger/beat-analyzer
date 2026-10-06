#pragma once

#include <string>

namespace BeatAnalyzer {
namespace Audio {

// The VU inputs as REAPER sends them: its outputs 31-70, one meter each, in
// blocks of ten (A3 Core manual, channel map, 2026-09-30):
//
//   31-40  inputs    31-34 ch 1-4 pre-fader (post-FX), 35-38 post-fader, 39-40 free
//   41-50  Main      41 sub, 42-50 tops 1-9
//   51-60  Booth     51 sub, 52-60 tops 1-9
//   61-70  stereo    61-62 phones, 63-64 rec, 65-66 aux, 67-70 free
//
// Input i (0-based) is vu_<name>, fed from REAPER out 31 + i, and its OSC is
// /vu/<i+1> -- counted from 1 like the map; the name is for whoever patches.
constexpr int kVuMapChannels = 40;

// The map's blocks are ten wide; the OSC sends one bundle per block.
constexpr int kVuMapBlock = 10;

// After the forty, the stereo channel meters (spec stereo-channel-meters,
// 2026-10-06), fed from REAPER outs 51-66:
//
//   inputs 40-47  in1_pre_L, in1_pre_R ... in4_pre_R    /vu/51-58
//   inputs 48-55  in1_post_L, in1_post_R ... in4_post_R  /vu/59-66
//
// /vu/41-50 lie between them and are StemDeck's (stems, AUX bus), so these
// inputs are sent as /vu/<i+11>. Two blocks of eight, a bundle each.
constexpr int kVuStereoChannels = 16;
constexpr int kVuStereoBlock = 8;
constexpr int kVuStereoFirstNumber = 51;

// Every VU input the analyzer has.
constexpr int kVuInputs = kVuMapChannels + kVuStereoChannels;

// What the meter arrays hold at most.
constexpr int kMaxVuChannels = 64;

std::string vuPortName(int index);

// Where input i (0-based) is sent: the truth's pattern ("/vu/{n}") with {n}
// = i+1 for the forty (REAPER out 30 + N), i+11 for the stereo channel
// meters (REAPER out N).
std::string vuOscPath(const std::string& pattern, int index);

// How many VU inputs are sent: at most kVuInputs. /vu/41-50 are StemDeck's
// own meters since spec stemdeck-remote (2026-10-01), and no input maps onto
// them, whatever NUM_VU_CHANNELS asks for.
int vuChannelsSent(int numVu);

// A channel count from the .env, held to what the arrays hold.
int clampVuChannels(int requested);

} // namespace Audio
} // namespace BeatAnalyzer
