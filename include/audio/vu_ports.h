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
// /vu/i -- the OSC stays positional, the name is for whoever patches.
constexpr int kVuMapChannels = 40;

// The map's blocks are ten wide; the OSC sends one bundle per block.
constexpr int kVuMapBlock = 10;

// What the meter arrays hold at most.
constexpr int kMaxVuChannels = 64;

std::string vuPortName(int index);

// A channel count from the .env, held to what the arrays hold.
int clampVuChannels(int requested);

} // namespace Audio
} // namespace BeatAnalyzer
