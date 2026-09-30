#include "audio/vu_ports.h"

#include <algorithm>
#include <array>

namespace BeatAnalyzer {
namespace Audio {

namespace {
// One name per REAPER VU output, 31 to 70 (see vu_ports.h).
const std::array<const char*, kVuMapChannels> kVuMapNames = {
    "in1_pre",   "in2_pre",   "in3_pre",   "in4_pre",
    "in1_post",  "in2_post",  "in3_post",  "in4_post",
    "free39",    "free40",
    "main_sub",  "main_top1", "main_top2", "main_top3", "main_top4",
    "main_top5", "main_top6", "main_top7", "main_top8", "main_top9",
    "booth_sub",  "booth_top1", "booth_top2", "booth_top3", "booth_top4",
    "booth_top5", "booth_top6", "booth_top7", "booth_top8", "booth_top9",
    "phones_L",  "phones_R",  "rec_L",     "rec_R",
    "aux_L",     "aux_R",
    "free67",    "free68",    "free69",    "free70",
};
} // namespace

std::string vuPortName(int index)
{
    if (index >= 0 && index < kVuMapChannels)
        return std::string("vu_") + kVuMapNames[static_cast<size_t>(index)];
    return "vu_" + std::to_string(index + 1);
}

int clampVuChannels(int requested)
{
    return std::clamp(requested, 0, kMaxVuChannels);
}

} // namespace Audio
} // namespace BeatAnalyzer
