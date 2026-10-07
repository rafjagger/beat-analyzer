#include "audio/vu_ports.h"

#include <algorithm>
#include <array>

namespace BeatAnalyzer {
namespace Audio {

namespace {
// One name per REAPER VU output, 31 to 70 (see vu_ports.h).
const std::array<const char*, kVuMapChannels> kVuMapNames = {
    "analog1_L", "analog1_R", "analog2_L", "analog2_R",
    "analog3_L", "analog3_R", "analog4_L", "analog4_R",
    "free39",    "free40",
    "main_sub",  "main_top1", "main_top2", "main_top3", "main_top4",
    "main_top5", "main_top6", "main_top7", "main_top8", "main_top9",
    "booth_sub",  "booth_top1", "booth_top2", "booth_top3", "booth_top4",
    "booth_top5", "booth_top6", "booth_top7", "booth_top8", "booth_top9",
    "phones_L",  "phones_R",  "rec_L",     "rec_R",
    "aux_L",     "aux_R",
    "free67",    "free68",    "free69",    "free70",
};

// The stereo channel meters, REAPER outs 51-66 (see vu_ports.h).
const std::array<const char*, kVuStereoChannels> kVuStereoNames = {
    "in1_pre_L",  "in1_pre_R",  "in2_pre_L",  "in2_pre_R",
    "in3_pre_L",  "in3_pre_R",  "in4_pre_L",  "in4_pre_R",
    "in1_post_L", "in1_post_R", "in2_post_L", "in2_post_R",
    "in3_post_L", "in3_post_R", "in4_post_L", "in4_post_R",
};

int vuNumber(int index)
{
    if (index < kVuMapChannels)
        return index + 1;
    return kVuStereoFirstNumber + (index - kVuMapChannels);
}
} // namespace

std::string vuPortName(int index)
{
    if (index >= 0 && index < kVuMapChannels)
        return std::string("vu_") + kVuMapNames[static_cast<size_t>(index)];
    if (index >= kVuMapChannels && index < kVuInputs)
        return std::string("vu_") + kVuStereoNames[static_cast<size_t>(index - kVuMapChannels)];
    return "vu_" + std::to_string(index + 1);
}

std::string vuOscPath(const std::string& pattern, int index)
{
    std::string path = pattern;
    const auto placeholder = path.find("{n}");
    if (placeholder != std::string::npos)
        path.replace(placeholder, 3, std::to_string(vuNumber(index)));
    return path;
}

int vuChannelsSent(int numVu)
{
    return std::min(numVu, kVuInputs);
}

int clampVuChannels(int requested)
{
    return std::clamp(requested, 0, kMaxVuChannels);
}

} // namespace Audio
} // namespace BeatAnalyzer
