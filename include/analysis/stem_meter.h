#pragma once

#include <algorithm>

namespace BeatAnalyzer {
namespace Analysis {

// A stem pair is metered as one: the louder of its two sides, peak and RMS
// each on their own (issue a3-system#71).
struct Level { float peak; float rms; };

inline Level louder(Level left, Level right)
{
    return {std::max(left.peak, right.peak), std::max(left.rms, right.rms)};
}

} // namespace Analysis
} // namespace BeatAnalyzer
