#pragma once

#include <map>
#include <string>
#include <vector>

namespace BeatAnalyzer {
namespace Config {

// The analyzer's OSC words and ports. They belong to the one truth, a3-core's
// a3-osc.json, and reach the analyzer through the .env: the a3-core package
// renders them into its a3-osc block (a3-osc-render, decided 2026-09-30).
// A line that is not there keeps the value below, so an .env from before the
// block still runs -- these are what the truth said on the day they were
// written, not a second opinion.
struct OscWords {
    std::string beat = "/beat";
    std::string tap = "/tap";
    std::string clockMode = "/clockmode";
    std::string vuPattern = "/vu/{n}";   // {n}: the meter, counted from 1
    int listenPort = 7775;               // /beat, /tap, /clockmode come in here
    int pioneerAnnounce = 50000;
    int pioneerBeat = 50001;
    int pioneerStatus = 50002;
};

// The .env keys OscWords reads.
std::vector<std::string> oscWordKeys();

// OscWords from the .env's values for oscWordKeys(); absent or unreadable
// ones keep their value.
OscWords oscWordsFrom(const std::map<std::string, std::string>& env);

// "host:port" as the .env writes a target. False without a port: there is
// no default to fall back on.
bool parseHostPort(const std::string& value, std::string& host, int& port);

} // namespace Config
} // namespace BeatAnalyzer
