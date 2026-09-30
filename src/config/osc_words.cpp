#include "config/osc_words.h"

#include <cstdlib>

namespace BeatAnalyzer {
namespace Config {

namespace {

void takeText(const std::map<std::string, std::string>& env, const char* key,
              std::string& into)
{
    const auto found = env.find(key);
    if (found != env.end() && !found->second.empty())
        into = found->second;
}

void takePort(const std::map<std::string, std::string>& env, const char* key,
              int& into)
{
    const auto found = env.find(key);
    if (found == env.end())
        return;
    char* end = nullptr;
    const long port = std::strtol(found->second.c_str(), &end, 10);
    if (end != found->second.c_str() && *end == '\0' && port > 0 && port < 65536)
        into = static_cast<int>(port);
}

} // namespace

std::vector<std::string> oscWordKeys()
{
    return {"OSC_ADDRESS_BEAT", "OSC_ADDRESS_TAP", "OSC_ADDRESS_CLOCKMODE",
            "OSC_ADDRESS_VU", "OSC_PORT_A3MOTION", "PIONEER_PORT_ANNOUNCE",
            "PIONEER_PORT_BEAT", "PIONEER_PORT_STATUS"};
}

OscWords oscWordsFrom(const std::map<std::string, std::string>& env)
{
    OscWords words;
    takeText(env, "OSC_ADDRESS_BEAT", words.beat);
    takeText(env, "OSC_ADDRESS_TAP", words.tap);
    takeText(env, "OSC_ADDRESS_CLOCKMODE", words.clockMode);
    takeText(env, "OSC_ADDRESS_VU", words.vuPattern);
    takePort(env, "OSC_PORT_A3MOTION", words.listenPort);
    takePort(env, "PIONEER_PORT_ANNOUNCE", words.pioneerAnnounce);
    takePort(env, "PIONEER_PORT_BEAT", words.pioneerBeat);
    takePort(env, "PIONEER_PORT_STATUS", words.pioneerStatus);
    return words;
}

} // namespace Config
} // namespace BeatAnalyzer
