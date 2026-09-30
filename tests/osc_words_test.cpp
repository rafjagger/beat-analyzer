// The analyzer's OSC words and ports come from the one truth, a3-core's
// a3-osc.json, which the a3-core package renders into the .env's a3-osc block
// (decided 2026-09-30). What is not there keeps today's value, so an .env from
// before the block still runs.
#include <cassert>
#include <iostream>
#include <map>
#include <string>

#include "../include/config/osc_words.h"
#include "../include/audio/vu_ports.h"

using namespace BeatAnalyzer::Config;

static void test_the_rendered_lines_are_taken()
{
    const std::map<std::string, std::string> env {
        {"OSC_ADDRESS_BEAT", "/t/beat"},
        {"OSC_ADDRESS_TAP", "/t/tap"},
        {"OSC_ADDRESS_CLOCKMODE", "/t/clockmode"},
        {"OSC_ADDRESS_VU", "/t/vu/{n}"},
        {"OSC_PORT_A3MOTION", "17775"},
        {"PIONEER_PORT_ANNOUNCE", "60000"},
        {"PIONEER_PORT_BEAT", "60001"},
        {"PIONEER_PORT_STATUS", "60002"},
    };
    const auto words = oscWordsFrom(env);
    assert(words.beat == "/t/beat");
    assert(words.tap == "/t/tap");
    assert(words.clockMode == "/t/clockmode");
    assert(words.vuPattern == "/t/vu/{n}");
    assert(words.listenPort == 17775);
    assert(words.pioneerAnnounce == 60000);
    assert(words.pioneerBeat == 60001);
    assert(words.pioneerStatus == 60002);
    std::cout << "  ✓ the rendered lines are taken" << std::endl;
}

static void test_an_env_without_the_block_keeps_todays_values()
{
    const auto words = oscWordsFrom({});
    assert(words.beat == "/beat");
    assert(words.tap == "/tap");
    assert(words.clockMode == "/clockmode");
    assert(words.vuPattern == "/vu/{n}");
    assert(words.listenPort == 7775);
    assert(words.pioneerAnnounce == 50000);
    assert(words.pioneerBeat == 50001);
    assert(words.pioneerStatus == 50002);
    std::cout << "  ✓ an .env without the block keeps today's values" << std::endl;
}

static void test_a_port_that_is_not_a_number_keeps_its_value()
{
    const auto words = oscWordsFrom({{"PIONEER_PORT_BEAT", "fifty"}});
    assert(words.pioneerBeat == 50001);
    std::cout << "  ✓ a port that is not a number keeps its value" << std::endl;
}

static void test_every_key_is_listed()
{
    const auto keys = oscWordKeys();
    assert(keys.size() == 8);
    std::cout << "  ✓ every key is listed" << std::endl;
}

static void test_a_target_names_its_port()
{
    std::string host;
    int port = 0;
    assert(parseHostPort("192.168.8.11:7772", host, port));
    assert(host == "192.168.8.11" && port == 7772);
    // No port is no target: the port was 9000 by default, a second truth
    // the block in the .env has no use for.
    assert(!parseHostPort("192.168.8.11", host, port));
    assert(!parseHostPort("192.168.8.11:seven", host, port));
    std::cout << "  ✓ a target names its port" << std::endl;
}

static void test_a_meter_goes_out_by_the_pattern()
{
    using BeatAnalyzer::Audio::vuOscPath;
    assert(vuOscPath("/t/vu/{n}", 0) == "/t/vu/1");
    assert(vuOscPath("/vu/{n}", 39) == "/vu/40");
    std::cout << "  ✓ a meter goes out by the pattern, counted from 1" << std::endl;
}

int main()
{
    std::cout << "OscWords" << std::endl;
    test_the_rendered_lines_are_taken();
    test_an_env_without_the_block_keeps_todays_values();
    test_a_port_that_is_not_a_number_keeps_its_value();
    test_every_key_is_listed();
    test_a_target_names_its_port();
    test_a_meter_goes_out_by_the_pattern();
    return 0;
}
