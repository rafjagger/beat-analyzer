// Which config files the analyzer reads, in which order. The package runs it
// as a user unit with no checkout around it, so the user's file lives in
// ~/.config/beat-analyzer; conf.d/ lets another package (a3-core) own a whole
// file of its own instead of splicing a block into the user's.
#include <cassert>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "../include/config/config_files.h"

using namespace BeatAnalyzer::Config;

namespace {

struct FakeDisk {
    std::set<std::string> files;
    std::vector<std::string> confd;  // names in conf.d/, in readdir order

    ConfigDisk disk() const
    {
        return {
            [this](const std::string& path) { return files.count(path) > 0; },
            [this](const std::string&) { return confd; },
        };
    }
};

const std::string kDir = "/home/u/.config/beat-analyzer";

}

static void test_config_dir_follows_xdg_then_home()
{
    assert(configDir("/x", "/home/u") == "/x/beat-analyzer");
    assert(configDir(nullptr, "/home/u") == kDir);
    assert(configDir("", "/home/u") == kDir);
    // XDG says a relative XDG_CONFIG_HOME is invalid and is ignored.
    assert(configDir("rel", "/home/u") == kDir);
    assert(configDir(nullptr, nullptr).empty());
    std::cout << "  ✓ the config dir is $XDG_CONFIG_HOME, else ~/.config" << std::endl;
}

static void test_a_checkouts_dot_env_wins_over_the_users_file()
{
    // A checkout's build/ folder runs as before even once the package has
    // seeded ~/.config/beat-analyzer/beat-analyzer.env.
    FakeDisk fake;
    fake.files = {kDir + "/beat-analyzer.env", ".env"};
    auto files = configFiles(std::nullopt, kDir, fake.disk());
    assert(files.size() == 1 && files[0] == ".env");

    fake.files = {kDir + "/beat-analyzer.env", "../.env"};
    files = configFiles(std::nullopt, kDir, fake.disk());
    assert(files.size() == 1 && files[0] == "../.env");
    std::cout << "  ✓ ./.env and ../.env before ~/.config/beat-analyzer/beat-analyzer.env" << std::endl;
}

static void test_the_users_file_wins_over_the_example()
{
    FakeDisk fake;
    fake.files = {kDir + "/beat-analyzer.env", ".env.example"};
    const auto files = configFiles(std::nullopt, kDir, fake.disk());
    assert(files.size() == 1 && files[0] == kDir + "/beat-analyzer.env");
    std::cout << "  ✓ the user's file before ./.env.example" << std::endl;
}

static void test_a_checkout_still_runs_on_its_dot_env()
{
    FakeDisk fake;
    fake.files = {"../.env", ".env.example"};
    auto files = configFiles(std::nullopt, kDir, fake.disk());
    assert(files.size() == 1 && files[0] == "../.env");

    fake.files = {".env", "../.env"};
    files = configFiles(std::nullopt, kDir, fake.disk());
    assert(files.size() == 1 && files[0] == ".env");

    fake.files = {".env.example"};
    files = configFiles(std::nullopt, kDir, fake.disk());
    assert(files.size() == 1 && files[0] == ".env.example");
    std::cout << "  ✓ without the user's file: ./.env, ../.env, ./.env.example" << std::endl;
}

static void test_an_explicit_file_replaces_the_search()
{
    FakeDisk fake;
    fake.files = {kDir + "/beat-analyzer.env", "/etc/mine.env"};
    const auto files = configFiles(std::string("/etc/mine.env"), kDir, fake.disk());
    assert(files.size() == 1 && files[0] == "/etc/mine.env");
    std::cout << "  ✓ --config FILE replaces the search" << std::endl;
}

static void test_conf_d_follows_sorted_and_only_env_files()
{
    FakeDisk fake;
    fake.files = {kDir + "/beat-analyzer.env"};
    fake.confd = {"50-a3-osc.env", "README", "10-local.env", "20-x.env.dpkg-old", ".hidden.env"};
    const auto files = configFiles(std::nullopt, kDir, fake.disk());
    const std::vector<std::string> want {
        kDir + "/beat-analyzer.env",
        kDir + "/conf.d/10-local.env",
        kDir + "/conf.d/50-a3-osc.env",
    };
    assert(files == want);
    std::cout << "  ✓ conf.d/*.env after the base file, sorted, hidden ones left out" << std::endl;
}

static void test_conf_d_is_read_even_with_an_explicit_file()
{
    FakeDisk fake;
    fake.files = {"/etc/mine.env"};
    fake.confd = {"50-a3-osc.env"};
    const auto files = configFiles(std::string("/etc/mine.env"), kDir, fake.disk());
    assert(files.size() == 2 && files[1] == kDir + "/conf.d/50-a3-osc.env");
    std::cout << "  ✓ conf.d is read after an explicit file too" << std::endl;
}

static void test_nothing_at_all_is_no_file()
{
    FakeDisk fake;
    assert(configFiles(std::nullopt, kDir, fake.disk()).empty());
    assert(configFiles(std::nullopt, "", fake.disk()).empty());
    std::cout << "  ✓ no file anywhere: built-in defaults" << std::endl;
}

static void test_an_unreadable_conf_d_file_is_skipped()
{
    // conf.d belongs to other packages: one bad file there must not keep the
    // meters and the beat clock dark.
    const std::vector<std::string> files {kDir + "/beat-analyzer.env", kDir + "/conf.d/10-bad.env",
                                          kDir + "/conf.d/50-a3-osc.env"};
    std::vector<std::string> loaded, skipped;
    const bool ok = loadEach(files, std::nullopt,
        [](const std::string& f) { return f.find("bad") == std::string::npos; },
        [&](const std::string& f, bool done) { (done ? loaded : skipped).push_back(f); });
    assert(ok);
    assert(loaded.size() == 2 && skipped.size() == 1 && skipped[0] == kDir + "/conf.d/10-bad.env");
    std::cout << "  ✓ an unreadable conf.d file is reported and skipped" << std::endl;
}

static void test_an_unreadable_explicit_file_stops_the_start()
{
    const std::vector<std::string> files {"/etc/missing.env", kDir + "/conf.d/50-a3-osc.env"};
    std::vector<std::string> skipped;
    const bool ok = loadEach(files, std::string("/etc/missing.env"),
        [](const std::string& f) { return f != "/etc/missing.env"; },
        [&](const std::string& f, bool done) { if (!done) skipped.push_back(f); });
    assert(!ok);
    assert(skipped.size() == 1 && skipped[0] == "/etc/missing.env");
    std::cout << "  ✓ an unreadable --config FILE stops the start" << std::endl;
}

static void test_the_config_argument()
{
    const char* a[] = {"beat-analyzer", "--config", "/etc/a.env"};
    assert(configArgument(3, const_cast<char**>(a)).value() == "/etc/a.env");
    const char* b[] = {"beat-analyzer", "--config=/etc/b.env"};
    assert(configArgument(2, const_cast<char**>(b)).value() == "/etc/b.env");
    const char* c[] = {"beat-analyzer"};
    assert(!configArgument(1, const_cast<char**>(c)).has_value());
    const char* d[] = {"beat-analyzer", "--config"};
    bool threw = false;
    try { configArgument(2, const_cast<char**>(d)); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    const char* e[] = {"beat-analyzer", "--config", "--config=/etc/e.env"};
    threw = false;
    try { configArgument(3, const_cast<char**>(e)); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);
    std::cout << "  ✓ --config FILE and --config=FILE; a bare --config is refused" << std::endl;
}

int main()
{
    test_config_dir_follows_xdg_then_home();
    test_a_checkouts_dot_env_wins_over_the_users_file();
    test_the_users_file_wins_over_the_example();
    test_a_checkout_still_runs_on_its_dot_env();
    test_an_explicit_file_replaces_the_search();
    test_conf_d_follows_sorted_and_only_env_files();
    test_conf_d_is_read_even_with_an_explicit_file();
    test_nothing_at_all_is_no_file();
    test_an_unreadable_conf_d_file_is_skipped();
    test_an_unreadable_explicit_file_stops_the_start();
    test_the_config_argument();
    return 0;
}
