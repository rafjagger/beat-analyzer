#pragma once

#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace BeatAnalyzer {
namespace Config {

// What configFiles() needs to know about the disk, so tests can fake it.
struct ConfigDisk {
    std::function<bool(const std::string& path)> isFile;
    std::function<std::vector<std::string>(const std::string& dir)> namesIn;
};

// ConfigDisk on the real file system.
ConfigDisk realDisk();

// $XDG_CONFIG_HOME/beat-analyzer, else $HOME/.config/beat-analyzer; empty
// when neither is set. A relative XDG_CONFIG_HOME is ignored, as XDG says.
std::string configDir(const char* xdgConfigHome, const char* home);

// The files to load, in order; a later file wins key by key.
//   1. `explicitFile` if given, else the first that exists of
//      ./.env, ../.env, <dir>/beat-analyzer.env, ./.env.example
//      (a checkout's build/ folder runs on its own .env as before, even once
//      the package has seeded the user's file);
//   2. every <dir>/conf.d/*.env, sorted by name -- a file another package
//      owns whole, like a3-core's rendered targets. Read for a checkout run
//      too: a3-core writes its targets only there.
std::vector<std::string> configFiles(const std::optional<std::string>& explicitFile,
                                     const std::string& dir, const ConfigDisk& disk);

// Loads `files` in order and reports each. An unreadable `explicitFile`
// (--config FILE) stops the start; any other unreadable file is skipped, so
// one bad file in conf.d never keeps the meters and the clock dark.
bool loadEach(const std::vector<std::string>& files, const std::optional<std::string>& explicitFile,
              const std::function<bool(const std::string&)>& load,
              const std::function<void(const std::string&, bool)>& report);

// FILE from `--config FILE` or `--config=FILE`; std::invalid_argument for a
// --config with nothing after it, or with another option after it.
std::optional<std::string> configArgument(int argc, char* argv[]);

}
}
