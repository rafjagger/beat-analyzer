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
//      <dir>/beat-analyzer.env, ./.env, ../.env, ./.env.example
//      (the last three: a checkout's build/ folder runs as before);
//   2. every <dir>/conf.d/*.env, sorted by name -- a file another package
//      owns whole, like a3-core's rendered a3-osc block.
std::vector<std::string> configFiles(const std::optional<std::string>& explicitFile,
                                     const std::string& dir, const ConfigDisk& disk);

// FILE from `--config FILE` or `--config=FILE`; std::invalid_argument for a
// --config with nothing after it.
std::optional<std::string> configArgument(int argc, char* argv[]);

}
}
