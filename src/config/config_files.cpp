#include "config/config_files.h"

#include <algorithm>
#include <cstring>
#include <filesystem>

namespace BeatAnalyzer {
namespace Config {

namespace {

const char* const kAppDir = "beat-analyzer";
const char* const kUserFile = "beat-analyzer.env";
const char* const kConfD = "conf.d";
const char* const kOptionName = "--config";

// The places a checkout's build/ folder has always been read from.
const std::vector<std::string> kCheckoutFiles {".env", "../.env", ".env.example"};

bool isEnvName(const std::string& name)
{
    const std::string suffix = ".env";
    if (name.empty() || name[0] == '.')
        return false;
    return name.size() > suffix.size()
        && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::optional<std::string> baseFile(const std::string& dir, const ConfigDisk& disk)
{
    if (!dir.empty()) {
        const std::string user = dir + "/" + kUserFile;
        if (disk.isFile(user))
            return user;
    }
    for (const auto& file : kCheckoutFiles) {
        if (disk.isFile(file))
            return file;
    }
    return std::nullopt;
}

std::vector<std::string> confDFiles(const std::string& dir, const ConfigDisk& disk)
{
    if (dir.empty())
        return {};
    const std::string confd = dir + "/" + kConfD;
    std::vector<std::string> names = disk.namesIn(confd);
    names.erase(std::remove_if(names.begin(), names.end(),
                               [](const std::string& name) { return !isEnvName(name); }),
                names.end());
    std::sort(names.begin(), names.end());
    std::vector<std::string> files;
    for (const auto& name : names)
        files.push_back(confd + "/" + name);
    return files;
}

}

ConfigDisk realDisk()
{
    namespace fs = std::filesystem;
    return {
        [](const std::string& path) {
            std::error_code error;
            return fs::is_regular_file(path, error);
        },
        [](const std::string& dir) {
            std::vector<std::string> names;
            std::error_code error;
            for (fs::directory_iterator it(dir, error), end; !error && it != end; it.increment(error)) {
                if (it->is_regular_file(error))
                    names.push_back(it->path().filename().string());
            }
            return names;
        },
    };
}

std::string configDir(const char* xdgConfigHome, const char* home)
{
    if (xdgConfigHome != nullptr && xdgConfigHome[0] == '/')
        return std::string(xdgConfigHome) + "/" + kAppDir;
    if (home != nullptr && home[0] != '\0')
        return std::string(home) + "/.config/" + kAppDir;
    return {};
}

std::vector<std::string> configFiles(const std::optional<std::string>& explicitFile,
                                     const std::string& dir, const ConfigDisk& disk)
{
    std::vector<std::string> files;
    const auto base = explicitFile ? explicitFile : baseFile(dir, disk);
    if (base)
        files.push_back(*base);
    for (const auto& file : confDFiles(dir, disk))
        files.push_back(file);
    return files;
}

std::optional<std::string> configArgument(int argc, char* argv[])
{
    const std::string withValue = std::string(kOptionName) + "=";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.rfind(withValue, 0) == 0)
            return arg.substr(withValue.size());
        if (arg == kOptionName) {
            if (i + 1 >= argc)
                throw std::invalid_argument("--config needs a file");
            return std::string(argv[i + 1]);
        }
    }
    return std::nullopt;
}

}
}
