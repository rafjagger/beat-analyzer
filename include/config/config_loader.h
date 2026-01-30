#pragma once

#include <string>
#include <memory>
#include <unordered_map>

namespace BeatAnalyzer {
namespace Config {

/**
 * Configuration loader for YAML config files.
 */
class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& configPath);
    ~ConfigLoader() = default;
    
    // Load configuration from file
    bool load();
    
    // Get values with default fallback
    std::string getString(const std::string& key, 
                         const std::string& defaultValue = "") const;
    int getInt(const std::string& key, int defaultValue = 0) const;
    float getFloat(const std::string& key, float defaultValue = 0.0f) const;
    bool getBool(const std::string& key, bool defaultValue = false) const;
    
    // Check if key exists
    bool hasKey(const std::string& key) const;
    
private:
    std::string m_configPath;
    std::unordered_map<std::string, std::string> m_config;
};

using ConfigLoaderPtr = std::shared_ptr<ConfigLoader>;

} // namespace Config
} // namespace BeatAnalyzer
