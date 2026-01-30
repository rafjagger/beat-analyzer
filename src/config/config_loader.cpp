#include "config/config_loader.h"
#include "util/logging.h"
#include <fstream>
#include <sstream>

namespace BeatAnalyzer {
namespace Config {

ConfigLoader::ConfigLoader(const std::string& configPath)
    : m_configPath(configPath) {
}

bool ConfigLoader::load() {
    std::ifstream file(m_configPath);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open config file: " + m_configPath);
        return false;
    }
    
    std::string line;
    std::string currentSection;
    
    while (std::getline(file, line)) {
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') continue;
        
        // Handle sections [section]
        if (line[0] == '[' && line[line.length() - 1] == ']') {
            currentSection = line.substr(1, line.length() - 2);
            continue;
        }
        
        // Parse key-value pairs
        size_t delim = line.find(':');
        if (delim != std::string::npos) {
            std::string key = line.substr(0, delim);
            std::string value = line.substr(delim + 1);
            
            // Trim
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            
            // Store with section prefix
            std::string fullKey = currentSection.empty() ? key : 
                                  currentSection + "." + key;
            m_config[fullKey] = value;
        }
    }
    
    LOG_INFO("Configuration loaded from: " + m_configPath);
    return true;
}

std::string ConfigLoader::getString(const std::string& key,
                                   const std::string& defaultValue) const {
    auto it = m_config.find(key);
    return it != m_config.end() ? it->second : defaultValue;
}

int ConfigLoader::getInt(const std::string& key, int defaultValue) const {
    auto it = m_config.find(key);
    if (it != m_config.end()) {
        try {
            return std::stoi(it->second);
        } catch (...) {}
    }
    return defaultValue;
}

float ConfigLoader::getFloat(const std::string& key, float defaultValue) const {
    auto it = m_config.find(key);
    if (it != m_config.end()) {
        try {
            return std::stof(it->second);
        } catch (...) {}
    }
    return defaultValue;
}

bool ConfigLoader::getBool(const std::string& key, bool defaultValue) const {
    std::string value = getString(key, defaultValue ? "true" : "false");
    return value == "true" || value == "1" || value == "yes";
}

bool ConfigLoader::hasKey(const std::string& key) const {
    return m_config.find(key) != m_config.end();
}

} // namespace Config
} // namespace BeatAnalyzer
