#pragma once

#include <string>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace BeatAnalyzer {

/**
 * Einfacher .env Parser
 */
class EnvConfig {
public:
    static EnvConfig& instance() {
        static EnvConfig instance;
        return instance;
    }
    
    // Lade .env Datei (falls vorhanden)
    bool load(const std::string& path = ".env") {
        std::ifstream file(path);
        if (!file.is_open()) {
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // Kommentare und leere Zeilen überspringen
            if (line.empty() || line[0] == '#') continue;
            
            auto pos = line.find('=');
            if (pos != std::string::npos) {
                std::string key = trim(line.substr(0, pos));
                std::string value = trim(line.substr(pos + 1));
                m_values[key] = value;
            }
        }
        return true;
    }
    
    // Getter mit Fallback auf Environment-Variable
    std::string getString(const std::string& key, const std::string& defaultValue = "") const {
        // Erst in geladenen Werten suchen
        auto it = m_values.find(key);
        if (it != m_values.end()) {
            return it->second;
        }
        
        // Dann Environment-Variable prüfen
        const char* env = std::getenv(key.c_str());
        return env ? env : defaultValue;
    }
    
    int getInt(const std::string& key, int defaultValue = 0) const {
        std::string val = getString(key, "");
        if (val.empty()) return defaultValue;
        try {
            return std::stoi(val);
        } catch (...) {
            return defaultValue;
        }
    }
    
    float getFloat(const std::string& key, float defaultValue = 0.0f) const {
        std::string val = getString(key, "");
        if (val.empty()) return defaultValue;
        try {
            return std::stof(val);
        } catch (...) {
            return defaultValue;
        }
    }

private:
    EnvConfig() = default;
    
    std::string trim(const std::string& s) const {
        size_t start = s.find_first_not_of(" \t\r\n");
        size_t end = s.find_last_not_of(" \t\r\n");
        return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
    }
    
    std::unordered_map<std::string, std::string> m_values;
};

} // namespace BeatAnalyzer
