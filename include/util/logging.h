#pragma once

#include <string>
#include <iostream>

namespace BeatAnalyzer {
namespace Util {

/**
 * Logging utilities.
 */
enum class LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};

class Logger {
public:
    static void setLogLevel(LogLevel level);
    
    static void debug(const std::string& message);
    static void info(const std::string& message);
    static void warning(const std::string& message);
    static void error(const std::string& message);
    static void fatal(const std::string& message);
    
private:
    static LogLevel s_logLevel;
    
    static std::string levelToString(LogLevel level);
    static void log(LogLevel level, const std::string& message);
};

} // namespace Util
} // namespace BeatAnalyzer

// Convenience macros
#define LOG_DEBUG(msg) BeatAnalyzer::Util::Logger::debug(msg)
#define LOG_INFO(msg) BeatAnalyzer::Util::Logger::info(msg)
#define LOG_WARN(msg) BeatAnalyzer::Util::Logger::warning(msg)
#define LOG_ERROR(msg) BeatAnalyzer::Util::Logger::error(msg)
#define LOG_FATAL(msg) BeatAnalyzer::Util::Logger::fatal(msg)
