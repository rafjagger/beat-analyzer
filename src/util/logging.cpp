#include "util/logging.h"
#include <iostream>
#include <chrono>
#include <iomanip>

namespace BeatAnalyzer {
namespace Util {

LogLevel Logger::s_logLevel = LogLevel::Info;

void Logger::setLogLevel(LogLevel level) {
    s_logLevel = level;
}

void Logger::debug(const std::string& message) {
    log(LogLevel::Debug, message);
}

void Logger::info(const std::string& message) {
    log(LogLevel::Info, message);
}

void Logger::warning(const std::string& message) {
    log(LogLevel::Warning, message);
}

void Logger::error(const std::string& message) {
    log(LogLevel::Error, message);
}

void Logger::fatal(const std::string& message) {
    log(LogLevel::Fatal, message);
}

std::string Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Debug:   return "DEBUG";
        case LogLevel::Info:    return "INFO ";
        case LogLevel::Warning: return "WARN ";
        case LogLevel::Error:   return "ERROR";
        case LogLevel::Fatal:   return "FATAL";
        default:                return "UNKN ";
    }
}

void Logger::log(LogLevel level, const std::string& message) {
    if (level < s_logLevel) return;
    
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&time), "%H:%M:%S") << "] "
       << levelToString(level) << ": " << message;
    
    std::cout << ss.str() << std::endl;
}

} // namespace Util
} // namespace BeatAnalyzer
