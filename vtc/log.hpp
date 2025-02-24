#pragma once

#include <filesystem>

#include "spdlog/sinks/msvc_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

namespace vtc::log {

namespace fs = std::filesystem;

/**
 * Alias of a shared_ptr of spdlog::logger. Note std::shared_ptr is not thread-safe.
 */
using Logger = std::shared_ptr<spdlog::logger>;

/**
 * Proxy class for thread-safe singleton logger.
 */
struct LoggerHolder
{
    /**
   * Inline static variable ensures a single instance across all translation units.
   * `std::atomic` provides thread-safe access.
   */
    inline static std::atomic<Logger> logger{nullptr};
};

/**
 * Thread-safe access to the singleton logger. It ought to be called only after
 * logger has been set up, i.e., setup_logger has been called; otherwise, nullptr
 * will be returned.
 */
inline Logger logger()
{
    return LoggerHolder::logger.load();// Ensure safe automatic read
}

/**
 * Set up the logger singleton with a given output path and file name. The log file
 * will be saved to the designated output path, using the specified logger name
 * appended with "-log.txt".
 *
 * The parent path needs to be pre-existing. If for any reason, the log file cannot
 * be created, default logger will be used (on Windows, that is the debug output).
 * If the log file can be created, the log file will be rotated by 1MB file size
 * and up to 3 files.
 *
 * This function can be called multiple times, as long as logger_name is different
 * each time. However, if the same logger name has been used, an exception will
 * be thrown.
 *
 * @param logger_name Name of internal logger as well as part of output file name.
 * @param level Log level.
 * @param path A path where to save the log file. If unspecified, will create debug view.
 *
 * @returns true if the intended logger is created, false default logger.
 */
inline bool setup_logger(const std::string &logger_name, const spdlog::level::level_enum level = spdlog::level::trace,
                         const fs::path &path = {})
{
    using namespace spdlog;

    Logger the_logger{nullptr};
    bool default_logger_created{true};

    if (!path.empty()) {
        const auto p = path / "log";
        // create_directory(p,ec) would return false if exists.
        if (std::error_code ec; create_directory(p, ec) || exists(p)) {
            const auto log_file = (p / (logger_name + "-log.txt")).string();
            the_logger = rotating_logger_mt(logger_name,// Internal logger name
                                            log_file,   // Output file name
                                            1024 * 1024,// 1 MB log file size
                                            3);         // rotate by 3 files at most
            default_logger_created = false;
        }
    }

    if (!the_logger) {
        the_logger = synchronous_factory::create<sinks::windebug_sink_mt>(logger_name + "_debugview", false);
        default_logger_created = true;
    }

    the_logger->set_level(level);
    LoggerHolder::logger = the_logger;
    return !default_logger_created;
}

}// namespace vtc::log
