/*!
  C++ Virtualization Library of Traffic Cabinet 2
  Copyright (c) Wuping Xin
  MPL 1.1/GPL 2.0/LGPL 2.1 tri-license
*/

#ifndef VIRTUAL_TRAFFIC_CABINET_LOG_H_
#define VIRTUAL_TRAFFIC_CABINET_LOG_H_

#include <filesystem>
#include <spdlog/sinks/msvc_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>

namespace vtc::log {

namespace fs = std::filesystem;

/*!
 * Alias of a shared_ptr of spdlog::logger. Note std::shared_ptr is not thread-safe.
 */
using Logger = std::shared_ptr<spdlog::logger>;

/*!
 * Unnamed namespace for VtcLoggerHolder in order to have a global thread-safe
 * singleton logger without defining it in a separate translation unit.
 */
namespace {
/*!
 * Proxy class for thread-safe singleton logger.
 */
struct LoggerHolder
{
  /*!
   * Inline initialization is possible with C++17. Alternatively, we can define
   * the logger separately outside LoggerHolder class declaration but inside
   * the unnamed namespace. LoggerHolder is a member of unnamed namespace,
   * its static data member does not have external linkage, which allows this
   * header to be included multiple times in different translation unit.
   */
  inline static std::atomic<Logger> logger{nullptr};
};

}// namespace

/*!
 * Thread-safe access to the singleton logger. It ought to be called only after
 * logger has been set up, i.e., setup_logger has been called; otherwise, nullptr
 * will be returned.
 */
Logger logger()
{
  return LoggerHolder::logger;
}

/*!
 * Setup the logger singleton with a given output path and file name. The log file
 * will be saved to the designated output path, using the specified logger name
 * appended with "-log.txt".
 *
 * The parent path needs to be pre-existing. If for any reason, the log file cannot
 * be created, default logger will be used (on Windows, that is is the debug output).
 * If the log file can be created, the log file will be rotated by 1MB file size
 * and up to 3 files.
 *
 * This function can be called multiple times, as long as logger_name is different
 * each time. However, if the same logger name has been used, an exception will
 * be thrown.
 *
 * @param path A path where to save the log file.
 * @param logger_name Name of internal logger as well as part of output file name.
 *
 * @returns true if the intended logger is created, false default logger.
 */
bool setup_logger(const fs::path &path, const std::string &logger_name)
{
  using namespace spdlog;

  const auto p = path / "log";
  std::error_code ec;

  Logger the_logger{nullptr};
  bool default_logger_created;

  // create_directory(p,ec) would return false if exists.
  if (fs::create_directory(p, ec) || fs::exists(p)) {
    const auto log_file = (p / (logger_name + "-log.txt")).string();
    the_logger = rotating_logger_mt(logger_name,// Internal logger name
                                    log_file,   // Output file name
                                    1024 * 1024,// 1 MB log file size
                                    3);         // rotate by 3 files at most
    default_logger_created = false;
  } else {
    the_logger = synchronous_factory::create<sinks::windebug_sink_mt>(logger_name + "_windbg");
    default_logger_created = true;
  }

  LoggerHolder::logger = the_logger;
  return !default_logger_created;
}

}// namespace vtc::log
#endif// VIRTUAL_TRAFFIC_CABINET_LOG_H_
