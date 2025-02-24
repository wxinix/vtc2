// clang-format off
#include <doctest/doctest.h>
// clang-format on
#include "log.hpp"

TEST_SUITE_BEGIN("TEST_SUITE vtc::log");

using namespace vtc::log;

TEST_CASE("TEST_CASE - can setup file logger")
{
    const auto file_logger_created = setup_logger("test", spdlog::level::trace, fs::current_path());
    CHECK(file_logger_created);
}

TEST_CASE("TEST_CASE - can do file log")
{
    CHECK(nullptr != logger());
    logger()->debug("Hello World! Test message!");
}

TEST_CASE("TEST_CASE - can setup debug view logger")
{
    const auto file_logger_created = setup_logger("test");
    CHECK(!file_logger_created); // should be the default debug view logger
}

TEST_CASE("TEST_CASE - can do debug view log")
{
    CHECK(nullptr != logger());
    logger()->debug("Hello World! Test message!");
}

TEST_SUITE_END;