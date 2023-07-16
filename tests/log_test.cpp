// clang-format off
#include <doctest/doctest.h>
// clang-format on
#include "log.hpp"

TEST_SUITE_BEGIN("TEST_SUITE vtc::log");

using namespace vtc::log;

TEST_CASE("TEST_CASE - can setup logger")
{
  auto created = setup_logger(fs::current_path(), "test");
  CHECK(created);
}

TEST_CASE("TEST_CASE - can do log")
{
  CHECK(nullptr != logger());
  logger()->info("Hello World! Test message!");
}

TEST_SUITE_END;