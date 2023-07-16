// clang-format off
#include <doctest/doctest.h>
// clang-format on
#include "utils.hpp"

TEST_SUITE_BEGIN("TEST_SUITE vtc::traits");

using namespace vtc::utils;

struct MyIndexedTestStruct
{
  static constexpr int index{1};
};

struct MyTestStruct
{
};

TEST_CASE("TEST_CASE - can use HasIndex concept")
{
  CHECK(HasIndex<MyIndexedTestStruct>);
  CHECK(!HasIndex<MyTestStruct>);
}

TEST_CASE("TEST_CASE - can use add_sequence_front")
{
  using seq_t = std::make_integer_sequence<size_t, 3>;
  auto seq = add_sequence_front_t<100, seq_t>{};
  auto val = vtc::utils::get(seq, 0);
  CHECK_EQ(val, 100);
}

TEST_SUITE_END;