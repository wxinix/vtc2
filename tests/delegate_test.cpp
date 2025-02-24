// clang-format off
#include <doctest/doctest.h>
// clang-format on
#include "delegate.hpp"

TEST_SUITE_BEGIN("TEST_SUITE vtc::delegate");

using namespace vtc::delegate;

bool compare(const int a, const int b)
{
    return a == b;
}

class MyTestClass
{
public:
    bool compare(const int a, const int b)
    {
        data_++;
        return a == b;
    }

    [[nodiscard]] bool compare_const(int a, int b) const
    {
        return (a + data_) == b;
    }

private:
    int data_{0};
};

TEST_CASE("TEST_CASE - global function")
{
    auto delegated_compare = delegate(compare);
    CHECK(delegated_compare(5, 5));
    CHECK_FALSE(delegated_compare(5, 6));
}

TEST_CASE("TEST_CASE - class member function")
{
    MyTestClass test_obj;

    SUBCASE("Can delegate non-constant member function")
    {
        auto delegated_compare = delegate(&test_obj, &MyTestClass::compare);
        CHECK(delegated_compare(5, 5));
        CHECK_FALSE(delegated_compare(5, 6));
    }

    SUBCASE("Can delegate constant member function")
    {
        auto delegated_compare = delegate(&test_obj, &MyTestClass::compare_const);
        CHECK(delegated_compare(5, 5));
        CHECK_FALSE(delegated_compare(5, 6));
    }
}

TEST_CASE("TEST_CASE - function object")
{
    const std::function compare_func = [&](const int a, const int b) { return a == b; };
    auto delegated_compare = delegate(compare_func);
    CHECK(delegated_compare(5, 5));
    CHECK_FALSE(delegated_compare(5, 6));
}

TEST_CASE("TEST_CASE - lambda expression")
{
    // lambda needs explicit type
    auto delegated_compare = delegate<bool(int, int)>([&](const int a, const int b) { return a == b; });

    CHECK(delegated_compare(5, 5));
    CHECK_FALSE(delegated_compare(5, 6));
}

TEST_SUITE_END;