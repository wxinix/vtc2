// clang-format off
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
// clang-format on

#include "io.hpp"

//----------------------------------------------------
TEST_SUITE_BEGIN("TEST_SUITE vtc::io");

using namespace vtc::io;
using namespace vtc::io::func;

TEST_CASE("TEST_CASE - basic vtc types")
{
    std::atomic_bool state;

    SUBCASE("can instantiate IoSignal with IoKind::Input and VehicleDetectorCall<1>")
    {
        Io<IoKind::Input, 1, VehicleDetectorCall<1>> io(state);
        CHECK_EQ(io.number, 1);
        CHECK_EQ(io.kind, IoKind::Input);
        CHECK_EQ(io.name, "Input 1");
        CHECK_EQ(decltype(io)::io_func_type::kind, InputFuncKind::VehicleDetectorCall);
        CHECK_EQ(decltype(io)::io_func_type::number, 1);
        CHECK_EQ(decltype(io)::io_func_type::name, "VehicleDetectorCall 1");
    }

    SUBCASE("can instantiate IoSignal with IoKind::Output and ChannelGreenWalkDriver")
    {
        Io<IoKind::InputOutput, 1, LoadSwitchGreenDriver<1>> io(state);
        CHECK_EQ(io.number, 1);
        CHECK_EQ(io.kind, IoKind::InputOutput);
        CHECK_EQ(io.name, "InputOutput 1");
        CHECK_EQ(decltype(io)::io_func_type::kind, OutputFuncKind::LoadSwitchGreenDriver);
        CHECK_EQ(decltype(io)::io_func_type::number, 1);
        CHECK_EQ(decltype(io)::io_func_type::name, "LoadSwitchGreenDriver 1");
    }

    SUBCASE("can instantiate IoSignal with IoKind::InputOutput and InputFunctionKind")
    {

    }

    SUBCASE("can not instantiate IoSignal with IoKind::Input and OutputFunctionKind")
    {
        // Uncommenting the following line will cause a compile-time error
        // IoSignal<IoKind::Input, 1, OutputFunctionKind::ChannelRedDoNotWalkDriver, 1> io_signal;
    }

    SUBCASE("can not instantiate IoSignal with IoKind::Output and InputFunctionKind")
    {
        // Uncommenting the following line will cause a compile-time error
        // IoSignal<IoKind::Output, 1, InputFunctionKind::VehicleDetectorCall, 1> io_signal;
    }

}

TEST_SUITE_END;