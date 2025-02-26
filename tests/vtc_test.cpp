// clang-format off
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
// clang-format on

#include "vtc.hpp"

//----------------------------------------------------
TEST_SUITE_BEGIN("TEST_SUITE vtc");

using namespace vtc;
using namespace vtc::io;
using namespace vtc::frame;

TEST_CASE("TEST_CASE - basic vtc types")
{
    SUBCASE("can set Bit")
    {
        Bit bit{};// Default off
        CHECK_EQ(sizeof(Bit), 1);
        CHECK_EQ(bit, Bit::Off);
        bit = Bit::On;
        CHECK_EQ(bit, Bit::On);
    }

    SUBCASE("can use IoVariableValueType")
    {
        // CHECK(std::is_same_v<Bit, io::IoVariableValueType<VehicleDetCall<1, IoBinding::Fio, 1>>>);
    }
}

TEST_CASE("TEST_CASE - vtc::octets_to_unit32")
{
    SUBCASE("can handle arg value that includes leading `0`s")
    {
        OctetNumber value{0x303037363133};
        auto [valid, number] = octets_to_uint32(value);
        CHECK(valid);
        CHECK_EQ(number, 7613);
    }

    SUBCASE("can handle arg value that includes trailing `0`s")
    {
        OctetNumber value{0x303736313330};
        auto [valid, number] = octets_to_uint32(value);
        CHECK(valid);
        CHECK_EQ(number, 76130);
    }

    SUBCASE("can handle arg value that includes middle `0`s")
    {
        OctetNumber value{0x30373036313330};
        auto [valid, number] = octets_to_uint32(value);
        CHECK(valid);
        CHECK_EQ(number, 706130);
    }

    SUBCASE("can handle arg value that includes non numeric chars")
    {
        OctetNumber value{0x3041363133};
        auto [valid, number] = octets_to_uint32(value);
        CHECK(!valid);
        CHECK_EQ(number, 0xFFFFFFFF);
    }

    SUBCASE("can handle arg value of 0")
    {
        OctetNumber value{0};
        auto [valid, number] = octets_to_uint32(value);
        CHECK(!valid);
        CHECK_EQ(number, 0xFFFFFFFF);
    }

    SUBCASE("can handle arg value that includes middle 0x00")
    {
        OctetNumber value{0x3000363133};
        auto [valid, number] = octets_to_uint32(value);
        CHECK(!valid);
        CHECK_EQ(number, 0xFFFFFFFF);
    }

    SUBCASE("can handle invalid arg type")
    {
        int value{0x30003631};// invalid type
        auto [valid, number] = octets_to_uint32(value);
        CHECK(!valid);
        CHECK_EQ(number, 0xFFFFFFFF);
    }
}

TEST_CASE("TEST_CASE - vtc::io")
{}

TEST_CASE("TEST_CASE - vtc::frame")
{
    SUBCASE("can serialize and deserialize FrameByte")
    {}

    SUBCASE("can serialize and deserialize FrameWord")
    {}

    SUBCASE("can serialize and deserialize FrameCardinal")
    {}

    SUBCASE("can serialize and deserialize FrameOctetNumber")
    {}
}

TEST_CASE("TEST_CASE - vtc::biu")
{
    using namespace vtc::biu;

    SUBCASE("will return DeviceKind::Biu with SilsBiu<1> instance")
    {}

    SUBCASE("can get cabinet index of SilsBiu<1> instance")
    {}

    SUBCASE("can get frame_maps_size of SilsBiu<1> instance")
    {}

    SUBCASE("can dispatch Type 64 command frame with SilsBiu<1> instance")
    {}

    SUBCASE("can dispatch Type 65 command frame with SilsBiu<1> instance")
    {}

    SUBCASE("can dispatch Type 66 command frame with SilsBiu<1> instance")
    {}

    SUBCASE("can reset I/O variables with SilsBiu<1> instance")
    {}

    SUBCASE("can generate response frame Type 190 with SilsBiu<1> instance")
    {}

    SUBCASE("can process command frame with SilsBiu<1> instance")
    {}
}

TEST_CASE("TEST_CASE - vtc::aux")
{}

TEST_CASE("TEST_CASE - vtc::aux::lsw")
{
    using namespace vtc::aux;
    using namespace vtc::aux::lsw;

    SUBCASE("can read LoadSwitch state")
    {}

    SUBCASE("can write LoadSwitch state")
    {}

    SUBCASE("can generate valid LoadSwitchChannels")
    {}

    SUBCASE("can read load switch cabinet")
    {}

    SUBCASE("can read load switch channel")
    {}

    SUBCASE("can create LoadSwitchWiring using LoadSwitchWiringFactory")
    {}
}

TEST_CASE("TEST_CASE - vtc::aux::du")
{
    using namespace aux;
    using namespace aux::du;

    SUBCASE("can get activation state of DetectorUnit<1,1> instance")
    {}

    SUBCASE("can set true to activation state of DetectorUnit<1,1> instance")
    {}

    SUBCASE("can get cabinet index of DetectorUnit<1,2> instance")
    {}

    SUBCASE("can get detector channel of DetectorUnit<1,2> instance")
    {}

    SUBCASE("can generate channels sequence using du::DetectorChannels")
    {}
}

TEST_CASE("TEST_CASE - vtc::rack")
{
    using namespace vtc::rack;

    SUBCASE("can reset VehicleDetCall 1..8 calling Reset()")
    {}

    SUBCASE("will generate Frame 193 calling GenerateResponseFrame with arg 193")
    {}

    SUBCASE("will return 0 calling controller_id() as initial value")
    {}

    SUBCASE("will throw runtime error calling SetSimulator with nullptr arg")
    {}
}

TEST_SUITE_END;