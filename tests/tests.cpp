#pragma warning(disable : 4068)
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedMacroInspection"

// clang-format off
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
// clang-format on

#include <array>
#include <stdexcept>

#include <pugixml/pugixml.hpp>

#include <vtc/log.hpp>
#include <vtc/traits.hpp>
#include <vtc/vtc.hpp>

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

  SUBCASE("can use DeviceKind")
  {
    DeviceKind dk = DeviceKind::Mmu;
    CHECK_EQ(dk, DeviceKind::Mmu);
    dk = DeviceKind::Biu;
    CHECK_EQ(dk, DeviceKind::Biu);
    dk = DeviceKind::Cu;
    CHECK_EQ(dk, DeviceKind::Cu);
    dk = DeviceKind::Rack;
    CHECK_EQ(dk, DeviceKind::Rack);
  }

  SUBCASE("can use IoBinding::Cu")
  {
    using T = io::IoVariable<1, IoBinding::Cu, Cardinal, 1>;
    T var{};
    CHECK(std::is_same_v<T::value_t, Cardinal>);
    CHECK_EQ(var.cabinet(), 1);
    CHECK_EQ(var.index(), 1);
    CHECK_EQ(var.binding(), IoBinding::Cu);
    CHECK_EQ(var.value, 0);
  }

  SUBCASE("can use IoBinding::Mmu")
  {
    using T = io::IoVariable<1, IoBinding::Mmu, Cardinal, 1>;
    T var{};
    CHECK(std::is_same_v<T::value_t, Cardinal>);
    CHECK_EQ(var.cabinet(), 1);
    CHECK_EQ(var.index(), 1);
    CHECK_EQ(var.binding(), IoBinding::Mmu);
    CHECK_EQ(var.value, 0);
  }

  SUBCASE("can use IoVariableValueType")
  {
    CHECK(std::is_same_v<Bit, io::IoVariableValueType<VehicleDetCall<1, IoBinding::Fio, 1>>>);
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
{
  SUBCASE("can specialize VehicleDetCall with cabinet index and detector channel")
  {
    using type = VehicleDetCall<1, IoBinding::Cu, 5>;
    CHECK_EQ(type::cabinet(), 1);
    CHECK_EQ(type::index(), 5);
  }

  SUBCASE("will fail ValidDetectorChannel using invalid detector channel")
  {
    CHECK(!ValidDetectorChannel<1 + MAX_VEHICLE_DETECTORS>);
  }

  SUBCASE("can read and write SimulationStartTime")
  {
    io::Global::instance<SimulationStartTime<IoBinding::Cu>>.value = 1682724403;// Friday, April 28, 2023 11:26:43 PM
    io::Global::instance<SimulationStartTime<IoBinding::Mmu>>.value = 1;
    CHECK_EQ(io::Global::instance<SimulationStartTime<IoBinding::Cu>>.value, 1682724403);
    CHECK_EQ(io::Global::instance<SimulationStartTime<IoBinding::Mmu>>.value, 1);
  }

  SUBCASE("can read and write ChannelRedDoNotWalkDriver")
  {
    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Cu, 5>>.value = Bit::On;
    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Cu, 5>>.value, Bit::On);

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Cu, 5>>.value = Bit::Off;
    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Cu, 5>>.value, Bit::Off);
  }

  SUBCASE("can read and write ChannelYellowPedClearDriver")
  {
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Cu, 5>>.value = Bit::On;
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Cu, 5>>.value, Bit::On);

    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Cu, 5>>.value = Bit::Off;
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Cu, 5>>.value, Bit::Off);
  }

  SUBCASE("can read and write ChannelGreenWalkDriver")
  {
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Cu, 5>>.value = Bit::On;
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Cu, 5>>.value, Bit::On);

    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Cu, 5>>.value = Bit::Off;
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Cu, 5>>.value, Bit::Off);
  }

  SUBCASE("can assign octets to IntersectionID instance")
  {
    IntersectionID<1, IoBinding::Cu> int_id;
    int_id.value = 0x3031323334353637;
    auto [valid, number] = octets_to_uint32(int_id.value);
    CHECK(valid);
    CHECK_EQ(number, 1234567);
  }

  SUBCASE("can convert IntersectionID instance to uint32")
  {
    IntersectionID<1, IoBinding::Cu> int_id;
    int_id.value = 0x3031323334353637;
    uint32_t value = int_id.to_uint32();
    CHECK_EQ(value, 1234567);
  }
}

TEST_CASE("TEST_CASE - vtc::frame")
{
  SUBCASE("can serialize and deserialize FrameByte")
  {
    using var_type = io::IoVariable<1, IoBinding::Cu, Byte, 0>;
    frame::detail::FrameByte<var_type, 0> elem{};
    CHECK_EQ(0, elem.pos);

    std::array<Byte, 5> data = {0x13, 0x02, 0x03, 0x04, 0x05};
    elem << data;
    CHECK_EQ(io::Global::instance<var_type>.value, 0x13);

    io::Global::instance<var_type>.value = 0x14;
    elem >> data;
    CHECK_EQ(data[0], 0x14);
  }

  SUBCASE("can serialize and deserialize FrameWord")
  {
    using var_type = io::IoVariable<1, IoBinding::Cu, Word, 1>;
    frame::detail::FrameWord<var_type, 1> elem{};
    CHECK_EQ(1, elem.pos);

    std::array<Byte, 5> data = {0x13, 0x02, 0x03, 0x04, 0x05};
    elem << data;
    CHECK_EQ(io::Global::instance<var_type>.value, 0x0302);

    io::Global::instance<var_type>.value = 0x0A0C;
    elem >> data;
    CHECK_EQ(data[1], 0x0C);
  }

  SUBCASE("can serialize and deserialize FrameCardinal")
  {
    using var_type = io::IoVariable<1, IoBinding::Cu, Cardinal, 2>;
    frame::detail::FrameCardinal<var_type, 2> elem{};
    CHECK_EQ(2, elem.pos);

    std::array<Byte, 6> data = {0x13, 0x02, 0x03, 0x04, 0x05, 0x06};
    elem << data;
    CHECK_EQ(io::Global::instance<var_type>.value, 0x06050403);

    io::Global::instance<var_type>.value = 0x0A0B0C0D;
    elem >> data;
    CHECK_EQ(data[2], 0x0D);
  }

  SUBCASE("can serialize and deserialize FrameOctetNumber")
  {
    using var_type = io::IoVariable<1, IoBinding::Cu, OctetNumber, 2>;
    frame::detail::FrameOctetNumber<var_type, 2> elem{};
    CHECK_EQ(2, elem.pos);
    // Special for OctetNumber, the first data byte goes as the most significant byte
    // of the 8-byte integer.
    std::array<Byte, 10> data = {0x13, 0x02, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x00};
    elem << data;
    CHECK_EQ(io::Global::instance<var_type>.value, 0x3334353637383900);

    io::Global::instance<var_type>.value = 0x3934353637383900;
    elem >> data;
    CHECK_EQ(data[2], 0x39);
  }

  SUBCASE("can deserialize SimulatorInitialHandshakeFrame from byte array")
  {
    // For CU = 1
    CHECK(std::is_same_v<SimulatorInitialHandshakeFrame<1, IoBinding::Fio>, biu::BiuFrameType<1, 64>::type>);

    SimulatorInitialHandshakeFrame<1, IoBinding::Fio> frame{};
    CHECK_EQ(frame.byte_size(), 11);

    std::array<Byte, 11> data_in = {254, 0x83, 64, 0x30, 0x37, 0x36, 0x31, 0x33};
    frame << data_in;
    io::IoVariableValueType<IntersectionID<1, IoBinding::Fio>> value = io::Global::instance<IntersectionID<1, IoBinding::Fio>>.value;
    CHECK_EQ(value, 0x3037363133000000);

    auto &int_id = io::Global::instance<IntersectionID<1, IoBinding::Fio>>;
    uint32_t int32_value = int_id.to_uint32();
    CHECK_EQ(int32_value, 7613);
  }

  SUBCASE("can serialize SimulatorCallDataFrame to byte array")
  {
    // For CU = 1
    CHECK(std::is_same_v<SimulatorCallDataFrame<1, IoBinding::Fio>, biu::BiuFrameType<1, 193>::type>);

    SimulatorCallDataFrame<1, IoBinding::Fio> frame{};
    CHECK_EQ(frame.byte_size(), 23);
    CHECK_EQ(frame.frame_kind(), frame::FrameKind::Response);

    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value = Bit::On;

    // const auto now = std::chrono::system_clock::now();
    // instance<SimulationStartTime>.value = now.time_since_epoch().count();
    io::Global::instance<SimulationStartTime<IoBinding::Fio>>.value = 0x64726B1B;

    std::array<Byte, 23> data = {};
    frame >> data;

    CHECK_EQ(data[0], 254);
    CHECK_EQ(data[1], 0x83);
    CHECK_EQ(data[2], 193);

    CHECK_EQ(data[3], 0b1010'0101);
    CHECK_EQ(data[19], 0x1B);
    CHECK_EQ(data[22], 0x64);
  }

  SUBCASE("can deserialize SimulatorLoadSwitchDriversFrame from byte array")
  {
    // For CU = 1
    CHECK(std::is_same_v<SimulatorLoadSwitchDriversFrame<1, IoBinding::Fio>, biu::BiuFrameType<1, 66>::type>);

    SimulatorLoadSwitchDriversFrame<1, IoBinding::Fio> frame{};
    CHECK_EQ(frame.byte_size(), 19);

    std::array<Byte, 19> data = {254, 0x83, 66, 0b1111'1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    frame << data;

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::On);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);
  }
}

TEST_CASE("TEST_CASE - vtc::log")
{
  using namespace vtc::log;

  SUBCASE("can setup logger")
  {
    auto created = setup_logger(fs::current_path(), "test");
    CHECK(created);
  }

  SUBCASE("can do log")
  {
    CHECK(nullptr != logger());
    logger()->info("Hello World! Test message!");
  }
}

TEST_CASE("TEST_CASE - vtc::biu")
{
  using namespace vtc::biu;

  SUBCASE("will return DeviceKind::Biu with SilsBiu<1> instance")
  {
    auto &sim_biu = biu::Global::instance<SilsBiu<1>>;
    CHECK(sim_biu.device_kind() == DeviceKind::Biu);
  }

  SUBCASE("can get cabinet index of SilsBiu<1> instance")
  {
    auto &sim_biu = biu::Global::instance<SilsBiu<1>>;
    CHECK_EQ(sim_biu.cabinet(), 1);
  }

  SUBCASE("can get frame_maps_size of SilsBiu<1> instance")
  {
    SilsBiu<1> sim_biu{};
    CHECK_EQ(sim_biu.frame_maps_size(), 3);
  }

  SUBCASE("can dispatch Type 64 command frame with SilsBiu<1> instance")
  {
    io::Global::instance<IntersectionID<1, IoBinding::Fio>>.value = 0;
    auto *biu = new SilsBiu<1>{};
    std::array<Byte, 11> data_in = {254, 0x83, 64, 0x30, 0x37, 0x36, 0x31, 0x33};

    auto [success, data_out] = biu->Dispatch(data_in);
    CHECK(success);
    CHECK_EQ(data_out.size(), 3);
    CHECK_EQ(data_out[0], 254);
    CHECK_EQ(data_out[1], 0x83);
    CHECK_EQ(data_out[2], 192);
    CHECK_EQ(io::Global::instance<IntersectionID<1, IoBinding::Fio>>.to_uint32(), 7613);

    delete biu;
  }

  SUBCASE("can dispatch Type 65 command frame with SilsBiu<1> instance")
  {
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value = Bit::On;
    io::Global::instance<SimulationStartTime<IoBinding::Fio>>.value = 0x64726B1B;

    auto *biu = new SilsBiu<1>{};
    std::array<Byte, 3> data_in = {254, 0x83, 65};
    auto [success, data_out] = biu->Dispatch(data_in);
    CHECK(success);
    CHECK_EQ(data_out.size(), 23);

    CHECK_EQ(data_out[0], 254);
    CHECK_EQ(data_out[1], 0x83);
    CHECK_EQ(data_out[2], 193);
    CHECK_EQ(data_out[3], 0b1000'0111);
    CHECK_EQ(data_out[19], 0x1B);
    CHECK_EQ(data_out[22], 0x64);

    delete biu;
  }

  SUBCASE("can dispatch Type 66 command frame with SilsBiu<1> instance")
  {
    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value = Bit::Off;

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value = Bit::Off;
    auto *biu = new SilsBiu<1>{};

    std::array<Byte, 19> data_in = {254, 0x83, 66, 0b1111'1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    auto [success, data_out] = biu->Dispatch(data_in);
    CHECK(success);
    CHECK_EQ(data_out.size(), 3);

    CHECK_EQ(data_out[0], 254);
    CHECK_EQ(data_out[1], 0x83);
    CHECK_EQ(data_out[2], 194);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::On);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);
    delete biu;
  }

  SUBCASE("can reset I/O variables with SilsBiu<1> instance")
  {
    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::On;

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value = Bit::On;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value = Bit::On;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value = Bit::On;

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value = Bit::On;

    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value = Bit::On;

    auto *biu = new SilsBiu<1>{};
    biu->Reset();

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value, Bit::Off);

    delete biu;
  }

  SUBCASE("can generate response frame Type 190 with SilsBiu<1> instance")
  {
    auto *biu = new SilsBiu<1>{};
    biu->Reset();

    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value = Bit::On;

    auto [success, bytes] = biu->GenerateResponseFrame(190);
    CHECK(not success);

    std::tie(success, bytes) = biu->GenerateResponseFrame(193);

    CHECK(success);
    CHECK_EQ(bytes[3], 0b1011'0101);
    CHECK_EQ(bytes.size(), 23);
    delete biu;
  }

  SUBCASE("can process command frame with SilsBiu<1> instance")
  {
    auto *biu = new SilsBiu<1>{};
    biu->Reset();

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);

    std::array<Byte, 19> bad_data_in = {254, 0x83, 60, 0b1111'1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    auto success = biu->ProcessCommandFrame(bad_data_in);
    CHECK(not success);

    std::array<Byte, 19> good_data_in = {254, 0x83, 66, 0b1111'1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    success = biu->ProcessCommandFrame(good_data_in);
    CHECK(success);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value, Bit::On);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 3>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 3>>.value, Bit::Off);

    delete biu;
  }
}

TEST_CASE("TEST_CASE - vtc::aux")
{
  using namespace vtc::aux;

  SUBCASE("can create wirings using make_wirings")
  {
    auto lsw_wirings = make_wirings<1>(lsw::LoadSwitchWiringFactory{}, lsw::LoadSwitchChannels{});
    auto det_wirings = make_wirings<1>(du::DetectorWiringFactory{}, du::DetectorChannels{});

    auto lsw_wirings_size = std::tuple_size<decltype(lsw_wirings)>::value;
    auto det_wirings_size = std::tuple_size<decltype(det_wirings)>::value;

    CHECK(lsw_wirings_size == MAX_LOAD_SWITCHES);
    CHECK(det_wirings_size == MAX_VEHICLE_DETECTORS);

    auto &lsw_wiring = std::get<0>(lsw_wirings);
    auto &lsw = std::get<0>(lsw_wiring);
    auto &sg = std::get<1>(lsw_wiring);
    auto sg_id = std::get<0>(sg);
    CHECK(sg_id == 0);
    CHECK(lsw.cabinet() == 1);
    CHECK(lsw.channel() == 1);

    auto &det_wiring = std::get<1>(det_wirings);
    auto &det = std::get<0>(det_wiring);
    auto &sids = std::get<1>(det_wiring);
    CHECK(det.cabinet() == 1);
    CHECK(det.channel() == 2);
    CHECK(sids.empty());
  }

  SUBCASE("can call for_each on load switch wirings")
  {
    auto lsw_wirings = make_wirings<1>(lsw::LoadSwitchWiringFactory{}, lsw::LoadSwitchChannels{});

    for_each(lsw_wirings, [&](auto &&el) {
      auto &[lsw, sg] = el;
      auto &turn_movements = std::get<1>(sg);
      CHECK(turn_movements.empty());
      if (lsw.channel() % 8 == 0) {
        lsw.set_state(lsw::LoadSwitchState::Yellow);
      } else {
        lsw.set_state(lsw::LoadSwitchState::Red);
      }
    });

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 16>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 16>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 16>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 2>>.value, Bit::Off);

    CHECK_EQ(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 31>>.value, Bit::On);
    CHECK_EQ(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 31>>.value, Bit::Off);
    CHECK_EQ(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 31>>.value, Bit::Off);
  }

  SUBCASE("can call for_each on detector wirings")
  {
    auto det_wirings = make_wirings<1>(du::DetectorWiringFactory{}, du::DetectorChannels{});
    for_each(det_wirings, [&](auto &&el) {
      auto &[det, sids] = el;
      if (det.channel() % 4 == 0) {
        det.set_activated(true);
      } else {
        det.set_activated(false);
      }
    });

    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value == Bit::On);
  }
}

TEST_CASE("TEST_CASE - vtc::aux::lsw")
{
  using namespace vtc::aux;
  using namespace vtc::aux::lsw;

  SUBCASE("can read LoadSwitch state")
  {
    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;

    auto &ls = lsw::Global::instance<LoadSwitch<1, 1>>;
    auto state = ls.state();
    CHECK_EQ(state, lsw::LoadSwitchState::Blank);

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;

    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Red);

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;

    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Yellow);

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::On;

    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Green);

    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;

    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Blank);
  }

  SUBCASE("can write LoadSwitch state")
  {
    io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;
    io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value = Bit::Off;

    auto &ls = lsw::Global::instance<LoadSwitch<1, 1>>;
    auto *als = &ls;

    auto state = ls.state();
    CHECK_EQ(state, lsw::LoadSwitchState::Blank);

    als->set_state(LoadSwitchState::Red);
    CHECK(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::On);
    CHECK(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Red);

    ls.set_state(LoadSwitchState::Yellow);
    CHECK(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value == Bit::On);
    CHECK(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Yellow);

    ls.set_state(LoadSwitchState::Green);
    CHECK(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::On);
    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Green);

    ls.set_state(LoadSwitchState::Blank);
    CHECK(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    state = ls.state();
    CHECK_EQ(state, LoadSwitchState::Blank);
  }

  SUBCASE("can generate valid LoadSwitchChannels")
  {
    auto load_switch_indexes = lsw::LoadSwitchChannels{};
    CHECK_EQ(load_switch_indexes.size(), 32);
    auto val = get(load_switch_indexes, 0);
    CHECK_EQ(val, 1);
    val = get(load_switch_indexes, 31);
    CHECK_EQ(val, 32);
  }

  SUBCASE("can read load switch cabinet")
  {
    auto &ls = lsw::Global::instance<LoadSwitch<1, 1>>;
    CHECK(ls.cabinet() == 1);
  }

  SUBCASE("can read load switch channel")
  {
    auto &ls = lsw::Global::instance<LoadSwitch<1, 2>>;
    CHECK(ls.channel() == 2);
  }

  SUBCASE("can create LoadSwitchWiring using LoadSwitchWiringFactory")
  {
    LoadSwitchWiring<1, 1> wiring = LoadSwitchWiringFactory::make<1, 1>();
    auto &ls = std::get<0>(wiring);
    ls.set_state(LoadSwitchState::Green);
    CHECK(io::Global::instance<ChannelRedDoNotWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelYellowPedClearDriver<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<ChannelGreenWalkDriver<1, IoBinding::Fio, 1>>.value == Bit::On);
    auto state = lsw::Global::instance<LoadSwitch<1, 1>>.state();
    CHECK_EQ(state, LoadSwitchState::Green);
  }
}

TEST_CASE("TEST_CASE - vtc::aux::du")
{
  using namespace aux;
  using namespace aux::du;

  SUBCASE("can get activation state of DetectorUnit<1,1> instance")
  {
    io::Global::instance<VehicleDetCall<1, io::IoBinding::Fio, 1>>.value = Bit::Off;
    auto &det = du::Global::instance<DetectorUnit<1, 1>>;
    auto activated = det.activated();
    CHECK(activated == false);

    io::Global::instance<VehicleDetCall<1, io::IoBinding::Fio, 1>>.value = Bit::On;
    activated = det.activated();
    CHECK(activated == true);
  }

  SUBCASE("can set true to activation state of DetectorUnit<1,1> instance")
  {
    io::Global::instance<VehicleDetCall<1, io::IoBinding::Fio, 1>>.value = Bit::Off;
    auto &det = du::Global::instance<DetectorUnit<1, 1>>;
    auto *det_ptr = &det;
    det_ptr->set_activated(true);
    CHECK(io::Global::instance<VehicleDetCall<1, io::IoBinding::Fio, 1>>.value == Bit::On);
  }

  SUBCASE("can get cabinet index of DetectorUnit<1,2> instance")
  {
    auto &det = du::Global::instance<DetectorUnit<1, 2>>;
    auto *det_ptr = &det;
    CHECK(det_ptr->cabinet() == 1);
    CHECK(det.cabinet() == 1);
  }

  SUBCASE("can get detector channel of DetectorUnit<1,2> instance")
  {
    auto &det = du::Global::instance<DetectorUnit<1, 2>>;
    auto *det_ptr = &det;
    CHECK(det_ptr->channel() == 2);
    CHECK(det.channel() == 2);
  }

  SUBCASE("can generate channels sequence using du::DetectorChannels")
  {
    auto chs = du::DetectorChannels{};
    CHECK_EQ(chs.size(), 128);
    auto val = get(chs, 0);
    CHECK_EQ(val, 1);
    val = get(chs, 127);
    CHECK_EQ(val, 128);
  }

  SUBCASE("can create DetectorWiring<1,1> instance by DetectorWiringFactory::make<1,1>")
  {
    DetectorWiring<1, 1> wiring = DetectorWiringFactory::make<1, 1>();
    auto &det = std::get<0>(wiring);
    det.set_activated(false);
    CHECK(io::Global::instance<io::VehicleDetCall<1, IoBinding::Fio, 1>>.value == Bit::Off);
    det.set_activated(true);
    CHECK(io::Global::instance<io::VehicleDetCall<1, IoBinding::Fio, 1>>.value == Bit::On);
  }
}

TEST_CASE("TEST_CASE - vtc::rack")
{
  using namespace vtc::rack;
  auto &rack = rack::Global::Global::instance<SilsBiuRack<1>>;

  SUBCASE("will return DeviceKind::Rack with SimulatorBiuRack<1> instance")
  {
    CHECK(rack.device_kind() == DeviceKind::Rack);
    auto *device = &rack;
    CHECK(device->device_kind() == DeviceKind::Rack);
  }

  SUBCASE("can reset VehicleDetCall 1..8 calling Reset()")
  {
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value = Bit::On;

    rack.Reset();

    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value == Bit::Off);
  }

  SUBCASE("will generate Frame 193 calling GenerateResponseFrame with arg 193")
  {
    rack.Reset();

    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 4>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value = Bit::On;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value = Bit::Off;
    io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value = Bit::On;

    auto [success, frame_data] = rack.GenerateResponseFrame(193);

    CHECK(success);
    CHECK(frame_data[2] == 193);
    CHECK(frame_data.size() == 23);
    CHECK(frame_data[3] == 0b10110101);
  }

  SUBCASE("will return 0 calling controller_id() as initial value")
  {
    CHECK(rack.controller_id() == 0);
  }

  SUBCASE("will throw runtime error calling SetSimulator with nullptr arg")
  {
    DOCTEST_CHECK_THROWS_AS([&]() { rack.SetSimulator(nullptr); }(), std::runtime_error);
  }
}

/**
 * For testing purpose only.
 */
class XilsTestSimulator : public vtc::xils::IXilSimulator
{
public:
  ControllerID GetControllerID(CabinetIndex cabinet) override
  {
    auto xpath_expr = std::format("/XilSCI/cabinet[@index='{}']/cu/@id", cabinet);
    pugi::xpath_node xp_node = m_xml_document.select_node(xpath_expr.c_str());
    pugi::xml_attribute attr = xp_node.attribute();
    return std::strtoul(attr.value(), nullptr, 10);
  }

  bool GetSensorState(DetectionZoneID sensor_id) override
  {
    switch (sensor_id) {
      case 5: //1
      case 1: //2
      case 2: //3
      case 9: //5
      case 8: //6
      case 6: //7
      case 7: //8
      case 28://9
        return true;
      case 29://10
      case 24://11
      case 25://12
      case 33://14
      case 34://15
      case 35://16
      case 36://17
      case 30://18
      case 42://19
      case 38://20
        return false;
      case 39://21
      case 45://23
      case 46://24
      case 47://25
      case 48://26
      case 18://27
      case 16://28
      case 12://29
      case 13://30
        return true;
      case 19://32
      case 20://33
      case 21://34
      case 22://35
      case 23://36
      case 11://49
      case 10://50
      case 32://51
        return false;
      case 31://52
      case 44://53
      case 43://54
      case 17://55
      case 14://56
        return true;
      default:
        return false;
    }

    return true;
  }

  void SetSignalState(SignalGroupID sg_id,
                      ApproachID approach_id,
                      TurnIndex turn_id,
                      aux::lsw::LoadSwitchState state) override
  {
    auto xpath_expr = std::format(
        "/XilSCI/cabinet/wirings/load_switch_wirings/load_switch_wiring[signal_group/movement[@approach='{}' and @turn='{}']]/load_switch/@channel",
        approach_id,
        turn_id);

    auto xp_node = m_xml_document.select_node(xpath_expr.c_str());
    pugi::xml_attribute attr = xp_node.attribute();
    auto ch = std::strtoul(attr.value(), nullptr, 10);
    cabinet_1_lsw_states[ch] = state;
  }

  std::span<const DetectionZoneID> GetDetectionZoneIDs(ControllerID id, DetectorChannel ch) override
  {
    auto xpath_expr = std::format(
        "/XilSCI/cabinet[cu/@id='{}']/wirings/detector_wirings/detector_wiring[detector_unit/@channel='{}']/detection_zones",
        id,
        ch);

    auto xp_node = m_xml_document.select_node(xpath_expr.c_str());

    uint32_t i = 0;
    uint32_t detection_zone_id = 0;
    for (auto &node : xp_node.node().children("detection_zone")) {
      detection_zone_id = std::strtoul(node.attribute("id").value(), nullptr, 10);
      m_detection_zones[i] = detection_zone_id;
      i++;
    }
    return {m_detection_zones.data(), i};
  }

  SignalGroupEx GetSignalGroup(ControllerID id, LoadSwitchChannel ch) override
  {
    auto xpath_expr = std::format(
        "/XilSCI/cabinet[cu/@id='{}']/wirings/load_switch_wirings/load_switch_wiring[load_switch/@channel='{}']/signal_group",
        id,
        ch);

    auto xp_node = m_xml_document.select_node(xpath_expr.c_str());
    auto xml_node = xp_node.node();

    uint32_t i = 0;
    SignalGroupID sg_id = std::strtoul(xml_node.attribute("id").value(), nullptr, 10);

    ApproachID approach_id;
    TurnIndex turn_id;

    for (auto &node : xp_node.node().children("movement")) {
      approach_id = std::strtoul(node.attribute("approach").value(), nullptr, 10);
      turn_id = std::strtoul(node.attribute("turn").value(), nullptr, 10);
      m_movements[i] = std::make_tuple(approach_id, turn_id);
      i++;
    }

    return {sg_id, {m_movements.data(), i}};
  }

  explicit XilsTestSimulator(const std::filesystem::path &config_file_path)
  {
    auto parse_result = m_xml_document.load_file(config_file_path.c_str());

    if (!parse_result) {
      throw std::invalid_argument(std::format("Failed to load config {}: {}",
                                              config_file_path.string(),
                                              parse_result.description()));
    }
  }

  std::array<aux::lsw::LoadSwitchState, 32> cabinet_1_lsw_states{};

private:
  pugi::xml_document m_xml_document{pugi::xml_document{}};
  std::array<DetectionZoneID, 64> m_detection_zones{};
  std::array<Movement, 64> m_movements{};
};

TEST_CASE("TEST_CASE - vtc::rack with mock simulator")
{
  auto simulator{XilsTestSimulator(std::filesystem::current_path() / "xils.config.xml")};
  auto &rack = vtc::rack::Global::Global::instance<vtc::rack::SilsBiuRack<1>>;

  rack.SetSimulator(&simulator);
  rack.Reset();

  SUBCASE("can get cabinet")
  {
    auto cabinet = std::remove_cvref_t<decltype(rack)>::cabinet();
    CHECK(cabinet == 1);
  }

  SUBCASE("can get rack_size")
  {
    auto size = std::remove_cvref_t<decltype(rack)>::rack_size();
    CHECK(size == 1);
  }

  SUBCASE("can get controller id")
  {
    auto id = simulator.GetControllerID(1);
    CHECK(id == 2023);
  }

  SUBCASE("can get detection zone ids for controller 2023 and detector channel 1 to 4")
  {
    auto ids = simulator.GetDetectionZoneIDs(2023, 1);
    CHECK(ids.size() == 1);
    CHECK(ids[0] == 5);

    ids = simulator.GetDetectionZoneIDs(2023, 2);
    CHECK(ids.size() == 1);
    CHECK(ids[0] == 1);

    ids = simulator.GetDetectionZoneIDs(2023, 3);
    CHECK(ids.size() == 1);
    CHECK(ids[0] == 2);

    ids = simulator.GetDetectionZoneIDs(2023, 4);
    CHECK(ids.size() == 0);
  }

  SUBCASE("can get signal group for controller 2023 and load switch channel 1")
  {
    auto sg = simulator.GetSignalGroup(2023, 1);
    CHECK(std::get<0>(sg) == 99);
    CHECK(std::get<1>(sg).size() == 1);
    CHECK(std::get<0>(std::get<1>(sg)[0]) == 3);
    CHECK(std::get<1>(std::get<1>(sg)[0]) == 2);
  }

  SUBCASE("can handle ProcessDetectorWirings")
  {
    rack.ProcessDetectorWirings();

    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 1>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 2>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 3>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 5>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 7>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 8>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 9>>.value == Bit::On);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 10>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 11>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 12>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 13>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 14>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 15>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 16>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 17>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 18>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 19>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 20>>.value == Bit::Off);
    CHECK(io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 21>>.value == Bit::On);

    auto [success, frame_data] = rack.GenerateResponseFrame(193);

    CHECK(success);
    CHECK(frame_data[2] == 193);
    CHECK(frame_data.size() == 23);
    CHECK(frame_data[3] == 0b11110111);
    CHECK(frame_data[4] == 0b00000001);
    CHECK(frame_data[5] == 0b11010000);
  }

  SUBCASE("can handle ProcessLoadSwitchWirings")
  {
    // 1 - G; 2 - R; 3 - Y; 4 - Blank; 5 - R; 6 - R; 7 - Y; 8 - G;
    std::array<Byte, 19> data = {254, 0x83, 66, 0b1000'1100, 0b10011010, 0b10001000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    auto success = rack.ProcessCommandFrame(data);
    CHECK(success);
    rack.ProcessLoadSwitchWirings();
    CHECK(simulator.cabinet_1_lsw_states[1] == aux::lsw::LoadSwitchState::Green);
    CHECK(simulator.cabinet_1_lsw_states[2] == aux::lsw::LoadSwitchState::Red);
    CHECK(simulator.cabinet_1_lsw_states[3] == aux::lsw::LoadSwitchState::Yellow);
    CHECK(simulator.cabinet_1_lsw_states[4] == aux::lsw::LoadSwitchState::Blank);
    CHECK(simulator.cabinet_1_lsw_states[5] == aux::lsw::LoadSwitchState::Red);
    CHECK(simulator.cabinet_1_lsw_states[5] == aux::lsw::LoadSwitchState::Red);
    CHECK(simulator.cabinet_1_lsw_states[7] == aux::lsw::LoadSwitchState::Yellow);
    CHECK(simulator.cabinet_1_lsw_states[8] == aux::lsw::LoadSwitchState::Green);
  }
}

struct MyIndexedTestStruct
{
  static constexpr int index{1};
};

struct MyTestStruct
{
};

TEST_CASE("TEST_CASE - traits")
{
  SUBCASE("can use HasIndex concept")
  {
    CHECK(HasIndex<MyIndexedTestStruct>);
    CHECK(!HasIndex<MyTestStruct>);
  }

  SUBCASE("can use add_sequence_front")
  {
    using seq_t = std::make_integer_sequence<size_t, 3>;
    auto seq = add_sequence_front_t<100, seq_t>{};
    auto val = get(seq, 0);
    CHECK_EQ(val, 100);
  }
}

TEST_SUITE_END;

//----------------------------------------------------

#pragma clang diagnostic pop