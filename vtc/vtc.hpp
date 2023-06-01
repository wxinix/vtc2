/*!
  C++ Virtualization Library of Traffic Cabinet 2
  Copyright (c) Wuping Xin
  MPL 1.1/GPL 2.0/LGPL 2.1 tri-license
*/

#ifndef VIRTUAL_TRAFFIC_CABINET_H_
#define VIRTUAL_TRAFFIC_CABINET_H_

#include <atomic>
#include <span>
#include <vector>
#include <vtc/traits.hpp>

namespace vtc {

/*!
 * Maximum number of load switch channels supported by each virtual cabinet.
 */
constexpr size_t MAX_LOAD_SWITCHES{32};

/*!
 * Maximum number of detector channels supported by each virtual cabinet.
 */
constexpr size_t MAX_VEHICLE_DETECTORS{128};

/*!
 * Maximum number of virtual cabinets supported by this framework.
 */
constexpr size_t MAX_CABINETS{16};

/*!
 * The buffer size of SDLC instance processor.
 */
constexpr size_t MAX_SDLC_FRAME_BYTES{64};

enum class Bit : bool
{
  Off = false,
  On = true
};

/*
 * 8 bit unsigned integer.
 */
using Byte = uint8_t;

/*
 * 16 bit unsigned integer.
 */
using Word = uint16_t;

/*
 * 32 bit unsigned integer.l
 */
using Cardinal = uint32_t;

/*
   Unsigned integer with its digits represented as 8 octets. For example,
   octets 0x30 ('0'), 0x31 ('1'), 0x32 ('2'), 0x33 ('3'), 0x34('4'), 0x35('5'),
   0x36('6'), 0x37('7'), when assigned to a OctetNumber, the numerical value
   is 1234567, with the leading 0 trimmed.

   We use 8 byte uint64_t as the internal storage, because it has trivial
   constructor and lock-free, good for thread-safe usage with std::atomic<T>.
 */
using OctetNumber = uint64_t;

template<typename T>
concept OctetNumberType = std::is_same_v<T, OctetNumber> || std::is_same_v<T, std::atomic<OctetNumber>>;

using Index = uint16_t;

template<Index I, size_t N>
concept ValidIndex = (I >= 1) && (I <= N);

using CabinetIndex = Index;

template<CabinetIndex Cabinet>
concept ValidCabinetIndex = ValidIndex<Cabinet, MAX_CABINETS>;

using DetectorChannel = Index;

template<DetectorChannel Ch>
concept ValidDetectorChannel = ValidIndex<Ch, MAX_VEHICLE_DETECTORS>;

using LoadSwitchChannel = Index;

template<LoadSwitchChannel Ch>
concept ValidLoadSwitchChannel = ValidIndex<Ch, MAX_LOAD_SWITCHES>;

using ControllerID = Cardinal;

/*!
 * ID of detection zone. Multiple detection zones can be wired to one detector unit.
 */
using DetectionZoneID = Cardinal;

using DetectionZoneIDs = std::vector<DetectionZoneID>;

/*!
 * ID of a signalized intersection approach. In TransModeler, this is the same
 * as "signal id" for the subject approach.
 */
using ApproachID = Cardinal;

/*!
 * ID of a turn movement that belongs to a specific approach. 0 means the rightmost
 * turn movement, and so on. For a typical three turn movements scenario, 0
 * represents right turn movement, 1 represents through movement, and 2 represents
 * left turn movement. In rare case that an approach has 5 turn movements, then from
 * right to left, the ID is counted as 0, 1, 2, 3, 4, 5.
 */
using TurnID = Byte;

/*!
 * A turn movement of an approach.
 */
using Movement = std::tuple<ApproachID, TurnID>;

/*!
 * A signal group refers to a specific movement or set of movements of vehicles
 * at an intersection that can be controlled by a load switch output.
 */
using SignalGroupID = Byte;
using SignalGroup = std::tuple<SignalGroupID, std::vector<Movement>>;

template<typename T>
std::tuple<bool, size_t> octets_to_uint32(const T &)
{
  return {false, 0xFFFFFFFF};
}

template<OctetNumberType T>
std::tuple<bool, uint32_t> octets_to_uint32(const T &octets)
{
  uint32_t value{0};
  bool is_valid{false};
  const char *bytes = reinterpret_cast<const char *>(&octets);

  for (int i = sizeof(OctetNumber) - 1; i >= 0; i--) {
    if (bytes[i] == 0) {
      if ((i < sizeof(OctetNumber) - 1) && (i > 0) && (bytes[i + 1] != 0) && (bytes[i - 1] != 0)) {
        is_valid = false;
        break;
      } else {
        continue;
      }
    } else if (bytes[i] >= '0' && bytes[i] <= '9') {
      is_valid = true;
      value = value * 10 + (bytes[i] - '0');
    } else {
      is_valid = false;
      break;
    }
  }

  if (!is_valid) {
    value = 0xFFFFFFFF;
  }

  return {is_valid, value};
}

enum class SdlcStationDeviceKind
{
  Mmu [[maybe_unused]],
  Biu,
  Cu [[maybe_unused]],
  Rack,
};

struct AbstractSdlcStationDevice
{
  virtual ~AbstractSdlcStationDevice() = default;

  /*!
   * Used for situation where controller unit propels the process loop in real-time:
   * CU sends command instance first, and secondary station responds. This is typical
   * of a hardware-in-the-loop simulation.
   * @param data_in The command frame from controller unit.
   * @return
   */
  virtual std::tuple<bool, std::span<const Byte>> Dispatch(std::span<const Byte> data_in) = 0;

  /*!
   * Used for situation where a traffic simulator propels the process.
   * @param frame_id
   * @return
   */
  virtual std::tuple<bool, std::span<const Byte>> GenerateResponseFrame(Byte frame_id) = 0;

  /*!
   * Used for situation where a traffic simulator propels the process.
   * @param data
   * @return
   */
  virtual bool ProcessCommandFrame(std::span<const Byte> data) = 0;

  /*!
   * Reset the underlying I/O variables.
   */
  virtual void Reset() = 0;

  [[nodiscard]] virtual Index cabinet() const = 0;
  [[nodiscard]] virtual SdlcStationDeviceKind sdlc_station_device_kind() const = 0;
};

template<typename Derived>
struct SdlcStationDevice : public AbstractSdlcStationDevice
{
  std::tuple<bool, std::span<const Byte>> Dispatch(std::span<const Byte> data_in) final
  {
    return static_cast<Derived *>(this)->DoDispatch(data_in);
  }

  std::tuple<bool, std::span<const Byte>> GenerateResponseFrame(Byte frame_id) final
  {
    return static_cast<Derived *>(this)->DoGenerateResponseFrame(frame_id);
  }

  bool ProcessCommandFrame(const std::span<const Byte> data) final
  {
    return static_cast<Derived *>(this)->DoProcessCommandFrame(data);
  }

  void Reset() final
  {
    static_cast<Derived *>(this)->DoReset();
  }
};

namespace io {

template<typename T>
concept ValidVariableValueType = std::disjunction_v<
    std::is_same<T, Bit>,
    std::is_same<T, Byte>,
    std::is_same<T, Word>,
    std::is_same<T, Cardinal>,
    std::is_same<T, OctetNumber>>;

struct VariableType
{
};

enum class IoBinding
{
  Fio,
  Mmu,
  Cu
};

template<CabinetIndex Cabinet, IoBinding Binding, ValidVariableValueType ValueT, Index VariableIndex>
  requires std::atomic<ValueT>::is_always_lock_free
struct Variable
{
  using value_t = ValueT;
  using type = VariableType;

  Variable() = default;
  Variable(Variable &) = delete;
  Variable(Variable &&) = delete;
  Variable &operator=(Variable &) = delete;
  Variable &operator=(Variable &&) = delete;

  static constexpr auto index{VariableIndex};
  static constexpr auto cabinet{Cabinet};
  static constexpr auto binding{Binding};

  std::atomic<ValueT> value{};
};

template<typename T>
using VariableValueType = typename T::Variable::value_t;

template<CabinetIndex Cabinet, IoBinding Binding, typename ValueT>
struct SimpleVariable : Variable<Cabinet, Binding, ValueT, 0>
{
};

template<typename T>
  requires std::is_same_v<typename T::Variable::type, VariableType>
T instance{};

template<CabinetIndex Cabinet, IoBinding Binding, DetectorChannel Ch>
  requires ValidDetectorChannel<Ch>
struct VehicleDetCall : Variable<Cabinet, Binding, Bit, Ch>
{
};

/*!
 The start of the simulation time in Linux timestamp format (i.e., Unix/POSIX
 timestamp), based on UTC. The value represents points in time as the number
 of seconds elapsed since the Unix epoch Jan 1, 1970 at 00:00:00 UTC.

 Note: Only the UTC time from the first message received by the external
 controller software (ECS) is used. The first message sets the time and clock
 in ECS that will be used by the its simulation.
 */
template<IoBinding Binding>
struct SimulationStartTime : SimpleVariable<0, Binding, Cardinal>
{
};

template<CabinetIndex Cabinet, IoBinding Binding, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
struct ChannelRedDoNotWalkDriver : Variable<Cabinet, Binding, Bit, Ch>
{
};

template<CabinetIndex Cabinet, IoBinding Binding, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
struct ChannelYellowPedClearDriver : Variable<Cabinet, Binding, Bit, Ch>
{
};

template<CabinetIndex Cabinet, IoBinding Binding, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
struct ChannelGreenWalkDriver : Variable<Cabinet, Binding, Bit, Ch>
{
};

template<CabinetIndex Cabinet, IoBinding Binding>
struct IntersectionID : SimpleVariable<Cabinet, Binding, OctetNumber>
{
  uint32_t to_uint32()
  {
    auto [valid, value] = octets_to_uint32(this->value);
    return valid ? value : 0xFFFFFFFF;
  }
};

}// namespace io

namespace frame {

namespace detail {

struct FrameElementType
{
};

template<typename T, size_t BitPos>
  requires std::is_same_v<io::VariableValueType<T>, Bit>
struct FrameBit
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> data_in)
  {
    static auto bytepos = pos / 8;
    static auto nbits_to_shift = pos % 8;
    auto value = (data_in[bytepos] & (0x01 << nbits_to_shift)) != 0;
    ref_var.value = static_cast<Bit>(value);
  }

  void operator>>(const std::span<Byte> data_out)
  {
    static auto byte_pos = pos / 8;
    static auto num_of_bits_to_shift = pos % 8;
    Byte i = (ref_var.value == Bit::On) ? 1 : 0;
    data_out[byte_pos] = data_out[byte_pos] | (i << num_of_bits_to_shift);
  }

  size_t pos{BitPos};
  T &ref_var{io::instance<T>};
};

template<typename T, size_t BytePos>
  requires std::is_same_v<io::VariableValueType<T>, Byte>
struct FrameByte
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> data_in)
  {
    ref_var.value = data_in[pos];
  }

  void operator>>(const std::span<Byte> data_out)
  {
    data_out[pos] = ref_var.value;
  }

  size_t pos{BytePos};
  T &ref_var{io::instance<T>};
};

template<typename T, size_t BytePos> /* BytePos: position of Word value low byte */
  requires std::is_same_v<io::VariableValueType<T>, Word>
struct FrameWord
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> data_in)
  {
    ref_var.value = data_in[pos] | (data_in[pos + 1] << 8);// LByte | HByte
  }

  void operator>>(const std::span<Byte> data_out)
  {
    data_out[pos] = ref_var.value & 0xFF;
    data_out[pos + 1] = ref_var.value >> 8 & 0xFF;
  }

  size_t pos{BytePos};
  T &ref_var{io::instance<T>};
};

template<typename T, size_t BytePos> /* BytePos: position of value the least significant byte */
  requires std::is_same_v<io::VariableValueType<T>, Cardinal>
struct FrameCardinal
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> data_in)
  {
    ref_var.value = data_in[pos]
        | data_in[pos + 1] << 8
        | data_in[pos + 2] << 16
        | data_in[pos + 3] << 24;
  }

  void operator>>(const std::span<Byte> data_out)
  {
    data_out[pos] = ref_var.value & 0xFF;
    data_out[pos + 1] = ref_var.value >> 0x08 & 0xFF;
    data_out[pos + 2] = ref_var.value >> 0x10 & 0xFF;
    data_out[pos + 3] = ref_var.value >> 0x18 & 0xFF;
  }

  size_t pos{BytePos};
  T &ref_var{io::instance<T>};
};

template<typename T, size_t BytePos> /* BytePos: position of OctetString starting byte */
  requires std::is_same_v<io::VariableValueType<T>, OctetNumber>
struct FrameOctetNumber
{
  using type = FrameElementType;

  void operator<<(const std::span<const Byte> data_in)
  {
    OctetNumber value{0};

    for (auto i = 0; i < sizeof(OctetNumber); i++) {
      value = value | static_cast<uint64_t>(data_in[pos + i]) << 8 * (sizeof(OctetNumber) - 1 - i);
    }

    ref_var.value = value;
  }

  void operator>>(const std::span<Byte> data_out)
  {
    for (auto i = 0; i < sizeof(OctetNumber); i++) {
      data_out[pos + i] = ref_var.value >> 8 * (sizeof(OctetNumber) - 1 - i);
    }
  }

  size_t pos{BytePos};
  T &ref_var{io::instance<T>};
};

template<size_t N>
concept ValidFrameByteSize = (N >= 1) && (N <= MAX_SDLC_FRAME_BYTES);

template<typename T>
concept ValidFrameElement = std::is_same_v<typename T::type, FrameElementType>;

template<size_t ByteSize, typename... Ts>
concept ValidFrame = (ValidFrameElement<Ts> && ...) && ValidFrameByteSize<ByteSize>;

enum class FrameKind
{
  Command,
  Response
};

/*
  -------------------------------------------------------------------
  FrameID                   Function
  -------------------------------------------------------------------
  000-063          Command Frame, Defined by TS2
  064-127          Command Frame, Manufacturer's Use
  128-191          Response Frame, Defined by TS2
  192-255          Response Frame, Manufacturer's Use
  -------------------------------------------------------------------
 */
template<Byte Address,
         Byte FrameID,
         size_t FrameByteSize,
         io::IoBinding Binding,
         typename... Ts>
  requires ValidFrame<FrameByteSize, Ts...>
class Frame
{
public:
  Frame() noexcept = default;
  Frame(Frame &) = delete;
  Frame(Frame &&) = delete;
  Frame &operator=(Frame &) = delete;
  Frame &operator=(Frame &&) = delete;

  void operator<<(const std::span<const Byte> data_in)
  {
    static_assert(((Binding != io::IoBinding::Cu) and (FrameID <= 127))
                  or /**/
                  ((Binding == io::IoBinding::Cu) and (FrameID >= 128)));

    Assign(data_in);
  }

  void operator>>(std::span<Byte> data_out)
  {
    static_assert(((Binding == io::IoBinding::Cu) and (FrameID <= 127))
                  or /**/
                  ((Binding != io::IoBinding::Cu) and (FrameID >= 128)));

    std::fill(data_out.begin(), data_out.end(), 0);
    data_out[0] = address;
    data_out[1] = 0x83;
    data_out[2] = id;
    Generate(data_out);
  }
  /*!
   * Reset the underlying I/O variables.
   */
  void Reset()
  {
    std::array<const Byte, MAX_SDLC_FRAME_BYTES> data_in{};
    Assign(data_in);
  }

  static constexpr Byte address{Address};
  static constexpr Byte id{FrameID};
  static constexpr size_t bytesize{FrameByteSize};

  /*!
   * NEMA-TS2: CommandFrame ID is [0,127], ResponseFrame [128,255]
   *
   * CommandFrame  : 000 - 063 NEMA-TS2 Use
   *                 064 - 127 Manufacture Use
   *
   * ResponseFrame:  128-191 NEMA-TS2 Use
   *                 192-255 Manufacture Use
   *
   * @return FrameKind
   */
  static constexpr FrameKind kind()
  {
    return (FrameID <= 127) ? FrameKind::Command : FrameKind::Response;
  }

private:
  template<size_t I = 0>
  inline void Assign(const std::span<const Byte> data_in)
  {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(frame_elements_) << data_in;
      Assign<I + 1>(data_in);
    }
  }

  template<size_t I = 0>
  inline void Generate(std::span<Byte> data_out)
  {
    if constexpr (I < sizeof...(Ts)) {
      std::get<I>(frame_elements_) >> data_out;
      Generate<I + 1>(data_out);
    }
  }

  std::tuple<Ts...> frame_elements_;
};

template<Byte Address,
         Byte FrameID,
         size_t FrameByteSize,
         io::IoBinding Binding,
         typename... Ts>
struct FrameGenerator
{
};

template<Byte Address,
         Byte FrameID,
         size_t FrameByteSize,
         io::IoBinding Binding,
         typename... Ts>
struct FrameGenerator<Address, FrameID, FrameByteSize, Binding, std::tuple<Ts...>>
{
  using type = Frame<Address, FrameID, FrameByteSize, Binding, Ts...>;
};

//-----------------------------------------------------------------
// Detector Call Data Frame Bits Generator
//-----------------------------------------------------------------

template<CabinetIndex Cabinet, io::IoBinding Binding, int N>
struct CallDataFrameBit
{
  using type = FrameBit<
      io::VehicleDetCall<Cabinet, Binding, N>,
      24 + N - 1>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, int N, typename... Ts>
struct CallDataFrameBitGenerator
{
  using type = typename CallDataFrameBitGenerator<
      Cabinet,
      Binding,
      N - 1,
      typename CallDataFrameBit<Cabinet, Binding, N>::type,
      Ts...>::type;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, typename... Ts>
struct CallDataFrameBitGenerator<Cabinet, Binding, 0, Ts...>
{
  using type = std::tuple<Ts...>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding>
using CallDataFrameBits = typename CallDataFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_VEHICLE_DETECTORS>::type;

//------------------------------------------------------------------
// Green Channel Driver Generator
//------------------------------------------------------------------

template<CabinetIndex Cabinet, io::IoBinding Binding, int N>
struct GreenChannelDriverFrameBit
{
  using type = FrameBit<
      io::ChannelGreenWalkDriver<Cabinet, Binding, N>,
      26 + 3 * (N - 1)>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, int N, typename... Ts>
struct GreenChannelDriverFrameBitGenerator
{
  using type = typename GreenChannelDriverFrameBitGenerator<
      Cabinet,
      Binding,
      N - 1,
      typename GreenChannelDriverFrameBit<Cabinet, Binding, N>::type,
      Ts...>::type;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, typename... Ts>
struct GreenChannelDriverFrameBitGenerator<Cabinet, Binding, 0, Ts...>
{
  using type = std::tuple<Ts...>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding>
using GreenChannelDriverFrameBits = typename GreenChannelDriverFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_LOAD_SWITCHES>::type;

//------------------------------------------------------------------
// Yellow Channel Driver Generator
//------------------------------------------------------------------

template<CabinetIndex Cabinet, io::IoBinding Binding, int N>
struct YellowChannelDriverFrameBit
{
  using type = FrameBit<
      io::ChannelYellowPedClearDriver<Cabinet, Binding, N>,
      25 + 3 * (N - 1)>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, int N, typename... Ts>
struct YellowChannelDriverFrameBitGenerator
{
  using type = typename YellowChannelDriverFrameBitGenerator<
      Cabinet,
      Binding,
      N - 1,
      typename YellowChannelDriverFrameBit<Cabinet, Binding, N>::type,
      Ts...>::type;
};

template<Index CuIndex, io::IoBinding Binding, typename... Ts>
struct YellowChannelDriverFrameBitGenerator<CuIndex, Binding, 0, Ts...>
{
  using type = std::tuple<Ts...>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding>
using YellowChannelDriverFrameBits = typename YellowChannelDriverFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_LOAD_SWITCHES>::type;

//------------------------------------------------------------------
// Red Channel Driver Generator
//------------------------------------------------------------------

template<CabinetIndex Cabinet, io::IoBinding Binding, int N>
struct RedChannelDriverFrameBit
{
  using type = FrameBit<
      io::ChannelRedDoNotWalkDriver<Cabinet, Binding, N>,
      24 + 3 * (N - 1)>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, int N, typename... Ts>
struct RedChannelDriverFrameBitGenerator
{
  using type = typename RedChannelDriverFrameBitGenerator<
      Cabinet,
      Binding,
      N - 1,
      typename RedChannelDriverFrameBit<Cabinet, Binding, N>::type,
      Ts...>::type;
};

template<CabinetIndex Cabinet, io::IoBinding Binding, typename... Ts>
struct RedChannelDriverFrameBitGenerator<Cabinet, Binding, 0, Ts...>
{
  using type = std::tuple<Ts...>;
};

template<CabinetIndex Cabinet, io::IoBinding Binding>
using RedChannelDriverFrameBits = typename RedChannelDriverFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_LOAD_SWITCHES>::type;

/*!
 * A FrameType concept that enforces that type must have a static constexpr method
 * called kind().
 * @tparam T
 */
template<typename T>
concept FrameType = requires {
  {
    T::kind()
  } -> std::same_as<FrameKind>;
};

template<typename T>
concept CommandFrameType = requires {
  {
    T::kind()
  } -> std::same_as<FrameKind>;

  T::kind() == FrameKind::Command;
};

template<typename T>
concept ResponseFrameType = requires {
  {
    T::kind()
  } -> std::same_as<FrameKind>;

  T::kind() == FrameKind::Response;
};

template<CommandFrameType T1, ResponseFrameType T2>
using FrameMap = std::tuple<T1 &, T2 &>;

template<typename... Ts>
using FrameMaps = std::tuple<Ts...>;

}// namespace detail

/*!
 * A trick for creating singleton instance of a instance type.
 * @tparam T The type of the instance.
 */
template<detail::FrameType T>
T instance{};

/*!
 Type 64 Command Frame for Software in the Loop Simulation. It represents a command
 instance used in the initial handshake phase of a Software in the Loop (SIL) simulation.
 This instance is intended to be transmitted by the External Controller Software (ECS)
 to notify the traffic simulator that it is ready to start the simulation. It contains
 the ID of the target intersection within the simulation model, serving as a means to
 identify and establish communication between the ECS and the traffic simulator. By
 utilizing this instance, the ECS initiates the simulation process and signals its
 readiness to begin the desired traffic scenario.
 */
template<CabinetIndex Cabinet, io::IoBinding Binding>
  requires ValidCabinetIndex<Cabinet>
using SimulatorInitialHandshakeFrame = detail::Frame<
    254,// Simulator Address = 254
    64, // FrameID = 64
    11, // Total number of bytes of the instance
    Binding,
    // ----------------------------------------------
    // Byte 0 - Address, 0xFE for Simulator
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 64
    // ----------------------------------------------
    // Byte 3 - 10
    detail::FrameOctetNumber<io::IntersectionID<Cabinet, Binding>, 0x03>>;

template<CabinetIndex Cabinet, io::IoBinding Binding>
  requires ValidCabinetIndex<Cabinet>
using SimulatorInitialHandshakeAckFrame = detail::Frame<
    254,// Simulator Address = 254
    192,// FrameID = 192
    3,  // Total number of bytes of the instance
    Binding
    // ----------------------------------------------
    // Byte 0 - Address, 0xFE for Simulator
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 192
    // ----------------------------------------------
    >;

template<CabinetIndex Cabinet, io::IoBinding Binding>
  requires ValidCabinetIndex<Cabinet>
using SimulatorCallRequestFrame = detail::Frame<
    254,// Simulator Address = 254
    65, // FrameID = 65
    3,  // Total number of bytes of the instance
    Binding
    // ----------------------------------------------
    // Byte 0 - Address, 0xFE for Simulator
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 65
    // ----------------------------------------------
    >;

template<CabinetIndex Cabinet, io::IoBinding Binding>
  requires ValidCabinetIndex<Cabinet>
using SimulatorCallDataFrame = detail::FrameGenerator<
    254,// Simulator Address = 254
    193,// FrameID = 193
    23, // Total number of bytes of the instance
    Binding,
    // ----------------------------------------------
    // Byte 0 - Address, 0xFE for Simulator
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 193
    // ----------------------------------------------
    append_tuple_t<
        // Byte 3 - 18
        detail::CallDataFrameBits<Cabinet, Binding>,
        // Byte 19 - 22
        detail::FrameCardinal<io::SimulationStartTime<Binding>, 19>>>::type;

template<CabinetIndex Cabinet, io::IoBinding Binding>
  requires ValidCabinetIndex<Cabinet>
using SimulatorLoadSwitchDriversFrame = detail::FrameGenerator<
    254,// Simulator Address = 254
    66, // FrameID = 66
    19, // Total number of bytes of the instance
    Binding,
    // ----------------------------------------------
    // Byte 0 - Address, 254
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 66
    // ----------------------------------------------
    concatenate_tuples_t<
        // Byte 3 - 14
        detail::RedChannelDriverFrameBits<Cabinet, Binding>,
        detail::YellowChannelDriverFrameBits<Cabinet, Binding>,
        detail::GreenChannelDriverFrameBits<Cabinet, Binding>>
    // Byte 15, 16, 17, 18 are unused.
    >::type;

template<CabinetIndex Cabinet, io::IoBinding Binding>
  requires ValidCabinetIndex<Cabinet>
using SimulatorLoadSwitchDriversAckFrame = detail::Frame<
    254,// Simulator Address = 254
    194,// FrameID = 194
    3,  // Total number of bytes of the instance
    Binding
    // ----------------------------------------------
    // Byte 0 - Address, 0xFE for Simulator
    // Byte 1 - Control, always 0x83
    // Byte 2 - FrameID, 194
    // ----------------------------------------------
    >;

}// namespace frame

namespace biu {

struct BiuType
{
};

template<typename Derived>
struct Biu : public SdlcStationDevice<Biu<Derived>>
{
public:
  friend class SdlcStationDevice<Biu<Derived>>;

  [[nodiscard]] CabinetIndex cabinet() const final
  {
    return Derived::cabinet;
  }

  [[nodiscard]] SdlcStationDeviceKind sdlc_station_device_kind() const final
  {
    return SdlcStationDeviceKind::Biu;
  }

private:
  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoGenerateResponseFrame(Byte frame_id)
  {
    if constexpr (I < Derived::frame_maps_size) {
      auto &res_frame = std::get<1>(std::get<I>(static_cast<Derived *>(this)->frame_maps));
      if (res_frame.id == frame_id) {
        res_frame >> this->buffer_;
        return {true, {this->buffer_.data(), res_frame.bytesize}};
      } else {
        return DoGenerateResponseFrame<I + 1>(frame_id);
      }
    } else {
      return {false, {this->buffer_.data(), 0}};
    }
  }

  template<size_t I = 0>
  bool DoProcessCommandFrame(const std::span<const Byte> data)
  {
    if constexpr (I < Derived::frame_maps_size) {
      auto &cmd_frame = std::get<0>(std::get<I>(static_cast<Derived *>(this)->frame_maps));
      if (cmd_frame.id == data[2]) {
        cmd_frame << data;
        return true;
      } else {
        return DoProcessCommandFrame<I + 1>(data);
      }
    } else {
      return false;
    }
  }

  template<size_t I = 0>
  void DoReset()
  {
    if constexpr (I < Derived::frame_maps_size) {
      auto &cmd_frame = std::get<0>(std::get<I>(static_cast<Derived *>(this)->frame_maps));
      auto &res_frame = std::get<1>(std::get<I>(static_cast<Derived *>(this)->frame_maps));
      cmd_frame.Reset();
      res_frame.Reset();
      return DoReset<I + 1>();
    } else {
      return;
    }
  }

  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoDispatch(std::span<const Byte> data_in)
  {
    if constexpr (I < Derived::frame_maps_size) {
      auto &cmd_frame = std::get<0>(std::get<I>(static_cast<Derived *>(this)->frame_maps));
      auto &res_frame = std::get<1>(std::get<I>(static_cast<Derived *>(this)->frame_maps));

      if (cmd_frame.id == data_in[2]) {
        cmd_frame << data_in;
        res_frame >> this->buffer_;// Buffer will be emptied at the beginning of >>().
        return {true, {this->buffer_.data(), res_frame.bytesize}};
      } else {
        return DoDispatch<I + 1>(data_in);
      }
    } else {
      return {false, {this->buffer_.data(), 0}};
    }
  }

private:
  std::array<Byte, MAX_SDLC_FRAME_BYTES> buffer_{};
};

template<CabinetIndex Cabinet, Byte FrameID>
struct FrameType
{
};

template<CabinetIndex Cabinet>
struct FrameType<Cabinet, 64>
{
  using type = frame::SimulatorInitialHandshakeFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct FrameType<Cabinet, 192>
{
  using type = frame::SimulatorInitialHandshakeAckFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct FrameType<Cabinet, 65>
{
  using type = frame::SimulatorCallRequestFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct FrameType<Cabinet, 193>
{
  using type = frame::SimulatorCallDataFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct FrameType<Cabinet, 66>
{
  using type = frame::SimulatorLoadSwitchDriversFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct FrameType<Cabinet, 194>
{
  using type = frame::SimulatorLoadSwitchDriversAckFrame<Cabinet, io::IoBinding::Fio>;
};

template<typename T>
  requires std::is_same_v<typename T::type, BiuType>
T instance{};

/*!
 * A bus interface unit for traffic simulator software such as TransModeler.
 * @tparam CuIndex Controller unit index.
 */
template<CabinetIndex Cabinet>
  requires ValidCabinetIndex<Cabinet>
struct SimulatorBiu : public Biu<SimulatorBiu<Cabinet>>
{
  using type = BiuType;

  using SimulatorBiuFrameMaps = frame::detail::FrameMaps<
      frame::detail::FrameMap<typename FrameType<Cabinet, 64>::type,
                              typename FrameType<Cabinet, 192>::type>,
      frame::detail::FrameMap<typename FrameType<Cabinet, 65>::type,
                              typename FrameType<Cabinet, 193>::type>,
      frame::detail::FrameMap<typename FrameType<Cabinet, 66>::type,
                              typename FrameType<Cabinet, 194>::type>>;

  const SimulatorBiuFrameMaps frame_maps{
      std::make_tuple(
          std::make_tuple(std::ref(frame::instance<typename FrameType<Cabinet, 64>::type>),
                          std::ref(frame::instance<typename FrameType<Cabinet, 192>::type>)),
          std::make_tuple(std::ref(frame::instance<typename FrameType<Cabinet, 65>::type>),
                          std::ref(frame::instance<typename FrameType<Cabinet, 193>::type>)),
          std::make_tuple(std::ref(frame::instance<typename FrameType<Cabinet, 66>::type>),
                          std::ref(frame::instance<typename FrameType<Cabinet, 194>::type>)))};

  constexpr static auto frame_maps_size{std::tuple_size_v<SimulatorBiuFrameMaps>};
  constexpr static auto cabinet{Cabinet};
};

}// namespace biu

namespace aux {// NEMA-TS2 Auxiliary Devices

template<CabinetIndex Cabinet, typename Factory, typename T, T... Is>
auto make_wirings(Factory, std::integer_sequence<T, Is...>)
{
  return std::make_tuple(Factory::template make<Cabinet, Is>()...);
}

/*!
 * Provide a convenience function for enumerating the individual wiring of a list of wirings.
 * @tparam WiringsT The wirings type.
 * @tparam F The callback functor type to be applied to each subject individual wiring.
 * @param wirings
 * @param func
 */
template<typename WiringsT, typename F>
void for_each(WiringsT &&wirings, F &&func)
{
  std::apply(
      [&func]<typename... T>(T &&...args) {
        (func(std::forward<T>(args)), ...);
      },
      std::forward<WiringsT>(wirings));
}

namespace lsw {// Load Switch

namespace detail {

using namespace vtc::io;

template<CabinetIndex Cabinet, Index Ch>
  requires ValidCabinetIndex<Cabinet> && ValidLoadSwitchChannel<Ch>
using LoadSwitchDriver /**/
    = std::tuple<
        io::ChannelGreenWalkDriver<Cabinet, IoBinding::Fio, Ch> &,
        io::ChannelYellowPedClearDriver<Cabinet, IoBinding::Fio, Ch> &,
        io::ChannelRedDoNotWalkDriver<Cabinet, IoBinding::Fio, Ch> &>;

template<CabinetIndex Cabinet, Index Ch>
auto make_load_switch_driver()
{
  return std::make_tuple(
      std::ref(instance<ChannelGreenWalkDriver<Cabinet, IoBinding::Fio, Ch>>),      /**/
      std::ref(instance<ChannelYellowPedClearDriver<Cabinet, IoBinding::Fio, Ch>>), /**/
      std::ref(instance<ChannelRedDoNotWalkDriver<Cabinet, IoBinding::Fio, Ch>>));  /**/
}

}// namespace detail

using LoadSwitchChannels =
    offset_sequence_t<
        0,
        std::make_integer_sequence<
            LoadSwitchChannel,
            MAX_LOAD_SWITCHES>>;

enum class LoadSwitchState : short
{
  Blank = 0,
  Red = 1,
  Yellow = 2,
  Green = 3
};

struct AbstractLoadSwitch
{
public:
  virtual ~AbstractLoadSwitch() = default;

  [[nodiscard]] virtual LoadSwitchState state() const noexcept = 0;
  [[nodiscard]] virtual LoadSwitchState operator()() const noexcept = 0;
  virtual void set_state(LoadSwitchState value) noexcept = 0;

  [[nodiscard]] virtual CabinetIndex cabinet() const noexcept = 0;
  [[nodiscard]] virtual LoadSwitchChannel channel() const noexcept = 0;
};

template<typename Derived>
struct LoadSwitchBase : public AbstractLoadSwitch
{
  [[nodiscard]] LoadSwitchState state() const noexcept final
  {
    auto &[g, y, r] = static_cast<const Derived *>(this)->driver;
    auto gyr_value = std::make_tuple(std::ref(g.value), std::ref(y.value), std::ref(r.value));
    LoadSwitchState result;

    if (gyr_value == std::make_tuple(Bit::On, Bit::Off, Bit::Off)) {
      result = LoadSwitchState::Green;
    } else if (gyr_value == std::make_tuple(Bit::Off, Bit::On, Bit::Off)) {
      result = LoadSwitchState::Yellow;
    } else if (gyr_value == std::make_tuple(Bit::Off, Bit::Off, Bit::On)) {
      result = LoadSwitchState::Red;
    } else {
      result = LoadSwitchState::Blank;
    }

    return result;
  }

  [[nodiscard]] LoadSwitchState operator()() const noexcept final
  {
    return state();
  }

  void set_state(LoadSwitchState value) noexcept final
  {
    auto &[g, y, r] = static_cast<const Derived *>(this)->driver;
    g.value = Bit::Off;
    r.value = Bit::Off;
    y.value = Bit::Off;

    switch (value) {
      case LoadSwitchState::Green:
        g.value = Bit::On;
        break;

      case LoadSwitchState::Yellow:
        y.value = Bit::On;
        break;

      case LoadSwitchState::Red:
        r.value = Bit::On;
        break;

      default:
        break;
    }
  }

  [[nodiscard]] CabinetIndex cabinet() const noexcept final
  {
    return Derived::cabinet;
  }

  [[nodiscard]] LoadSwitchChannel channel() const noexcept final
  {
    return Derived::channel;
  }
};

template<CabinetIndex Cabinet, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch> && ValidCabinetIndex<Cabinet>
struct LoadSwitch : public LoadSwitchBase<LoadSwitch<Cabinet, Ch>>
{
  static constexpr auto channel{Ch};
  static constexpr auto cabinet{Cabinet};
  const detail::LoadSwitchDriver<Cabinet, Ch> driver{
      detail::make_load_switch_driver<Cabinet, Ch>()};
};

template<CabinetIndex Cabinet, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
using LoadSwitchWiring = std::tuple<LoadSwitch<Cabinet, Ch> &, SignalGroup>;

template<typename T>
  requires std::is_base_of_v<AbstractLoadSwitch, T>
T instance{};

struct LoadSwitchWiringFactory
{
  template<CabinetIndex Cabinet, LoadSwitchChannel Ch>
  static auto make()
  {
    return std::make_tuple(std::ref(instance<LoadSwitch<Cabinet, Ch>>), SignalGroup{});
  }
};

template<CabinetIndex Cabinet>
using LoadSwitchWirings =
    decltype(make_wirings<Cabinet>(LoadSwitchWiringFactory{}, LoadSwitchChannels{}));

}// namespace lsw

namespace du {// detector unit

using DetectorChannels =
    offset_sequence_t<
        0,
        std::make_integer_sequence<
            DetectorChannel,
            MAX_VEHICLE_DETECTORS>>;

struct AbstractDetectorUnit
{
public:
  virtual ~AbstractDetectorUnit() = default;

  [[nodiscard]] virtual bool activated() const noexcept = 0;
  [[nodiscard]] virtual bool operator()() const noexcept = 0;
  virtual void set_activated(bool value) noexcept = 0;

  [[nodiscard]] virtual CabinetIndex cabinet() const noexcept = 0;
  [[nodiscard]] virtual DetectorChannel channel() const noexcept = 0;
};

template<typename Derived>
struct DetectorUnitBase : public AbstractDetectorUnit
{
  [[nodiscard]] bool activated() const noexcept final
  {
    return static_cast<const Derived *>(this)->state.value == Bit::On;
  }

  [[nodiscard]] bool operator()() const noexcept final
  {
    return activated();
  }

  [[nodiscard]] CabinetIndex cabinet() const noexcept final
  {
    return Derived::cabinet;
  }

  [[nodiscard]] DetectorChannel channel() const noexcept final
  {
    return Derived::channel;
  }

  void set_activated(bool value) noexcept final
  {
    static_cast<const Derived *>(this)->state.value = value ? Bit::On : Bit::Off;
  }
};

template<CabinetIndex Cabinet, DetectorChannel Ch>
  requires ValidDetectorChannel<Ch> && ValidCabinetIndex<Cabinet>
struct DetectorUnit : public DetectorUnitBase<DetectorUnit<Cabinet, Ch>>
{
  static constexpr auto channel{Ch};
  static constexpr auto cabinet{Cabinet};

  io::VehicleDetCall<Cabinet, io::IoBinding::Fio, Ch>
      &state{io::instance<io::VehicleDetCall<Cabinet, io::IoBinding::Fio, Ch>>};
};

template<CabinetIndex Cabinet, DetectorChannel Ch>
  requires ValidDetectorChannel<Ch> && ValidCabinetIndex<Cabinet>
using DetectorWiring = std::tuple<DetectorUnit<Cabinet, Ch> &, DetectionZoneIDs>;

template<typename T>
  requires std::is_base_of_v<AbstractDetectorUnit, T>
T instance{};

struct DetectorWiringFactory
{
  template<CabinetIndex Cabinet, DetectorChannel Ch>
  static auto make()
  {
    return std::make_tuple(std::ref(instance<DetectorUnit<Cabinet, Ch>>), DetectionZoneIDs{});
  }
};

template<CabinetIndex Cabinet>
using DetectorWirings = decltype(make_wirings<Cabinet>(DetectorWiringFactory{}, DetectorChannels{}));

}// namespace du

}// namespace aux

namespace xils {// X in the loop simulation, x = software/hardware

struct IXilSimulator
{
  /*!
   * Get controller id for the given cabinet from the simulator. This id is defined by
   * the simulator.
   * @param cabinet
   * @return
   */
  virtual ControllerID GetControllerID(CabinetIndex cabinet) = 0;

  /*!
   * Retrieve sensor state, ON or OFF, of a given sensor from the simulator.
   * @param sensor_id
   * @return
   */
  virtual bool GetSensorState(DetectionZoneID sensor_id) = 0;

  /*!
   * Set signal state of a given signal in the simulator.
   * @param sg_id
   * @param approach_id
   * @param turn_id
   * @param state
   */
  virtual void SetSignalState(SignalGroupID sg_id,
                              ApproachID approach_id,
                              TurnID turn_id,
                              aux::lsw::LoadSwitchState state) = 0;

  /*!
   * Get the mapped sensor id in the simulator for the given detector channel.
   * @param id
   * @param ch
   * @return
   */
  virtual std::span<const DetectionZoneID> GetDetectionZoneIDs(ControllerID id, DetectorChannel ch) = 0;

  /*!
   * We don't want to use std::vector, so we alias a new type here.
   */
  using SignalGroupEx = std::tuple<SignalGroupID, std::span<Movement>>;

  /*!
   * Get the mapped signal group in the simulator for the given load switch channel.
   * @param id
   * @param ch
   * @return
   */
  virtual SignalGroupEx GetSignalGroup(ControllerID id, LoadSwitchChannel ch) = 0;
};

}// namespace xils

namespace rack {// biu rack

struct RackType
{
};

template<typename Derived, CabinetIndex Cabinet>
struct Rack : public SdlcStationDevice<Rack<Derived, Cabinet>>
{
public:
  friend class SdlcStationDevice<Rack<Derived, Cabinet>>;

  void ProcessDetectorWirings()
  {
    if (not m_simulator)
      return;

    aux::for_each(m_detector_wirings, [&](auto &&el) {
      auto &[detector, sensor_ids] = el;
      bool value = false;

      if (!sensor_ids.empty()) {
        for (const auto sensor_id : sensor_ids) {
          value = value or m_simulator->GetSensorState(sensor_id);
        }
        detector.set_activated(value);
      }
    });
  }

  void ProcessLoadSwitchWirings()
  {
    if (not m_simulator)
      return;

    aux::for_each(m_load_switch_wirings, [&](auto &&el) {
      auto &[lsw, sg] = el;
      auto &[sg_id, movements] = sg;

      if (not movements.empty()) {
        for (const auto &movement : movements) {
          const auto &[approach, turn] = movement;
          m_simulator->SetSignalState(0, approach, turn, lsw.state());
        }
      } else if (sg_id > 0) {
        m_simulator->SetSignalState(sg_id, 0, 0, lsw.state());
      }
    });
  }

  void SetSimulator(xils::IXilSimulator *simulator)
  {
    if (not simulator)
      throw std::runtime_error("Null pointer encountered while calling SimulatorBiuRack::SetSimulator.");

    m_simulator = simulator;
    m_controller_id = simulator->GetControllerID(Cabinet);

    aux::for_each(m_detector_wirings, [&](auto &&el) {
      auto &[du, zones] = el;
      auto l_zones = m_simulator->GetDetectionZoneIDs(m_controller_id, du.channel);
      zones.clear();

      for (int i = 0; i < l_zones.size(); i++) {
        zones.push_back(l_zones[i]);
      }
    });

    aux::for_each(m_load_switch_wirings, [&](auto &&el) {
      auto &[lsw, sg] = el;
      auto &[sg_id, movements] = sg;
      movements.clear();

      auto [l_sg_id, l_movements] = m_simulator->GetSignalGroup(m_controller_id, lsw.channel);
      sg_id = l_sg_id;

      for (int i = 0; i < l_movements.size(); i++) {
        movements.push_back(l_movements[i]);
      }
    });
  }

  [[nodiscard]] CabinetIndex cabinet() const final
  {
    return Cabinet;
  }

  [[nodiscard]] ControllerID controller_id() const
  {
    return m_controller_id;
  }

  [[nodiscard]] SdlcStationDeviceKind sdlc_station_device_kind() const final
  {
    return SdlcStationDeviceKind::Rack;
  }

private:
  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoGenerateResponseFrame(Byte frame_id)
  {
    if constexpr (I < Derived::rack_size) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->m_bius);
      auto result = biu.GenerateResponseFrame(frame_id);

      if (std::get<0>(result)) {
        return result;
      } else {
        return DoGenerateResponseFrame<I + 1>(frame_id);
      }
    } else {
      return {false, {}};
    }
  }

  template<size_t I = 0>
  bool DoProcessCommandFrame(const std::span<const Byte> data)
  {
    if constexpr (I < Derived::rack_size) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->m_bius);
      return biu.ProcessCommandFrame(data) || DoProcessCommandFrame<I + 1>(data);
    } else {
      return false;
    }
  }

  template<size_t I = 0>
  void DoReset()
  {
    if constexpr (I < Derived::rack_size) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->m_bius);
      return biu.Reset(), DoReset<I + 1>();
    } else {
      return;
    }
  }

  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoDispatch(std::span<const Byte> data)
  {
    if constexpr (I < Derived::rack_size) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->m_bius);
      auto result = biu.Dispatch(data);
      if (std::get<0>(result)) {
        return result;
      } else {
        return DoDispatch<I + 1>(data);
      }
    } else {
      return {false, {}};
    }
  }

private:
  aux::lsw::LoadSwitchWirings<Cabinet> m_load_switch_wirings{
      aux::make_wirings<Cabinet>(aux::lsw::LoadSwitchWiringFactory{},
                                 aux::lsw::LoadSwitchChannels{})};

  aux::du::DetectorWirings<Cabinet> m_detector_wirings{
      aux::make_wirings<Cabinet>(aux::du::DetectorWiringFactory{},
                                 aux::du::DetectorChannels{})};

  ControllerID m_controller_id{0};

  xils::IXilSimulator *m_simulator{nullptr};
};

template<typename T>
  requires std::is_same_v<typename T::type, RackType>
T instance{};

template<CabinetIndex Cabinet>
struct SimulatorBiuRack : Rack<SimulatorBiuRack<Cabinet>, Cabinet>
{
public:
  friend class Rack<SimulatorBiuRack<Cabinet>, Cabinet>;

  using type = RackType;
  using Bius = std::tuple<biu::SimulatorBiu<Cabinet> &>;

  constexpr static auto rack_size{std::tuple_size_v<Bius>};

private:
  const Bius m_bius{std::ref(biu::instance<biu::SimulatorBiu<Cabinet>>)};
};

/*

template<CabinetIndex Cabinet>
struct DetectorBiuRack : Rack<DetectorBiuRack<Cabinet>, Cabinet>
{
public:
  friend class Rack<DetectorBiuRack<Cabinet>, Cabinet>;

  using type = RackType;

  using Bius = std::tuple<
      biu::DetectorBiu<Cabinet, 1> &,
      biu::DetectorBiu<Cabinet, 2> &,
      biu::DetectorBiu<Cabinet, 3> &,
      biu::DetectorBiu<Cabinet, 4> &>;

  constexpr static auto rack_size{std::tuple_size_v<Bius>};

private:
  const Bius m_bius{std::ref(biu::instance<biu::DetectorBiu<Cabinet, 1>>),
                    std::ref(biu::instance<biu::DetectorBiu<Cabinet, 2>>),
                    std::ref(biu::instance<biu::DetectorBiu<Cabinet, 3>>),
                    std::ref(biu::instance<biu::DetectorBiu<Cabinet, 4>>)};
};

template<CabinetIndex Cabinet>
struct TFBiuRack : Rack<TFBiuRack<Cabinet>, Cabinet>
{
public:
  friend class Rack<TFBiuRack<Cabinet>, Cabinet>;

  using type = RackType;

  using Bius = std::tuple<
      biu::TFBiu<Cabinet, 1> &,
      biu::TFBiu<Cabinet, 2> &,
      biu::TFBiu<Cabinet, 3> &,
      biu::TFBiu<Cabinet, 4> &>;

  constexpr static auto rack_size{std::tuple_size_v<Bius>};

private:
  const Bius m_bius{std::ref(biu::instance<biu::TFBiu<Cabinet, 1>>),
                    std::ref(biu::instance<biu::TFBiu<Cabinet, 2>>),
                    std::ref(biu::instance<biu::TFBiu<Cabinet, 3>>),
                    std::ref(biu::instance<biu::TFBiu<Cabinet, 4>>)};
};

*/

}// namespace rack

}// namespace vtc

#endif//VIRTUAL_TRAFFIC_CABINET_H_