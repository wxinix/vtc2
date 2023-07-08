#ifndef VIRTUAL_TRAFFIC_CABINET_H_
#define VIRTUAL_TRAFFIC_CABINET_H_

#include <array>
#include <atomic>
#include <span>
#include <stdexcept>
#include <vector>
#include <vtc/traits.hpp>

namespace vtc {

/**
 * Maximum number of load switch channels supported by each virtual cabinet.
 */
inline constexpr size_t MAX_LOAD_SWITCHES{32};

/**
 * Maximum number of detector channels supported by each virtual cabinet.
 */
inline constexpr size_t MAX_VEHICLE_DETECTORS{128};

/**
 * Maximum number of virtual cabinets supported by this framework.
 */
inline constexpr size_t MAX_CABINETS{16};

/**
 * The buffer size of SDLC instance processor.
 */
inline constexpr size_t MAX_SDLC_FRAME_BYTES{64};

/**
 * A bit has two states: Off and On.
 */
enum class Bit : bool
{
  Off = false,
  On = true
};

/**
 * 8 bit unsigned integer.
 */
using Byte = uint8_t;

/**
 * 16 bit unsigned integer.
 */
using Word = uint16_t;

/**
 * 32 bit unsigned integer.l
 */
using Cardinal = uint32_t;

/**
 * Unsigned integer with its digits represented as 8 octets (chars). For example,
 * octets 0x30 ('0'), 0x31 ('1'), 0x32 ('2'), 0x33 ('3'), 0x34('4'), 0x35('5'),
 * 0x36('6'), 0x37('7') represents a numerical value 1234567, with the leading
 * '0' trimmed.
 * @remarks 8-byte uint64_t used as the internal storage, because it has trivial
 * constructor and lock-free, ready for std::atomic<T>.
 */
using OctetNumber = uint64_t;

template<typename T>
concept OctetNumberType = std::is_same_v<T, OctetNumber> || std::is_same_v<T, std::atomic<OctetNumber>>;

using Index = uint16_t;

template<Index I, size_t N>
concept ValidIndex = (I >= 1) && (I <= N);

/**
 * Index of a cabinet.
 */
using CabinetIndex = Index;

template<CabinetIndex Cabinet>
concept ValidCabinetIndex = ValidIndex<Cabinet, MAX_CABINETS>;

/**
 * ID of detector channel.
 */
using DetectorChannel = Index;

template<DetectorChannel Ch>
concept ValidDetectorChannel = ValidIndex<Ch, MAX_VEHICLE_DETECTORS>;

/**
 * ID of load switch channel.
 */
using LoadSwitchChannel = Index;

template<LoadSwitchChannel Ch>
concept ValidLoadSwitchChannel = ValidIndex<Ch, MAX_LOAD_SWITCHES>;

/**
 * ID of controller.
 */
using ControllerID = Cardinal;

/**
 * ID of detection zone. Multiple detection zones can be wired to one detector unit.
 */
using DetectionZoneID = Cardinal;

/**
 * Detection zone ID list.
 */
using DetectionZoneIDs = std::vector<DetectionZoneID>;

/**
 * ID of a signalized intersection approach. In TransModeler, this is equivalent
 * to "signal id" for the subject approach.
 */
using ApproachID = Cardinal;

/**
 * Index of a turn movement that belongs to a specific approach. 0 means the
 * rightmost one, and so on. For a typical three turn movements scenario, 0
 * represents right turn movement, 1 through movement, and 2 left turn.
 *
 * In the context of TransModeler, for signal movement index (that counted from
 * right to left), depending on the number of movements, the index of a particular
 * movement is not fixed.
 */
using TurnIndex = Byte;

/**
 * A turn movement of an approach.
 */
using Movement = std::tuple<ApproachID, TurnIndex>;

/**
 * ID of signal group.
 */
using SignalGroupID = Byte;

/**
 * A signal group refers to a specific movement or set of movements of vehicles
 * at an intersection that can be controlled by a load switch output. This class
 * includes a tuple of SignalGroupID and the associated list of movements.
 */
using SignalGroup = std::tuple<SignalGroupID, std::vector<Movement>>;

template<typename T>
std::tuple<bool, size_t> octets_to_uint32(const T &)
{
  return {false, 0xFFFFFFFF};
}

/**
 * Converts octet bytes (up to 8) to its literal uint32 value.
 * @param octets OctetNumberType using 8 bytes. The leading and trailing hex 0's
 * will be trimmed. Hex 0's are not allowed in between. "Significant" bytes must
 * be ASCII chars between '0' and '9'.
 * @return
 */
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

/**
 * Different kind of cabinet device.
 */
enum class DeviceKind
{
  /**
   * Malfunction Management Unit.
   */
  Mmu,

  /**
   * Bus Interface Unit.
   */
  Biu,

  /**
   * Controller Unit.
   */
  Cu,

  /**
   * Rack such as Detector Rack or BIU Rack.
   */
  Rack,
};

/**
 * Base class of SDLC secondary station device using Curiously Recurring
 * Template Pattern (CRTP).
 * @tparam Derived Derived class type.
 */
template<typename Derived>
struct SdlcSecondaryStationDevice
{
  /**
   * Parses a received command frame and immediately generates the corresponding
   * response frame.
   * @param data_in The command frame received from controller unit.
   * @return If success, returns the response frame to the command frame;
   * otherwise, an empty byte array.
   */
  std::tuple<bool, std::span<const Byte>> Dispatch(std::span<const Byte> data_in)
  {
    return static_cast<Derived *>(this)->DoDispatch(data_in);
  }

  /**
   * Generates a response frame given a frame id. The payload will be based on
   * the instantaneous states of the I/O variables bound to the device.
   * @param frame_id Id of the response frame to be generated.
   * @return If success, returns the response frame of the given id; otherwise,
   * and empty byte array.
   */
  std::tuple<bool, std::span<const Byte>> GenerateResponseFrame(Byte frame_id)
  {
    return static_cast<Derived *>(this)->DoGenerateResponseFrame(frame_id);
  }

  /**
   * Processes the command frame, and updates states of the I/O variables bound
   * to this device accordingly.
   * @param data The command frame to process.
   * @return True for success and false otherwise.
   */
  bool ProcessCommandFrame(const std::span<const Byte> data)
  {
    return static_cast<Derived *>(this)->DoProcessCommandFrame(data);
  }

  /**
   * Resets the underlying I/O variables that are bound to this device.
   */
  void Reset()
  {
    static_cast<Derived *>(this)->DoReset();
  }

  /**
   * Returns the cabinet index of the subject device.
   */
  [[nodiscard]] static consteval auto cabinet()
  {
    return Derived::get_cabinet();
  }

  /**
   * Returns the kind of the subject device.
   */
  [[nodiscard]] static consteval auto device_kind()
  {
    return Derived::get_device_kind();
  }

  /**
   * Returns the size of the frame maps.
   * @return
   */
  [[nodiscard]] static consteval auto frame_maps_size()
  {
    return Derived::get_frame_maps_size();
  }
};

namespace io {

/**
 * Verifies that a type is one of the allowed types for specifying IO variable value.
 * @tparam T The type to verify.
 */
template<typename T>
concept ValidIoVariableValueType = std::disjunction_v<
    std::is_same<T, Bit>,
    std::is_same<T, Byte>,
    std::is_same<T, Word>,
    std::is_same<T, Cardinal>,
    std::is_same<T, OctetNumber>>;

/**
 * Specifies the housing device to binds an IO variable to.
 */
enum class IoBinding
{
  Fio, /* Field IO */
  Mmu, /* Malfunction Management Unit */
  Cu   /* Controller Unit */
};

template<typename T>
concept IoVariableType = requires {
  {
    T::binding()
  } -> std::same_as<IoBinding>;

  ValidIoVariableValueType<typename T::IoVariable::value_t>;
};

/**
 * IO IoVariable class.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the variable.
 * @tparam ValueT Value type.
 * @tparam VariableIndex Index of the variable.
 */
template<CabinetIndex Cabinet, IoBinding Binding, ValidIoVariableValueType ValueT, Index VariableIndex>
  requires std::atomic<ValueT>::is_always_lock_free
struct IoVariable
{
  using value_t = ValueT;

  IoVariable() = default;

  IoVariable(IoVariable &) = delete;

  IoVariable(IoVariable &&) = delete;

  IoVariable &operator=(IoVariable &) = delete;

  IoVariable &operator=(IoVariable &&) = delete;

  [[nodiscard]] static constexpr auto index()
  {
    return VariableIndex;
  };

  [[nodiscard]] static constexpr auto cabinet()
  {
    return Cabinet;
  };

  [[nodiscard]] static constexpr auto binding()
  {
    return Binding;
  };

  std::atomic<ValueT> value{};
};

template<typename T>
using IoVariableValueType = typename T::IoVariable::value_t;

/**
 * Simple scalar IO variable.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the variable.
 * @tparam ValueT Value type.
 */
template<CabinetIndex Cabinet, IoBinding Binding, typename ValueT>
struct SimpleIoVariable : IoVariable<Cabinet, Binding, ValueT, 0>
{
};

/**
 * Proxy class for accessing the singleton instance of each IOVariable type.
 */
struct Global
{
  template<IoVariableType T>
  inline static T instance{};
};

/**
 * Vehicle detector call state.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the detector.
 * @tparam Ch Index of the detector.
 */
template<CabinetIndex Cabinet, IoBinding Binding, DetectorChannel Ch>
  requires ValidDetectorChannel<Ch>
struct VehicleDetCall : IoVariable<Cabinet, Binding, Bit, Ch>
{
};

/**
 * The start of the simulation time in Linux timestamp format, in terms of the
 * number of seconds elapsed since the Unix epoch Jan 1, 1970 at 00:00:00 UTC.
 * @remarks Only the UTC time from the first message received by the external
 * controller software (ECS) is used.
 */
template<IoBinding Binding>
struct SimulationStartTime : SimpleIoVariable<0, Binding, Cardinal>
{
};

/**
 * Red or DoNotWalk signal driver of a given load switch channel.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the driver.
 * @tparam Ch Index of the load switch channel.
 */
template<CabinetIndex Cabinet, IoBinding Binding, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
struct ChannelRedDoNotWalkDriver : IoVariable<Cabinet, Binding, Bit, Ch>
{
};

/**
 * Yellow or Pedestrian Clearance signal driver of a given load switch channel.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the driver.
 * @tparam Ch Index of the load switch channel.
 */
template<CabinetIndex Cabinet, IoBinding Binding, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
struct ChannelYellowPedClearDriver : IoVariable<Cabinet, Binding, Bit, Ch>
{
};

/**
 * Green or Walk signal driver of a given load switch channel.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the driver.
 * @tparam Ch Index of the load switch channel.
 */
template<CabinetIndex Cabinet, IoBinding Binding, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
struct ChannelGreenWalkDriver : IoVariable<Cabinet, Binding, Bit, Ch>
{
};

/**
 * Intersection ID stored as OctetNumber.
 * @tparam Cabinet Index of the cabinet.
 * @tparam Binding IO binding of the driver.
 */
template<CabinetIndex Cabinet, IoBinding Binding>
struct IntersectionID : SimpleIoVariable<Cabinet, Binding, OctetNumber>
{
  uint32_t to_uint32()
  {
    auto [_ /* valid */, value] = octets_to_uint32(this->value);
    return value;
  }
};

}// namespace io

namespace frame {

/**
 * Kind of SDLC data frame: can be Command or Response.
 */
enum class FrameKind
{
  Command,
  Response
};

template<typename T>
concept FrameType = requires {
  {
    T::frame_kind()
  } -> std::same_as<FrameKind>;
};

template<typename T>
concept CommandFrameType = requires {
  {
    T::frame_kind()
  } -> std::same_as<FrameKind>;

  T::frame_kind() == FrameKind::Command;
};

template<typename T>
concept ResponseFrameType = requires {
  {
    T::frame_kind()
  } -> std::same_as<FrameKind>;

  T::frame_kind() == FrameKind::Response;
};

namespace detail {

/**
 * An element of data frame. An instance of the element can be stored as Bit,
 * Byte, Word, Cardinal, or OctetNumber.
 */
struct FrameElement
{
  virtual void operator<<(std::span<const Byte> data_in) = 0;
  virtual void operator>>(std::span<Byte> data_out) = 0;
};

/**
 * Frame element stored as Bit.
 * @tparam T The IO variable type, its value type is Bit.
 * @tparam BitPos 0-based bit position in the frame payload.
 */
template<typename T, size_t BitPos>
  requires std::is_same_v<io::IoVariableValueType<T>, Bit>
struct FrameBit : FrameElement
{
  void operator<<(const std::span<const Byte> data_in) final
  {
    static auto byte_pos = pos / 8;
    static auto nbits_to_shift = pos % 8;
    auto value = (data_in[byte_pos] & (0x01 << nbits_to_shift)) != 0;
    ref_var.value = static_cast<Bit>(value);
  }

  void operator>>(const std::span<Byte> data_out) final
  {
    static auto byte_pos = pos / 8;
    static auto num_of_bits_to_shift = pos % 8;
    Byte i = (ref_var.value == Bit::On) ? 1 : 0;
    data_out[byte_pos] = data_out[byte_pos] | (i << num_of_bits_to_shift);
  }

  size_t pos{BitPos};
  T &ref_var{io::Global::instance<T>};
};

/**
 * Frame element stored as Byte.
 * @tparam T The IO variable type, its value type is Byte.
 * @tparam BitPos 0-based byte position in the frame payload.
 */
template<typename T, size_t BytePos>
  requires std::is_same_v<io::IoVariableValueType<T>, Byte>
struct FrameByte : FrameElement
{
  void operator<<(const std::span<const Byte> data_in) final
  {
    ref_var.value = data_in[pos];
  }

  void operator>>(const std::span<Byte> data_out) final
  {
    data_out[pos] = ref_var.value;
  }

  size_t pos{BytePos};
  T &ref_var{io::Global::instance<T>};
};

/**
 * Frame element stored as Word.
 * @tparam T The IO variable type, its value type is Word.
 * @tparam BitPos 0-based starting byte position in the frame payload.
 */
template<typename T, size_t BytePos> /* BytePos: position of Word value low byte */
  requires std::is_same_v<io::IoVariableValueType<T>, Word>
struct FrameWord : FrameElement
{
  void operator<<(const std::span<const Byte> data_in) final
  {
    ref_var.value = data_in[pos] | (data_in[pos + 1] << 8);// LByte | HByte
  }

  void operator>>(const std::span<Byte> data_out) final
  {
    data_out[pos] = ref_var.value & 0xFF;
    data_out[pos + 1] = ref_var.value >> 8 & 0xFF;
  }

  size_t pos{BytePos};
  T &ref_var{io::Global::instance<T>};
};

/**
 * Frame element stored as Cardinal.
 * @tparam T The IO variable type, its value type is Cardinal.
 * @tparam BitPos 0-based byte position in the frame payload.
 */
template<typename T, size_t BytePos> /* BytePos: position of value the least significant byte */
  requires std::is_same_v<io::IoVariableValueType<T>, Cardinal>
struct FrameCardinal : FrameElement
{
  void operator<<(const std::span<const Byte> data_in) final
  {
    ref_var.value = data_in[pos]
        | data_in[pos + 1] << 8
        | data_in[pos + 2] << 16
        | data_in[pos + 3] << 24;
  }

  void operator>>(const std::span<Byte> data_out) final
  {
    data_out[pos] = ref_var.value & 0xFF;
    data_out[pos + 1] = ref_var.value >> 0x08 & 0xFF;
    data_out[pos + 2] = ref_var.value >> 0x10 & 0xFF;
    data_out[pos + 3] = ref_var.value >> 0x18 & 0xFF;
  }

  size_t pos{BytePos};
  T &ref_var{io::Global::instance<T>};
};

/**
 * Frame element stored as OctetNumber.
 * @tparam T The IO variable type, its value type is OctetNumber.
 * @tparam BitPos 0-based byte position in the frame payload.
 */
template<typename T, size_t BytePos> /* BytePos: position of OctetString starting byte */
  requires std::is_same_v<io::IoVariableValueType<T>, OctetNumber>
struct FrameOctetNumber : FrameElement
{
  void operator<<(const std::span<const Byte> data_in) final
  {
    OctetNumber value{0};

    for (auto i = 0; i < sizeof(OctetNumber); i++) {
      value = value | static_cast<uint64_t>(data_in[pos + i]) << 8 * (sizeof(OctetNumber) - 1 - i);
    }

    ref_var.value = value;
  }

  void operator>>(const std::span<Byte> data_out) final
  {
    for (auto i = 0; i < sizeof(OctetNumber); i++) {
      data_out[pos + i] = ref_var.value >> 8 * (sizeof(OctetNumber) - 1 - i);
    }
  }

  size_t pos{BytePos};
  T &ref_var{io::Global::instance<T>};
};

template<size_t N>
concept ValidFrameByteSize = (N >= 1) && (N <= MAX_SDLC_FRAME_BYTES);

template<typename T>
concept ValidFrameElement = std::is_base_of_v<FrameElement, T>;

template<size_t ByteSize, typename... Ts>
concept ValidFrame = (ValidFrameElement<Ts> && ...) && ValidFrameByteSize<ByteSize>;

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

/**
 * This class enables a functional and declarative style for defining SDLC frames.
 * @tparam Address SDLC address.
 * @tparam FrameID Frame Type ID.
 * @tparam FrameByteSize Number of bytes required by this frame.
 * @tparam Binding IO binding for the underlying IO variables.
 * @tparam FrameElementTs Frame element types.
 */
template<Byte Address,
         Byte FrameID,
         size_t FrameByteSize,
         io::IoBinding Binding,
         typename... FrameElementTs>
  requires ValidFrame<FrameByteSize, FrameElementTs...>
class Frame
{
public:
  Frame() noexcept = default;

  Frame(Frame &) = delete;

  Frame(Frame &&) = delete;

  Frame &operator=(Frame &) = delete;

  Frame &operator=(Frame &&) = delete;

  /**
   * Deserialize frame payload into respective IO variables.
   * @param data_in Frame payload to deserialize from.
   */
  void operator<<(const std::span<const Byte> data_in)
  {
    static_assert(((Binding != io::IoBinding::Cu) and (FrameID <= 127))
                  or /**/
                  ((Binding == io::IoBinding::Cu) and (FrameID >= 128)));

    Assign(data_in);
  }

  /**
   * Serialize IO variables into frame payload.
   * @param data_out Frame payload to serialize to.
   */
  void operator>>(std::span<Byte> data_out)
  {
    static_assert(((Binding == io::IoBinding::Cu) and (FrameID <= 127))
                  or /**/
                  ((Binding != io::IoBinding::Cu) and (FrameID >= 128)));

    std::fill(data_out.begin(), data_out.end(), 0);
    data_out[0] = address();
    data_out[1] = 0x83;
    data_out[2] = id();
    Generate(data_out);
  }

  /**
   * Reset underlying IO variables to 0's.
   */
  void Reset()
  {
    std::array<const Byte, MAX_SDLC_FRAME_BYTES> data_in{};
    Assign(data_in);
  }

  [[nodiscard]] static constexpr auto address()
  {
    return Address;
  };

  [[nodiscard]] static constexpr auto id()
  {
    return FrameID;
  };

  [[nodiscard]] static constexpr auto byte_size()
  {
    return FrameByteSize;
  };

  /*
    NEMA-TS2: CommandFrame ID is [0,127], ResponseFrame [128,255]

    CommandFrame  : 000 - 063 NEMA-TS2
                    064 - 127 Manufacture

    ResponseFrame:  128-191 NEMA-TS2
                    192-255 Manufacture
   */

  /**
   * Returns the frame kind, with value being Command or Response.
   */
  [[nodiscard]] static constexpr auto frame_kind()
  {
    return (FrameID <= 127) ? FrameKind::Command : FrameKind::Response;
  }

private:
  template<size_t I = 0>
  inline void Assign(const std::span<const Byte> data_in)
  {
    if constexpr (I < sizeof...(FrameElementTs)) {
      std::get<I>(frame_elements_) << data_in;
      Assign<I + 1>(data_in);
    }
  }

  template<size_t I = 0>
  inline void Generate(std::span<Byte> data_out)
  {
    if constexpr (I < sizeof...(FrameElementTs)) {
      std::get<I>(frame_elements_) >> data_out;
      Generate<I + 1>(data_out);
    }
  }

  std::tuple<FrameElementTs...> frame_elements_;
};

template<Byte Address,
         Byte FrameID,
         size_t FrameByteSize,
         io::IoBinding Binding,
         typename... FrameElementTs>
struct FrameGenerator
{
};

template<Byte Address,
         Byte FrameID,
         size_t FrameByteSize,
         io::IoBinding Binding,
         typename... FrameElementTs>
struct FrameGenerator<Address, FrameID, FrameByteSize, Binding, std::tuple<FrameElementTs...>>
{
  using type = Frame<Address, FrameID, FrameByteSize, Binding, FrameElementTs...>;
};

//-----------------------------------------------------------------
// Detector Call Data Frame Bits Generator
//-----------------------------------------------------------------

/**
 * A type factory for io::VehicleDetCall element in detector call data frame used in simulation.
 * @tparam Cabinet Index of cabinet.
 * @tparam Binding IO binding.
 * @tparam N Index of detector channel.
 */
template<CabinetIndex Cabinet, io::IoBinding Binding, int N>
struct CallDataFrameBit
{
  using type = FrameBit<
      io::VehicleDetCall<Cabinet, Binding, N>,
      // The first 3 bytes are Address, Control, and FrameID. The bit position definition starts at 24.
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

/* The type would be std::tuple<
   FrameBit<VehicleDetCall<Cabinet, Binding, 128>, 151>
   FrameBit<VehicleDetCall<Cabinet, Binding, 127>, 150>
   FrameBit<VehicleDetCall<Cabinet, Binding, 126>, 149>
   ...
   FrameBit<VehicleDetCall<Cabinet, Binding, 1>, 24>
   >
 */
template<CabinetIndex Cabinet, io::IoBinding Binding>
using CallDataFrameBits = typename CallDataFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_VEHICLE_DETECTORS>::type;

/**
 * A type factory for io::ChannelGreenWalkDriver element in load switch driver frame for simulation.
 * @tparam Cabinet Index of cabinet.
 * @tparam Binding IO binding.
 * @tparam N Index of load switch channel.
 */
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

/*
   The type would be std::tuple<
   FrameBit<ChannelGreenWalkDriver<Cabinet, Binding, 32>, 119>
   FrameBit<ChannelGreenWalkDriver<Cabinet, Binding, 31>, 116>
   FrameBit<ChannelGreenWalkDriver<Cabinet, Binding, 30>, 113>
   ...
   FrameBit<ChannelGreenWalkDriver<Cabinet, Binding, 1>, 26>
   >
 */
template<CabinetIndex Cabinet, io::IoBinding Binding>
using GreenChannelDriverFrameBits = typename GreenChannelDriverFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_LOAD_SWITCHES>::type;

/**
 * A type factory for io::ChannelYellowPedClearDriver element in load switch driver frame for simulation.
 * @tparam Cabinet Index of cabinet.
 * @tparam Binding IO binding.
 * @tparam N Index of load switch channel.
 */
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

/*
   The type would be std::tuple<
   FrameBit<ChannelYellowPedClearDriver<Cabinet, Binding, 32>, 118>
   FrameBit<ChannelYellowPedClearDriver<Cabinet, Binding, 31>, 115>
   FrameBit<ChannelYellowPedClearDriver<Cabinet, Binding, 30>, 112>
   ...
   FrameBit<ChannelGreenWalkDriver<Cabinet, Binding, 1>, 25>
   >
 */
template<CabinetIndex Cabinet, io::IoBinding Binding>
using YellowChannelDriverFrameBits = typename YellowChannelDriverFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_LOAD_SWITCHES>::type;

/**
 * A type factory for io::ChannelRedDoNotWalkDriver element in load switch driver frame for simulation.
 * @tparam Cabinet Index of cabinet.
 * @tparam Binding IO binding.
 * @tparam N Index of load switch channel.
 */
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

/*
   The type would be std::tuple<
   FrameBit<ChannelRedDoNotWalkDriver<Cabinet, Binding, 32>, 117>
   FrameBit<ChannelRedDoNotWalkDriver<Cabinet, Binding, 31>, 114>
   FrameBit<ChannelRedDoNotWalkDriver<Cabinet, Binding, 30>, 111>
   ...
   FrameBit<ChannelRedDoNotWalkDriver<Cabinet, Binding, 1>, 24>
   >
 */
template<CabinetIndex Cabinet, io::IoBinding Binding>
using RedChannelDriverFrameBits = typename RedChannelDriverFrameBitGenerator<
    Cabinet,
    Binding,
    MAX_LOAD_SWITCHES>::type;

template<CommandFrameType T1, ResponseFrameType T2>
using FrameMap = std::tuple<T1 &, T2 &>;

template<typename... Ts>
using FrameMaps = std::tuple<Ts...>;

}// namespace detail

/**
 * Proxy class for accessing the singleton instance of each frame type.
 */
struct Global
{
  template<FrameType T>
  inline static T instance{};
};

/*
 Type 64 Command Frame for Software in the Loop Simulation. It is used in the
 initial handshake phase, transmitted from the External Controller Software(ECS)
 to notify the traffic simulator that the ECS is ready. The frame contains the
 ID of the target intersection, serving as a means to identify and establish
 communication between the ECS and the traffic simulator. By this frame, the ECS
 initiates the simulation process and signals its readiness to begin the desired
 traffic scenario.
 */

/**
 * Type 64 Command Frame for establishing the communication between external
 * controller software (ECS) and traffic simulator.
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

template<typename T>
concept BiuType = requires {
  T::device_kind() == DeviceKind::Biu;
};

/**
 * Abstract BIU type. It should be used as the base class and should not create an
 * instance by itself.
 * @tparam Derived The specific BIU type.
 */
template<typename Derived>
struct Biu : public SdlcSecondaryStationDevice<Biu<Derived>>
{
public:
  friend class SdlcSecondaryStationDevice<Biu<Derived>>;

private:
  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoGenerateResponseFrame(Byte frame_id)
  {
    if constexpr (I < Derived::frame_maps_size()) {
      auto &res_frame = std::get<1>(std::get<I>(static_cast<Derived *>(this)->frame_maps_));
      if (res_frame.id() == frame_id) {
        res_frame >> this->buffer_;
        return {true, {this->buffer_.data(), res_frame.byte_size()}};
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
    if constexpr (I < Derived::frame_maps_size()) {
      auto &cmd_frame = std::get<0>(std::get<I>(static_cast<Derived *>(this)->frame_maps_));
      if (cmd_frame.id() == data[2]) {
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
    if constexpr (I < Derived::frame_maps_size()) {
      auto &cmd_frame = std::get<0>(std::get<I>(static_cast<Derived *>(this)->frame_maps_));
      auto &res_frame = std::get<1>(std::get<I>(static_cast<Derived *>(this)->frame_maps_));
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
    if constexpr (I < Derived::frame_maps_size()) {
      auto &cmd_frame = std::get<0>(std::get<I>(static_cast<Derived *>(this)->frame_maps_));
      auto &res_frame = std::get<1>(std::get<I>(static_cast<Derived *>(this)->frame_maps_));

      if (cmd_frame.id() == data_in[2]) {
        cmd_frame << data_in;
        res_frame >> this->buffer_;// Buffer will be emptied at the beginning of >>().
        return {true, {this->buffer_.data(), res_frame.byte_size()}};
      } else {
        return DoDispatch<I + 1>(data_in);
      }
    } else {
      return {false, {this->buffer_.data(), 0}};
    }
  }

  [[nodiscard]] static consteval auto get_cabinet()
  {
    return Derived::get_cabinet();
  }

  [[nodiscard]] static consteval auto get_device_kind()
  {
    return DeviceKind::Biu;
  }

  [[nodiscard]] static consteval auto get_frame_maps_size()
  {
    return Derived::get_frame_maps_size();
  }

private:
  std::array<Byte, MAX_SDLC_FRAME_BYTES> buffer_{};
};

template<CabinetIndex Cabinet, Byte FrameID>
struct BiuFrameType
{
};

template<CabinetIndex Cabinet>
struct BiuFrameType<Cabinet, 64>
{
  using type = frame::SimulatorInitialHandshakeFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct BiuFrameType<Cabinet, 192>
{
  using type = frame::SimulatorInitialHandshakeAckFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct BiuFrameType<Cabinet, 65>
{
  using type = frame::SimulatorCallRequestFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct BiuFrameType<Cabinet, 193>
{
  using type = frame::SimulatorCallDataFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct BiuFrameType<Cabinet, 66>
{
  using type = frame::SimulatorLoadSwitchDriversFrame<Cabinet, io::IoBinding::Fio>;
};

template<CabinetIndex Cabinet>
struct BiuFrameType<Cabinet, 194>
{
  using type = frame::SimulatorLoadSwitchDriversAckFrame<Cabinet, io::IoBinding::Fio>;
};

/**
 * Proxy class for accessing the singleton instance of each biu type.
 */
struct Global
{
  template<BiuType T>
  inline static T instance{};
};

/**
 * A BIU type  for traffic simulator such as TransModeler.
 * @tparam Cabinet Index of the cabinet.
 */
template<CabinetIndex Cabinet>
  requires ValidCabinetIndex<Cabinet>
struct SilsBiu : public Biu<SilsBiu<Cabinet>>
{
public:
  friend class Biu<SilsBiu<Cabinet>>;

private:
  using SilsBiuFrameMaps = frame::detail::FrameMaps<
      frame::detail::FrameMap<typename BiuFrameType<Cabinet, 64>::type,
                              typename BiuFrameType<Cabinet, 192>::type>,
      frame::detail::FrameMap<typename BiuFrameType<Cabinet, 65>::type,
                              typename BiuFrameType<Cabinet, 193>::type>,
      frame::detail::FrameMap<typename BiuFrameType<Cabinet, 66>::type,
                              typename BiuFrameType<Cabinet, 194>::type>>;

  const SilsBiuFrameMaps frame_maps_{
      std::make_tuple(
          std::make_tuple(std::ref(frame::Global::instance<typename BiuFrameType<Cabinet, 64>::type>),
                          std::ref(frame::Global::instance<typename BiuFrameType<Cabinet, 192>::type>)),
          std::make_tuple(std::ref(frame::Global::instance<typename BiuFrameType<Cabinet, 65>::type>),
                          std::ref(frame::Global::instance<typename BiuFrameType<Cabinet, 193>::type>)),
          std::make_tuple(std::ref(frame::Global::instance<typename BiuFrameType<Cabinet, 66>::type>),
                          std::ref(frame::Global::instance<typename BiuFrameType<Cabinet, 194>::type>)))};

  [[nodiscard]] static consteval auto get_frame_maps_size()
  {
    return std::tuple_size_v<SilsBiuFrameMaps>;
  };

  [[nodiscard]] static consteval auto get_cabinet()
  {
    return Cabinet;
  };
};

}// namespace biu

namespace aux {// NEMA-TS2 Auxiliary Devices

template<CabinetIndex Cabinet, typename Factory, typename T, T... Is>
auto make_wirings(Factory, std::integer_sequence<T, Is...>)
{
  return std::make_tuple(Factory::template make<Cabinet, Is>()...);
}

/**
 * Enumerates a list of wirings and applies the func to each wiring item.
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

enum class LoadSwitchState : short
{
  Blank = 0,
  Red = 1,
  Yellow = 2,
  Green = 3
};

template<typename T>
concept LoadSwitchType = requires(T t) {
  {
    t.state()
  } -> std::same_as<LoadSwitchState>;
};

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
      std::ref(Global::instance<ChannelGreenWalkDriver<Cabinet, IoBinding::Fio, Ch>>),      /**/
      std::ref(Global::instance<ChannelYellowPedClearDriver<Cabinet, IoBinding::Fio, Ch>>), /**/
      std::ref(Global::instance<ChannelRedDoNotWalkDriver<Cabinet, IoBinding::Fio, Ch>>));  /**/
}

}// namespace detail

/**
 * An integer sequence of 1 .. MAX_LOAD_SWITCHES
 */
using LoadSwitchChannels =
    offset_sequence_t<
        0,
        std::make_integer_sequence<
            LoadSwitchChannel,
            MAX_LOAD_SWITCHES>>;

template<CabinetIndex Cabinet, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch> && ValidCabinetIndex<Cabinet>
struct LoadSwitch
{
  [[nodiscard]] LoadSwitchState state() const noexcept
  {
    auto &[g, y, r] = driver_;
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

  [[nodiscard]] LoadSwitchState operator()() const noexcept
  {
    return state();
  }

  void set_state(LoadSwitchState value) noexcept
  {
    auto &[g, y, r] = driver_;
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

  [[nodiscard]] static constexpr auto cabinet()
  {
    return Cabinet;
  };

  [[nodiscard]] static constexpr auto channel()
  {
    return Ch;
  };

private:
  const detail::LoadSwitchDriver<Cabinet, Ch> driver_{detail::make_load_switch_driver<Cabinet, Ch>()};
};

template<CabinetIndex Cabinet, LoadSwitchChannel Ch>
  requires ValidLoadSwitchChannel<Ch>
using LoadSwitchWiring = std::tuple<LoadSwitch<Cabinet, Ch> &, SignalGroup>;

/**
 * Proxy class for accessing the singleton instance of each load switch.
 */
struct Global
{
  template<LoadSwitchType T>
  inline static T instance{};
};

struct LoadSwitchWiringFactory
{
  template<CabinetIndex Cabinet, LoadSwitchChannel Ch>
  static auto make()
  {
    return std::make_tuple(std::ref(Global::instance<LoadSwitch<Cabinet, Ch>>), SignalGroup{});
  }
};

template<CabinetIndex Cabinet>
using LoadSwitchWirings =
    decltype(make_wirings<Cabinet>(LoadSwitchWiringFactory{}, LoadSwitchChannels{}));

}// namespace lsw

namespace du {// detector unit

template<typename T>
concept DetectorUnitType = requires(T t) {
  {
    t.activated()
  } -> std::same_as<bool>;
};

using DetectorChannels =
    offset_sequence_t<
        0,
        std::make_integer_sequence<
            DetectorChannel,
            MAX_VEHICLE_DETECTORS>>;

template<CabinetIndex Cabinet, DetectorChannel Ch>
  requires ValidDetectorChannel<Ch> && ValidCabinetIndex<Cabinet>
struct DetectorUnit
{
  [[nodiscard]] bool activated() const noexcept
  {
    return state_.value == Bit::On;
  }

  void set_activated(bool value) noexcept
  {
    state_.value = value ? Bit::On : Bit::Off;
  }

  [[nodiscard]] bool operator()() const noexcept
  {
    return activated();
  }

  [[nodiscard]] static consteval auto cabinet()
  {
    return Cabinet;
  };

  [[nodiscard]] static consteval auto channel()
  {
    return Ch;
  };

private:
  io::VehicleDetCall<Cabinet, io::IoBinding::Fio, Ch>
      &state_{io::Global::instance<io::VehicleDetCall<Cabinet, io::IoBinding::Fio, Ch>>};
};

template<CabinetIndex Cabinet, DetectorChannel Ch>
  requires ValidDetectorChannel<Ch> && ValidCabinetIndex<Cabinet>
using DetectorWiring = std::tuple<DetectorUnit<Cabinet, Ch> &, DetectionZoneIDs>;

/**
 * Proxy class for accessing the singleton instance of each detector unit type.
 */
struct Global
{
  template<DetectorUnitType T>
  inline static T instance{};
};

struct DetectorWiringFactory
{
  template<CabinetIndex Cabinet, DetectorChannel Ch>
  static auto make()
  {
    return std::make_tuple(std::ref(Global::instance<DetectorUnit<Cabinet, Ch>>), DetectionZoneIDs{});
  }
};

template<CabinetIndex Cabinet>
using DetectorWirings = decltype(make_wirings<Cabinet>(DetectorWiringFactory{}, DetectorChannels{}));

}// namespace du

}// namespace aux

namespace xils {// X in the loop simulation, x = software/hardware

/**
 * X-in-the-Loop Simulation (XILS) interface that needs to be implemented
 * by the simulator.
 */
struct IXilSimulator
{
  /**
   * Gets the controller id for the given cabinet from the simulator. This id is
   * defined by the simulator.
   * @param cabinet Index of the cabinet.
   */
  virtual ControllerID GetControllerID(CabinetIndex cabinet) = 0;

  /**
   * Gets sensor state, which can be ON or OFF, of a given sensor from the simulator.
   * @param sensor_id ID of the sensor defined by the simulator.
   */
  virtual bool GetSensorState(DetectionZoneID sensor_id) = 0;

  /**
   * Sets signal state of a given signal in the simulator. The signal can be either
   * identified by signal group ID, or by the combination of approach ID and turn
   * ID. Some simulator implementation uses signal group ID, some the other way.
   * @param sg_id ID of the signal group in the simulation model.
   * @param approach_id ID of the approach in the simulation model.
   * @param turn_id ID of the turn movement in the simulation model.
   * @param state Signal state to set.
   */
  virtual void SetSignalState(SignalGroupID sg_id,
                              ApproachID approach_id,
                              TurnIndex turn_id,
                              aux::lsw::LoadSwitchState state) = 0;

  /**
   * Gets the list of sensor IDs in the simulation model associated with the given
   * detector channel.
   * @param id ID of the controller in the simulation model.
   * @param ch ID of the detector channel.
   */
  virtual std::span<const DetectionZoneID> GetDetectionZoneIDs(ControllerID id, DetectorChannel ch) = 0;

  /**
   * A signal group refers to a specific movement or set of movements of vehicles
   * at an intersection that can be controlled by a load switch output. This class
   * includes a tuple of SignalGroupID and the associated list of movements.
   * @remarks Note the difference from SignalGroup defined as
   * std::tuple<SignalGroupID,vector<Movement>>, and the use of std::span here for
   * better memory management.
   */
  using SignalGroupEx = std::tuple<SignalGroupID, std::span<Movement>>;

  /**
   * Gets signal group (if defined) and turn movements defined in the simulation model
   * that is associated with the given load switch channel.
   * @param id ID of the controller in the simulation model.
   * @param ch ID of the load switch channel.
   */
  virtual SignalGroupEx GetSignalGroup(ControllerID id, LoadSwitchChannel ch) = 0;
};

}// namespace xils

namespace rack {// biu rack

template<typename T>
concept RackType = requires {
  T::device_kind() == DeviceKind::Rack;
};

/**
 * Rack manages a list of BIUs. It should be used as the base class, and should
 * not create an instance by itself.
 * @tparam Derived
 */
template<typename Derived>
struct Rack : public SdlcSecondaryStationDevice<Rack<Derived>>
{
  friend class SdlcSecondaryStationDevice<Rack<Derived>>;

  [[nodiscard]] static constexpr auto rack_size()
  {
    return Derived::get_rack_size();
  }

private:
  [[nodiscard]] static consteval auto get_cabinet()
  {
    return Derived::get_cabinet();
  }

  [[nodiscard]] static constexpr auto get_device_kind()
  {
    return DeviceKind::Rack;
  }

  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoGenerateResponseFrame(Byte frame_id)
  {
    if constexpr (I < rack_size()) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->get_bius());
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
    if constexpr (I < Derived::get_rack_size()) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->get_bius());
      return biu.ProcessCommandFrame(data) || DoProcessCommandFrame<I + 1>(data);
    } else {
      return false;
    }
  }

  template<size_t I = 0>
  void DoReset()
  {
    if constexpr (I < Derived::get_rack_size()) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->get_bius());
      return biu.Reset(), DoReset<I + 1>();
    } else {
      return;
    }
  }

  template<size_t I = 0>
  std::tuple<bool, std::span<const Byte>> DoDispatch(std::span<const Byte> data)
  {
    if constexpr (I < Derived::get_rack_size()) {
      auto &biu = std::get<I>(static_cast<Derived *>(this)->get_bius());
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
};

/**
 * Proxy class for accessing the singleton instance of each rack type.
 */
struct Global
{
  template<RackType T>
  inline static T instance{};
};

/**
 * X-in-the-Loop Simulation Biu Rack. It should be used as the base for
 * Hardware-in-the-Loop or Software-in-the-Loop Simulation Biu Rack, and
 * is not supposed to create an instance by itself.
 */
template<typename Derived, CabinetIndex Cabinet>
struct XilsBiuRack : Rack<XilsBiuRack<Derived, Cabinet>>
{
public:
  friend class Rack<XilsBiuRack<Derived, Cabinet>>;
  /**
   * Processes detector unit wirings. This involves obtaining detector states
   * from simulation model and sets the states to the detector unit channel.
   */
  void ProcessDetectorWirings()
  {
    if (not simulator_)
      return;

    aux::for_each(du_wirings_, [&](auto &&el) {
      auto &[detector, sensor_ids] = el;
      bool value = false;

      if (!sensor_ids.empty()) {
        for (const auto sensor_id : sensor_ids) {
          value = value or simulator_->GetSensorState(sensor_id);
        }
        detector.set_activated(value);
      }
    });
  }

  /**
   * Processes load switch wirings. This involves obtaining load switch driver
   * states and sets the states to signals in the simulation model.
   */
  void ProcessLoadSwitchWirings()
  {
    if (not simulator_)
      return;

    aux::for_each(lsw_wirings_, [&](auto &&el) {
      auto &[lsw, sg] = el;
      auto &[sg_id, movements] = sg;

      if (not movements.empty()) {
        for (const auto &movement : movements) {
          const auto &[approach, turn] = movement;
          simulator_->SetSignalState(0, approach, turn, lsw.state());
        }
      } else if (sg_id > 0) {
        simulator_->SetSignalState(sg_id, 0, 0, lsw.state());
      }
    });
  }

  /**
   * Sets the simulator interface to this rack instance. This will sets up the
   * detector unit channel mapping and load switch channel mapping between the
   * external controller software and the traffic simulator.
   * @param a_simulator The simulator interface; must be not null.
   */
  void SetSimulator(xils::IXilSimulator *a_simulator)
  {
    if (not a_simulator)
      throw std::runtime_error("NRE while calling Rack::SetSimulator.");

    simulator_ = a_simulator;
    controller_id_ = a_simulator->GetControllerID(this->cabinet());
    // Set up the simulation sensor ids and detector unit channel mapping.
    aux::for_each(du_wirings_, [&](auto &&el) {
      auto &[du, zones] = el;
      auto l_zones = simulator_->GetDetectionZoneIDs(controller_id_, du.channel());
      zones.clear();

      for (int i = 0; i < l_zones.size(); i++) {
        zones.push_back(l_zones[i]);
      }
    });

    // Set up simulation signals and load switch channel mapping.
    aux::for_each(lsw_wirings_, [&](auto &&el) {
      auto &[lsw, sg] = el;
      auto &[sg_id, movements] = sg;
      movements.clear();

      auto [l_sg_id, l_movements] = simulator_->GetSignalGroup(controller_id_, lsw.channel());
      // Set signal group id based on simulator returns.
      sg_id = l_sg_id;
      // Set movements.
      for (int i = 0; i < l_movements.size(); i++) {
        movements.push_back(l_movements[i]);
      }
    });
  }

  /**
   * ID of the associated controller.
   * @return
   */
  [[nodiscard]] ControllerID controller_id() const
  {
    return controller_id_;
  }

private:
  [[nodiscard]] static consteval auto get_rack_size()
  {
    return Derived::rack_size_;
  };

  [[nodiscard]] static consteval auto get_cabinet()
  {
    return Cabinet;
  };

  [[nodiscard]] const auto &get_bius()
  {
    return static_cast<Derived *>(this)->bius_;
  };

  /*
   * We have to include Cabinet as template argument because we need it for the
   * following template specialization.
   */
  aux::lsw::LoadSwitchWirings<Cabinet> lsw_wirings_{
      aux::make_wirings<Cabinet>(aux::lsw::LoadSwitchWiringFactory{},
                                 aux::lsw::LoadSwitchChannels{})};

  aux::du::DetectorWirings<Cabinet> du_wirings_{
      aux::make_wirings<Cabinet>(aux::du::DetectorWiringFactory{},
                                 aux::du::DetectorChannels{})};

  ControllerID controller_id_{0};

  xils::IXilSimulator *simulator_{nullptr};
};

/**
 * Software in the Loop Simulation BIU Rack.
 * @tparam Cabinet Index of the cabinet.
 */
template<CabinetIndex Cabinet>
  requires ValidCabinetIndex<Cabinet>
struct SilsBiuRack : public XilsBiuRack<SilsBiuRack<Cabinet>, Cabinet>
{
public:
  friend class XilsBiuRack<SilsBiuRack<Cabinet>, Cabinet>;

private:
  using Bius = std::tuple<biu::SilsBiu<Cabinet> &>;
  static constexpr auto rack_size_{std::tuple_size_v<Bius>};
  const Bius bius_{std::ref(biu::Global::instance<biu::SilsBiu<Cabinet>>)};
};

}// namespace rack

}// namespace vtc

#endif//VIRTUAL_TRAFFIC_CABINET_H_