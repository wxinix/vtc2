#pragma once

#include <format>
#include <magic_enum/magic_enum.hpp>

namespace vtc::io::basic_types {

/**
 * Maximum number of load switch channels supported by each virtual cabinet.
 */
inline constexpr size_t MAX_LOAD_SWITCH_CHANNELS{32};

/**
 * Maximum number of detector channels supported by each virtual cabinet.
 */
inline constexpr size_t MAX_VEHICLE_DETECTORS{128};

/**
 * The buffer size of SDLC instance processor.
 */
inline constexpr size_t MAX_SDLC_FRAME_BYTES{64};

enum class IoKind
{
    Input,
    Output,
    InputOutput,
    OptoInput
};

template<typename T>
concept IsIoKind = std::is_same_v<T, IoKind>;

enum class InputFuncKind
{
    NotActive,
    Alarm1,
    Alarm2,
    AlternateSequenceA,
    AlternateSequenceB,
    AlternateSequenceC,
    AlternateSequenceD,
    AutomaticFlash,
    CallToNonActuated1,
    CallToNonActuated2,
    FreeNoCoord,
    DimmingEnable,
    ExternalMinimumRecall,
    ExternalStart,
    IntervalAdvance,
    LocalFlashStatus,
    ManualControlEnable,
    MmuFlashStatus,
    Offset1,
    Offset2,
    Offset3,
    PedestrianDetector,
    PedestrianOmit,
    PhaseHold,
    PhaseOmit,
    PreemptDetector,
    RingForceOff,
    RingInhibitMaxTermination,
    RingMax2Selection,
    RingOmitRedClearance,
    RingPedRecycle,
    RingRedRest,
    RingStopTiming,
    SignalPlanA,
    SignalPlanB,
    SpecialFunctionInput,
    SystemAddressBit0,
    SystemAddressBit1,
    SystemAddressBit2,
    SystemAddressBit3,
    SystemAddressBit4,
    TBCOnline,
    TestA,
    TestB,
    TestC,
    TimingPlanA,
    TimingPlanB,
    TimingPlanC,
    TimingPlanD,
    VehicleDetectorCall,
    VehicleDetectorReset,
    WalkRestModifier,
};

enum class OutputFuncKind
{
    NotActive,
    AutomaticFlashStatus,
    LoadSwitchGreenDriver,
    LoadSwitchYellowDriver,
    LoadSwitchRedDriver,
    FreeNoCoordStatus,
    Offset1,
    Offset2,
    Offset3,
    PhaseCheck,
    PhaseNext,
    PhaseOn,
    PreemptStatus,
    RingStatusBitA,
    RingStatusBitB,
    RingStatusBitC,
    SystemSpecialFunction,
    TBCAuxiliary1,
    TBCAuxiliary2,
    TBCAuxiliary3,
    TimingPlanA,
    TimingPlanB,
    TimingPlanC,
    TimingPlanD,
    VehDetectorReset
};

template<typename T>
concept IsIoFuncKind = std::is_same_v<T, InputFuncKind> || std::is_same_v<T, OutputFuncKind>;

template<auto IoFuncKind, size_t IoFuncNumber = 0>
    requires IsIoFuncKind<decltype(IoFuncKind)>
struct IoFunc
{
    using io_func_kind_type = decltype(IoFuncKind);

    static constexpr auto kind{IoFuncKind};
    static constexpr auto number{IoFuncNumber};

    static inline auto name = [] {
        constexpr auto kind_str = magic_enum::enum_name(IoFuncKind);
        const auto num_str = std::to_string(IoFuncNumber);
        return std::format("{} {}", kind_str, num_str);
    }();

    std::atomic_bool io_func_state{false};
};

// Define the concept to enforce that T is an instantiation of IoFunction
template<typename F>
concept IoFuncConcept = std::is_same_v<F, IoFunc<F::kind, F::number>>;

template<typename F>
concept InputFuncConcept = IoFuncConcept<F> && std::is_same_v<typename F::KindT, InputFuncKind>;

template<typename F>
concept OutputFuncConcept = IoFuncConcept<F> && std::is_same_v<typename F::KindT, OutputFuncKind>;

template<IoKind IoKind, size_t IoNumber>
struct IoBase
{
    static constexpr auto kind{IoKind};
    static constexpr auto number{IoNumber};

    static inline std::string name = [] {
        constexpr auto kind_str = magic_enum::enum_name(IoKind);
        const auto num_str = std::to_string(IoNumber);
        return std::format("{} {}", kind_str, num_str);
    }();
};

template<IoKind IoKind, size_t IoNumber, IoFuncConcept F>
struct CustomIo : IoBase<IoKind, IoNumber>
{
    using io_func_type = F;

    // Conditionally include buffer only when AIoFuncKind is OutputFunctionKind
    std::conditional_t<                                                          //
        std::is_same_v<typename io_func_type::io_func_kind_type, OutputFuncKind>,//
        std::atomic_bool,                                                        //
        std::monostate>
        buffer;

    explicit CustomIo(std::atomic_bool &s) : io_func_state_(s)
    {}

    [[nodiscard]] bool ReadIoFuncState() const
        requires std::is_same_v<typename io_func_type::io_func_kind_type, InputFuncKind>
    {
        return io_func_state_.get().load(std::memory_order_relaxed);
    }

    void WriteIoFuncState(const bool value) const
        requires std::is_same_v<typename io_func_type::io_func_kind_type, OutputFuncKind>
    {
        buffer.store(value, std::memory_order_relaxed);
    }

    void TransferOutput() const
        requires std::is_same_v<typename io_func_type::io_func_kind_type, OutputFuncKind>
    {
        auto value = buffer.load(std::memory_order_relaxed);
        io_func_state_.get().store(value, std::memory_order_relaxed);
    }

private:
    std::reference_wrapper<std::atomic_bool> io_func_state_;// reference to the io_function;
};

// Primary IoSignal template (Unimplemented) – will be specialized below
template<IoKind IoKind, size_t IoNumber, IoFuncConcept F>
struct Io : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

// Specialization for Input Function
template<IoKind IoKind, size_t IoNumber, InputFuncConcept F>
    requires(IoKind == basic_types::IoKind::Input || IoKind == basic_types::IoKind::OptoInput)
struct Io<IoKind, IoNumber, F> : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

// Specialization for Output Function
template<IoKind IoKind, size_t IoNumber, OutputFuncConcept F>
    requires(IoKind == basic_types::IoKind::Output)
struct Io<IoKind, IoNumber, F> : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

// Specialization for IoKind::InputOutput (both Input or Output Function)
template<IoKind IoKind, size_t IoNumber, IoFuncConcept F>
    requires(IoKind == basic_types::IoKind::InputOutput)
struct Io<IoKind, IoNumber, F> : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

}// namespace vtc::io::basic_types