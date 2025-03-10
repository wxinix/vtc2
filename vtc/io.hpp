#pragma once

#include <magic_enum/magic_enum.hpp>
#include <utils.hpp>

namespace vtc::io {

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

namespace basic_types {

enum class IoKind
{
    Input,
    Output,
    InputOutput,
    OptoInput
};

enum class InputFuncKind
{
    NotActive,
    CabinetDoorOpen,
    ChannelFaultStatus,
    CoordFreeSwitch,
    CustomAlarm,
    ManualControlGroupAction,
    MinGreen2,
    OverlapOmit,
    PatternInput,
    PedDetectorCall,
    PedDetectorReset,
    PhaseForceOff,
    PhaseHold,
    PhasePedOmit,
    PhasePhaseOmit,
    PreemptGateDown,
    PreemptGateUp,
    PreemptHighPrioritorLow,
    PreemptInput,
    PreemptInputCRC,
    PreemptInputNormalOff,
    PreemptInputNormalOn,
    PrioritorCheckIn,
    PrioritorCheckOut,
    PrioritorPreemptDetector,
    RingForceOff,
    RingInhibitMaxTermination,
    RingMax2Selection,
    RingMax3Selection,
    RingOmitRedClearance,
    RingPedestrianRecycle,
    RingRedRest,
    RingStopTiming,
    SpecialFunctionInput,
    UnitAlarm1,
    UnitAlarm2,
    UnitAlternateSequenceA,
    UnitAlternateSequenceB,
    UnitAlternateSequenceC,
    UnitAlternateSequenceD,
    UnitAutomaticFlash,
    UnitCallPedNA,
    UnitCNA1,
    UnitCNA2,
    UnitClockReset,
    UnitCmuMmuReset,
    UnitDimming,
    UnitExternWatchdog,
    UnitExternalMinRecall,
    UnitExternalStart,
    UnitIndicatorLampControl,
    UnitIntervalAdvance,
    UnitIOModeBit0,
    UnitIOModeBit1,
    UnitIOModeBit2,
    UnitIOModeBit3,
    UnitITSLocalFlashSense,
    UnitLocalFlash,
    UnitLocalFlashSense,
    UnitManualControlEnable,
    UnitMmuFlash,
    UnitOffset1,
    UnitOffset2,
    UnitOffset3,
    UnitSignalPlanA,
    UnitSignalPlanB,
    UnitStopTime,
    UnitSystemAddressBit0,
    UnitSystemAddressBit1,
    UnitSystemAddressBit2,
    UnitSystemAddressBit3,
    UnitSystemAddressBit4,
    UnitTBCHoldOnline,
    UnitTBCOnline,
    UnitTestInputA,
    UnitTestInputB,
    UnitTestInputC,
    UnitTimingPlanA,
    UnitTimingPlanB,
    UnitTimingPlanC,
    UnitTimingPlanD,
    UnitWalkRestModifier,
    VehicleDetectorCall,
    VehicleDetectorReset
};

enum class OutputFuncKind
{
    NotActive,
    AltFlashState,
    AuxFunctionOn,
    ChannelGreenWalkDriver,
    ChannelRedDoNotWalkDriver,
    ChannelYellowPedClearDriver,
    CustomAlarm,
    FlashState,
    GlobalVariable,
    HealthStatus,
    OverlapGreen,
    OverlapProtectedGreen,
    OverlapRed,
    OverlapYellow,
    PedCall,
    PedDetectorReset,
    PhaseAdvWarning,
    PhaseCheck,
    PhaseDoNotWalk,
    PhaseGreen,
    PhaseNext,
    PhaseOmitStatus,
    PhaseOn,
    PhasePedClearance,
    PhasePreClear,
    PhasePreClear2,
    PhaseRed,
    PhaseWalk,
    PhaseYellow,
    PreemptStatus,
    PreemptStatusFlash,
    RingStatusBitA,
    RingStatusBitB,
    RingStatusBitC,
    RingStatusBitD,
    SpecialFunctionOn,
    UnitAutomaticFlashStatus,
    UnitFaultMonitor,
    UnitFreeCoordStatus,
    UnitOffset1,
    UnitOffset2,
    UnitOffset3,
    UnitPreemptCRCEnabledStatus,
    UnitTBCAux1,
    UnitTBCAux2,
    UnitTBCAux3,
    UnitTimingPlanA,
    UnitTimingPlanB,
    UnitTimingPlanC,
    UnitTimingPlanD,
    UnitVoltageMonitor,
    VehDetectorReset,
    WatchDog
};

template<typename T>
concept IoFuncKindC = std::is_same_v<T, InputFuncKind> || std::is_same_v<T, OutputFuncKind>;

template<auto IoFuncKind, size_t IoFuncNumber = 0>
    requires IoFuncKindC<decltype(IoFuncKind)>
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
concept IoFuncC = std::is_same_v<F, IoFunc<F::kind, F::number>>;

template<typename F>
concept InputFuncC = IoFuncC<F> && std::is_same_v<typename F::KindT, InputFuncKind>;

template<typename F>
concept OutputFuncC = IoFuncC<F> && std::is_same_v<typename F::KindT, OutputFuncKind>;

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

template<IoKind IoKind, size_t IoNumber, IoFuncC F>
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
template<IoKind IoKind, size_t IoNumber, IoFuncC F>
struct Io : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

// Specialization for Input Function
template<IoKind IoKind, size_t IoNumber, InputFuncC F>
    requires(IoKind == basic_types::IoKind::Input || IoKind == basic_types::IoKind::OptoInput)
struct Io<IoKind, IoNumber, F> : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

// Specialization for Output Function
template<IoKind IoKind, size_t IoNumber, OutputFuncC F>
    requires(IoKind == basic_types::IoKind::Output)
struct Io<IoKind, IoNumber, F> : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

// Specialization for IoKind::InputOutput (both Input or Output Function)
template<IoKind IoKind, size_t IoNumber, IoFuncC F>
    requires(IoKind == basic_types::IoKind::InputOutput)
struct Io<IoKind, IoNumber, F> : CustomIo<IoKind, IoNumber, F>
{
    explicit Io(std::atomic_bool &s) : CustomIo<IoKind, IoNumber, F>(s)
    {}
};

}// namespace basic_types

namespace func {

using namespace vtc::io::basic_types;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_VEHICLE_DETECTORS)
using VehicleDetectorCall = IoFunc<InputFuncKind::VehicleDetectorCall, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using ChannelGreenWalkDriver = IoFunc<OutputFuncKind::ChannelGreenWalkDriver, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using ChannelYellowPedClearDriver = IoFunc<OutputFuncKind::ChannelYellowPedClearDriver, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using ChannelRedDoNotWalkDriver = IoFunc<OutputFuncKind::ChannelRedDoNotWalkDriver, FuncNumber>;

}// namespace fn

namespace biu {

using namespace vtc::io::basic_types;

enum class BiuVariableKind
{
    DateTimeBroadcastMonth,
    DateTimeBroadcastDay,
    DateTimeBroadcastYear,
    DateTimeBroadcastHour,
    DateTimeBroadcastMinute,
    DateTimeBroadcastSecond,
    DateTimeBroadcastDecisecond,
    DetectorTimestamp,
    DetectorWatchDogFailureState,
    DetectorOpenLoopState,
    DetectorShortLoopState,
    DetectorExcessiveInductanceChange,
    MessageData,
    TFBiuPresent,
    DetBiuPresent
};

// BIU or SIU internal variable for generating response frame.
template<BiuVariableKind VarKind, size_t VarNumber, typename DataT>
struct BiuVariable
{
    static constexpr auto var_kind{VarKind};
    static constexpr auto var_number{VarNumber};
    using data_type = DataT;

    DataT data;
};

template<size_t InputNumber>
concept ValidBiuInputNumberC = InputNumber > 1 && InputNumber <= 8;// NEMA-TS2 BIU supports 8 inputs

template<size_t InputNumber, IoFuncC F>
    requires ValidBiuInputNumberC<InputNumber>
using BiuInput = Io<IoKind::Input, InputNumber, F>;

template<size_t OutputNumber>
concept ValidBiuOutputNumberC = OutputNumber > 1 && OutputNumber <= 15;// NEMA-TS2 BIU supports 15 outputs

template<size_t OutputNumber, IoFuncC F>
    requires ValidBiuOutputNumberC<OutputNumber>
using BiuOutput = Io<IoKind::Output, OutputNumber, F>;

template<size_t OptoInputNumber>
concept ValidOptoInputNumberC = OptoInputNumber > 1 && OptoInputNumber <= 4;

template<size_t OptoInputNumber, IoFuncC F>
    requires ValidOptoInputNumberC<OptoInputNumber>
using BiuOptoInput = Io<IoKind::OptoInput, OptoInputNumber, F>;

template<size_t InputOutputNumber>
concept ValidInputOutputNumberC = InputOutputNumber > 1 && InputOutputNumber <= 24;

template<size_t InputOutputNumber, IoFuncC F>
    requires ValidInputOutputNumberC<InputOutputNumber>
using BiuInputOutput = Io<IoKind::InputOutput, InputOutputNumber, F>;

}// namespace biu

}// namespace vtc::io