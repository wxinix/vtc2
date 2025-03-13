#pragma once

#include "io/io_basic_types.hpp"

namespace vtc::io::biu {

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
concept IsValidBiuInputNumber = InputNumber > 1 && InputNumber <= 8;// NEMA-TS2 BIU supports 8 inputs

template<size_t InputNumber, IoFuncConcept F>
    requires IsValidBiuInputNumber<InputNumber>
using BiuInput = Io<IoKind::Input, InputNumber, F>;

template<size_t OutputNumber>
concept IsValidBiuOutputNumber = OutputNumber > 1 && OutputNumber <= 15;// NEMA-TS2 BIU supports 15 outputs

template<size_t OutputNumber, IoFuncConcept F>
    requires IsValidBiuOutputNumber<OutputNumber>
using BiuOutput = Io<IoKind::Output, OutputNumber, F>;

template<size_t OptoInputNumber>
concept IsValidOptoInputNumber = OptoInputNumber > 1 && OptoInputNumber <= 4;

template<size_t OptoInputNumber, IoFuncConcept F>
    requires IsValidOptoInputNumber<OptoInputNumber>
using BiuOptoInput = Io<IoKind::OptoInput, OptoInputNumber, F>;

template<size_t InputOutputNumber>
concept IsValidInputOutputNumber = InputOutputNumber > 1 && InputOutputNumber <= 24;

template<size_t InputOutputNumber, IoFuncConcept F>
    requires IsValidInputOutputNumber<InputOutputNumber>
using BiuInputOutput = Io<IoKind::InputOutput, InputOutputNumber, F>;

}// namespace vtc::io::biu