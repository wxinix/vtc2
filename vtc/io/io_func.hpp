#pragma once

#include "io/io_basic_types.hpp"

namespace vtc::io::func {

using namespace vtc::io::basic_types;



template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using LoadSwitchGreenDriver = IoFunc<OutputFuncKind::LoadSwitchGreenDriver, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using LoadSwitchYellowDriver = IoFunc<OutputFuncKind::LoadSwitchYellowDriver, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using LoadSwitchRedDriver = IoFunc<OutputFuncKind::LoadSwitchRedDriver, FuncNumber>;




template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_VEHICLE_DETECTORS)
using VehicleDetectorCall = IoFunc<InputFuncKind::VehicleDetectorCall, FuncNumber>;

}// namespace vtc::io::func