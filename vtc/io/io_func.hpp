#pragma once

#include "io/io_basic_types.hpp"
#include "io_basic_types.hpp"

namespace vtc::io::func {

using namespace vtc::io::basic_types;

namespace output {

//----------------------------------------
// LoadSwitch Drivers
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using LoadSwitchGreenDriver = IoFunc<OutputFuncKind::LoadSwitchGreenDriver, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using LoadSwitchYellowDriver = IoFunc<OutputFuncKind::LoadSwitchYellowDriver, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_LOAD_SWITCH_CHANNELS)
using LoadSwitchRedDriver = IoFunc<OutputFuncKind::LoadSwitchRedDriver, FuncNumber>;

//----------------------------------------
// Free (No Coord) Status
//----------------------------------------
using FreeNoCoordStatus = IoFunc<OutputFuncKind::FreeNoCoordStatus, 0>;

//----------------------------------------
// Offsets
//----------------------------------------
using Offset1 = IoFunc<OutputFuncKind::Offset1, 0>;
using Offset2 = IoFunc<OutputFuncKind::Offset2, 0>;
using Offset3 = IoFunc<OutputFuncKind::Offset3, 0>;

//----------------------------------------
// Phase Check
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PHASES)
using PhaseCheck = IoFunc<OutputFuncKind::PhaseCheck, FuncNumber>;

//----------------------------------------
// Phase Next
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PHASES)
using PhaseNext = IoFunc<OutputFuncKind::PhaseNext, FuncNumber>;

//----------------------------------------
// Phase On
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PHASES)
using PhaseOn = IoFunc<OutputFuncKind::PhaseOn, FuncNumber>;

//----------------------------------------
// Preempt Status
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PREEMPTS)
using PreemptStatus = IoFunc<OutputFuncKind::PreemptStatus, FuncNumber>;

//----------------------------------------
// Coded Ring Status Bits
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingStatusBitA = IoFunc<OutputFuncKind::RingStatusBitA, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingStatusBitB = IoFunc<OutputFuncKind::RingStatusBitB, FuncNumber>;

template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingStatusBitC = IoFunc<OutputFuncKind::RingStatusBitC, FuncNumber>;

//----------------------------------------
// System Special Functions
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_SYSTEM_SPECIAL_FUNCTIONS)
using SystemSpecialFunction = IoFunc<OutputFuncKind::SystemSpecialFunction, FuncNumber>;

//----------------------------------------
// Time Based Control (TBC) Auxiliary
//----------------------------------------
using TBCAuxiliary1 = IoFunc<OutputFuncKind::TBCAuxiliary1, 0>;
using TBCAuxiliary2 = IoFunc<OutputFuncKind::TBCAuxiliary2, 0>;
using TBCAuxiliary3 = IoFunc<OutputFuncKind::TBCAuxiliary3, 0>;

//----------------------------------------
// Time Plan Bits
//----------------------------------------
using TimingPlanA = IoFunc<OutputFuncKind::TimingPlanA, 0>;
using TimingPlanB = IoFunc<OutputFuncKind::TimingPlanB, 0>;
using TimingPlanC = IoFunc<OutputFuncKind::TimingPlanC, 0>;
using TimingPlanD = IoFunc<OutputFuncKind::TimingPlanD, 0>;

//----------------------------------------
// Vehicle Detector Reset
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_DETECTOR_RACK_SLOT_GROUPS)
using VehicleDetectorReset = IoFunc<OutputFuncKind::VehicleDetectorReset, FuncNumber>;

}// namespace output

namespace input {

//----------------------------------------
// Address Bits
//----------------------------------------
using AddressBit0 = IoFunc<InputFuncKind::SystemAddressBit0, 0>;
using AddressBit1 = IoFunc<InputFuncKind::SystemAddressBit1, 0>;
using AddressBit2 = IoFunc<InputFuncKind::SystemAddressBit2, 0>;
using AddressBit3 = IoFunc<InputFuncKind::SystemAddressBit3, 0>;

//----------------------------------------
// Alarm
//----------------------------------------
using Alarm1 = IoFunc<InputFuncKind::Alarm1, 0>;
using Alarm2 = IoFunc<InputFuncKind::Alarm2, 0>;

//----------------------------------------
// Alternate Sequence
//----------------------------------------
using AlternateSequenceA = IoFunc<InputFuncKind::AlternateSequenceA, 0>;
using AlternateSequenceB = IoFunc<InputFuncKind::AlternateSequenceB, 0>;
using AlternateSequenceC = IoFunc<InputFuncKind::AlternateSequenceC, 0>;
using AlternateSequenceD = IoFunc<InputFuncKind::AlternateSequenceD, 0>;

//----------------------------------------
// Automatic Flash
//----------------------------------------
using AutomaticFlash = IoFunc<InputFuncKind::AutomaticFlash, 0>;

//----------------------------------------
// CNA_I
//----------------------------------------
using CallToNonActuated1 = IoFunc<InputFuncKind::CallToNonActuated1, 0>;

//----------------------------------------
// CNA_II
//----------------------------------------
using CallToNonActuated2 = IoFunc<InputFuncKind::CallToNonActuated2, 0>;

//----------------------------------------
// Dimming Enable
//----------------------------------------
using DimmingEnable = IoFunc<InputFuncKind::DimmingEnable, 0>;

//----------------------------------------
// External Minimum Recall
//----------------------------------------
using ExternalMinimumRecall = IoFunc<InputFuncKind::ExternalMinimumRecall, 0>;

//----------------------------------------
// External Start
//----------------------------------------
using ExternalStart = IoFunc<InputFuncKind::ExternalStart, 0>;

//----------------------------------------
// Free (NoCoord) Input
//----------------------------------------
using FreeNoCoord = IoFunc<InputFuncKind::FreeNoCoord, 0>;

//----------------------------------------
// Interval Advance Input
//----------------------------------------
using IntervalAdvance = IoFunc<InputFuncKind::IntervalAdvance, 0>;

//----------------------------------------
// Local Flash Status
//----------------------------------------
using LocalFlashStatus = IoFunc<InputFuncKind::LocalFlashStatus, 0>;

//----------------------------------------
// Manual Control Enable
//----------------------------------------
using ManualControlEnable = IoFunc<InputFuncKind::ManualControlEnable, 0>;

//----------------------------------------
// MMU Flash Status
//----------------------------------------
using MmuFlashStatus = IoFunc<InputFuncKind::MmuFlashStatus, 0>;

//----------------------------------------
// Offset
//----------------------------------------
using Offset1 = IoFunc<InputFuncKind::Offset1, 0>;
using Offset2 = IoFunc<InputFuncKind::Offset2, 0>;
using Offset3 = IoFunc<InputFuncKind::Offset3, 0>;

//----------------------------------------
// Force Off
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingForceOff = IoFunc<InputFuncKind::RingForceOff, FuncNumber>;

//----------------------------------------
// Max II Selection
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingMax2Selection = IoFunc<InputFuncKind::RingMax2Selection, FuncNumber>;

//----------------------------------------
// Inhibit MAX Termination
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingInhibitMaxTermination = IoFunc<InputFuncKind::RingInhibitMaxTermination, FuncNumber>;

//----------------------------------------
// Stop Timing
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= 2)
using RingStopTiming = IoFunc<InputFuncKind::RingStopTiming, FuncNumber>;

//----------------------------------------
// Phase Hold
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PHASES)
using PhaseHold = IoFunc<InputFuncKind::PhaseHold, FuncNumber>;

//----------------------------------------
// Phase Omit
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PHASES)
using PhaseOmit = IoFunc<InputFuncKind::PhaseOmit, FuncNumber>;

//----------------------------------------
// Preempt Detector
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PREEMPTS)
using PreemptDetector = IoFunc<InputFuncKind::PreemptDetector, FuncNumber>;

//----------------------------------------
// Pedestrian Detector
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_PEDESTRIAN_DETECTORS)
using PedestrianDetector = IoFunc<InputFuncKind::PedestrianDetector, FuncNumber>;

//----------------------------------------
// TBC Online
//----------------------------------------
using TBCOnline = IoFunc<InputFuncKind::TBCOnline, 0>;

//----------------------------------------
// Test Inputs
//----------------------------------------
using TestA = IoFunc<InputFuncKind::TestA, 0>;
using TestB = IoFunc<InputFuncKind::TestB, 0>;
using TestC = IoFunc<InputFuncKind::TestC, 0>;

//----------------------------------------
// Timing Plan
//----------------------------------------
using TimingPlanA = IoFunc<InputFuncKind::TimingPlanA, 0>;
using TimingPlanB = IoFunc<InputFuncKind::TimingPlanB, 0>;
using TimingPlanC = IoFunc<InputFuncKind::TimingPlanC, 0>;
using TimingPlanD = IoFunc<InputFuncKind::TimingPlanD, 0>;

//----------------------------------------
// Walk Rest Modifier
//----------------------------------------
using WalkRestModifier = IoFunc<InputFuncKind::WalkRestModifier, 0>;

//----------------------------------------
// Vehicle Detector Call
//----------------------------------------
template<size_t FuncNumber>
    requires(FuncNumber >= 1) && (FuncNumber <= MAX_VEHICLE_DETECTORS)
using VehicleDetectorCall = IoFunc<InputFuncKind::VehicleDetectorCall, FuncNumber>;
}// namespace input

}// namespace vtc::io::func