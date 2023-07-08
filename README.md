# Virtual Traffic Cabinet 2

Virtual Traffic Cabinet (VTC) 2 represents a notable advancement over the [original VTC framework](https://github.com/wxinix/vtc), providing an enhanced virtualization solution in modern C++20 for traffic cabinets in accordance with the NEMA-TS2 and ATC Standards. 

VTC 2 introduces a higher level of abstraction compared to its predecessor, enabling the seamless integration and manipulation of various NEMA-TS2 and ATC elements. This advanced level of abstraction empowers developers to create versatile and software-defined traffic cabinets and traffic signal controllers. By leveraging the capabilities of VTC 2, engineers can design and implement generic solutions that are adaptable to diverse traffic management scenarios.

## VTC2 Design

One crucial aspect of a traffic cabinet is its pre-defined set of Inputs and Outputs (IOs), which determine how the cabinet's components interact with each other and with the external traffic signal control system. In modern NEMA-TS2 or ATC cabinets, these IOs are named entities representing logical inputs and outputs. For example, "VehicleDetCall" denotes the input for a vehicle detector call on a specific channel. If the Controller Unit (CU) supports up to 64 detector channels, the CU indexes "VehicleDetCall" from 1 to 64.

It's important to note that the same IO notation can be logically applied in the context of different devices. The Controller Unit (CU) manages 64 Vehicle Detector Call channels, while the Detector Bus Interface (BIU) device also handles Vehicle Detector Calls. As a result, the same IO entity can have logical bindings to different devices. In VTC2 framework, this is modeled as IO binding - the same IO can be bound to different devices.

When virtualizing a traffic cabinet and its devices, there is often a significant amount of overlap and redundancy among different cabinet components. To minimize code redundancy and enhance elegance and performance, the VTC2 framework heavily relies on C++ templates. By utilizing templates, the framework generates specialized code for various IO types, reducing the need for duplicated code while maintaining a high level of abstraction, flexibility, and efficiency. The C++ meta template programming technique also enables a functional and declarative approach to composing different cabinet elements and devices, which is essential for a Software Defined Virtual Cabinet.

### IO Variable

The IO Variable serves as the foundational component of the VTC framework, acting as an Output, Input, or both, depending on its binding. For instance, the Load switch driver functions as an Output from the Control Unit (CU) and an Input to the Memory Management Unit (MMU). Additionally, an IO Variable can be associated with an index, providing logical differentiation for variables of the same kind. For example, Phase Next can be indexed to represent different phases.

In Object-Oriented Programming or Procedural Programming paradigms, one might consider using an array or list to manage indexed IO variables. However, VTC2 adopts a different approach to achieve higher abstraction and avoid runtime arrays. The use of runtime arrays would make it challenging to implement a declarative API style for VTC2. Instead, VTC2 incorporates the `Index`, `IO binding`, and `Cabinet`t affinity as non-type template arguments, effectively making each indexed IO variable a distinct type.

### Frame

The defining feature of the VTC framework lies in its remarkable expressiveness and the ability to define various frame types in a declarative style.  Frame serialization and deserialization is handled by the framework generically without the need to write individual parsers for individual frame types.

For example: 

```
// ----------------------------------------------
// Frame Type 13
// ----------------------------------------------
using TfBiu04_OutputsInputsRequestFrame
= Frame<
    0x03, // TF BIU#1 Address = 3
    0x0D, // FrameID = 13
    8,    // 8 Bytes
    SSR_CommandFrameType,
    // ----------------------------------------------
    // Byte 3
    // ----------------------------------------------
    FrameBit<io::output::PhaseOn<1>, 0x18>,
    FrameBit<io::output::PhaseOn<2>, 0x19>,
    FrameBit<io::output::PhaseOn<3>, 0x1A>,
    FrameBit<io::output::PhaseOn<4>, 0x1B>,
    FrameBit<io::output::PhaseOn<5>, 0x1C>,
    FrameBit<io::output::PhaseOn<6>, 0x1D>,
    FrameBit<io::output::PhaseOn<7>, 0x1E>,
    FrameBit<io::output::PhaseOn<8>, 0x1F>,
    // ----------------------------------------------
    // Byte 4
    // ----------------------------------------------
    FrameBit<io::output::PhaseNext<1>, 0x20>,
    FrameBit<io::output::PhaseNext<2>, 0x21>,
    FrameBit<io::output::PhaseNext<3>, 0x22>,
    FrameBit<io::output::PhaseNext<4>, 0x23>,
    FrameBit<io::output::PhaseNext<5>, 0x24>,
    FrameBit<io::output::PhaseNext<6>, 0x25>,
    FrameBit<io::output::PhaseNext<7>, 0x26>,
    // 0x27 - Reserved.
    // ----------------------------------------------
    // Byte 5
    // ----------------------------------------------
    FrameBit<io::output::PhaseNext<8>, 0x28>,
    FrameBit<io::output::PhaseCheck<1>, 0x29>,
    FrameBit<io::output::PhaseCheck<2>, 0x2A>,
    FrameBit<io::output::PhaseCheck<3>, 0x2B>,
    FrameBit<io::output::PhaseCheck<4>, 0x2C>,
    FrameBit<io::output::PhaseCheck<5>, 0x2D>,
    FrameBit<io::output::PhaseCheck<6>, 0x2E>,
    FrameBit<io::output::PhaseCheck<7>, 0x2F>,
    // ----------------------------------------------
    // Byte 6
    // ----------------------------------------------
    FrameBit<io::output::PhaseCheck<8>, 0x30>
    // 0x31 - Designated Input
    // 0x32 - Designated Input
    // 0x33 - Designated Input
    // 0x34 - Designated Input
    // 0x35 - Designated Input
    // 0x36 - Spare
    // 0x37 - Spare
    // ----------------------------------------------
    // Byte 7
    // ----------------------------------------------
    // 0x38 - Spare
    // 0x39 - Spare
    // 0x3A - Spare
    // 0x3B - Designated Input
    // 0x3C - Designated Input
    // 0x3D - Designated Input
    // 0x3E - Designated Input
    // 0x3F - Designated Input
>;
```

### Global::instance Pattern

VTC2 employs the `singleton` pattern to reference each unique instance of IO variables, BIUs, MMUs, and CUs. For instance, to reference the global instance of `VehicleDetCall`` for Cabinet 1, Field IO, and Detector Channel 6, you can use ```io::Global::instance<VehicleDetCall<1, IoBinding::Fio, 6>>```. This strong-typed approach eliminates the need for runtime index searching, with direct ```O(1)``` access to the corresponding global variable.

Note that `Cabinet` is one of the non-type template arguments. For X-in-the-Loop Simulation (X = hardware / software) there could be multiple cabinets with each having independent set of devices.

## Compiler
Requires a C++ compiler that supports C++20, and CMake build system.

## Example
To be added.