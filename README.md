# Clarvi

A simple RISC-V implementation in SystemVerilog. Intended to be clear and understandable for teaching purposes, while maintaining good performance.

This is an RV32I core. It provides a minimum implementation of the v1.9 RISC-V privileged specification, including full support for interrupts and exceptions.
Only machine mode is implemented, and there is only partial support for catching invalid instructions.
There is also support for the full v2.1 user-level specification.

It passes all RV32I user-level tests, and the relevant RV32I machine-mode tests (but not those which assume support for user mode, virtual memory etc).

There are minor deviations from the privileged specification:
- On a branch to an invalid or misaligned address, an exception is only raised on the subsequent fetch, so any side effects of the branch (e.g. for JALR) are maintained.
  The specification is ambiguous on this matter, which causes the `ma_fetch` test to fail.
- The `timecmp` register is implemented as a CSR rather than being memory mapped - its address can be found in `riscv.svh`.
- The `dscratch` register is adopted as means of producing debug output.

It is possible to disable machine mode by undefining `MACHINE_MODE`. This removes support for all privileged instructions, interrupts, exceptions and handling of illegal instructions, but provides a reasonable frequency increase.

The processor has a 6-stage pipeline, which is similar to the classic RISC pipeline, with a few differences:
- The intruction fetch is 2-stage, to accomodate on-chip memory with a fixed 1 cycle latency
- Loads and Stores are initiated in the Execute stage, while load results are aligned in the Memory Align stage
- Jumps are computed in the Execute stage, meaning a 3-cycle penalty for any taken branch.


## Memory Requirements

The core has a generic memory interface, with separate instruction and data ports. These could be connected to the same or different memories. Be sure to configure the `ADDR_WIDTH` parameters appropriately for the amount of memory attached - accesses beyond this address width will raise access faults. Note that this is the **word** address width. All memory bus addresses are word not byte addresses.

The included `clarvi_avalon` component is a wrapper which exposes an Altera Avalon MM interface. Although it includes the `readdatavalid` signal, the core expects memory with a fixed single cycle latency. It does however support the use of the `wait` signal when there is contention on the memory bus.

To attach memory with a longer or variable latency, a wrapper component should be created which uses the core's `wait` signal to stall it when memory access will have a latency of more than 1 cycle. The `wait` signal is expected to behave like the Avalon MM `waitrequest` signal.


## Performance

On a Cyclone V board using on-chip BRAM as memory, 100 to 150Mhz can be achieved depending on the amount of RAM and whether machine mode is enabled.

CPI is as follows:

- 4: Branch taken
- 2: Load followed by dependent instruction
- 1: All other instructions

Average CPI is therefore roughly 1.5, depending on how branch-heavy the code is.


## What's in the box

This repository includes the following:

### Main source files

- The CPU source code is in `clarvi.sv`. It includes `riscv.svh` which contains various struct, enumeration and constant definitions.
- Testbench code is in `sim.sv`. It relies upon the included `bram` testbench component which provides a mockup of a dual-port on-chip memory.

### Software and Tests

A makefile, linker script and `init` program is provided. Software is built into a `.mem.txt` file supported by ModelSim, and a `.mem.hex` file which is an Intel HEX format memory image, supported by Altera Quartus/Qsys.

To simulate the processor in ModelSim, run the command `do sim.tcl <path to .mem.txt file>`.

If you want to see a full instruction trace, run `do sim.tcl <path to .mem.txt file> TRACE`.

The RISC-V test suite can be run by building the tests with the makefile provided in the tests directory.
You may have to correct the path to your `riscv-tests` repository location in the makefile.

Also included is a 'b' (bare) test environment, needed to run the tests with machine mode disabled, and a custom linker script.

There are scripts provided to run the tests using ModelSim:
- `./tests-all.sh` runs all tests
- `./tests-all.sh <environment>` runs all tests with the given environment. For instance `./tests-all.sh b` runs all tests configured for machine mode disabled
- Inside modelsim, an individual test can be run with the command `do test.tcl <path to .mem.txt file>`.

The tests may also be run in hardware. To help identify when a test ends and its result, the test result register (x28) is exposed on a debug port.
