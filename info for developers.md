# Documentation for developers

Some more detailed information about the structure of the Clarvi code. To be expanded.

## Use of SystemVerilog

A general remark: the code employs SystemVerilog rather than just Verilog features where possible. In particular, `logic` and user-defined types are prefered to `reg`/`wire` and `always_comb` is prefered to `assign` statements.

However, this is at times constrained by poor support from Altera synthesis tools for certain constructs. For instance this means that some declarations are earlier than I would like, union types are not used although they would be convenient, and `logic` is used instead of custom types in many places.

## General Structure

The Clarvi code is in two halves. The first half is roughly divided up into sections corresponding to pipeline stages, with declarations and process blocks (always_comb and always_ff) that correspond to the signals produced by that stage.

Combinational functions and occasionally tasks are also used heavily. These are located in the second half of the code, below the per-stage declarations and process blocks. Beware that some functions/tasks access state declared in the first half of the code in addition to their arguments, merely for convenience.

The choice to use functions instead of separate modules was made to reduce the amount of wiring-up boilerplate code required, admitedly at the cost of some degree of isolation between components.

## Debugging

There are two debugging aspects to the code:

1.  The `clarvi_debug.sv` code is included in the main `clarvi.sv` file when run in ModelSim. It contains code for stopping the simulator in certain conditions, such as an invalid instruction or an `ecall`, and for outputting an instruction trace.

    The debugging code generally looks at the contents of the MA stage, since at this point the all information about the instruction's execution is known. However, for stopping the simulator, it is more sensible to look at the contents of the WB stage, so that the preceeding instruction is given the chance to write back first.

2.  The `sim.sv` testbench includes some modules for debugging or testing various aspects of the core from an external perspective.

    The `memory_debug` module makes it easy to catch reads/writes, for instance to output debugging info when particular addresses are accessed.

    There are also `mock_waitrequest` and `mock_interrupt` modules to test out the behaviour of the core under memory contention and when interrupts occur.
