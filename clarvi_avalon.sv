/*******************************************************************************
Copyright (c) 2016, Robert Eady
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


// This is an Avalon MM wrapper for clarvi.

`timescale 1ns/10ps

module clarvi_avalon #(
    parameter DATA_ADDR_WIDTH = 14,
    parameter INSTR_ADDR_WIDTH = 14,
    parameter INITIAL_PC = 0,
    parameter DEFAULT_TRAP_VECTOR = 0
)(
    input logic clock,
    input logic reset,

    // data memory port (read/write)
    output logic [DATA_ADDR_WIDTH-1:0] avm_main_address,
    output logic [3:0]  avm_main_byteenable,
    output logic        avm_main_read,
    input  logic [31:0] avm_main_readdata,
    output logic        avm_main_write,
    output logic [31:0] avm_main_writedata,
    input  logic        avm_main_waitrequest,
    input  logic        avm_main_readdatavalid,

    // instruction memory port (read-only)
    output logic [INSTR_ADDR_WIDTH-1:0] avm_instr_address,
    output logic        avm_instr_read,
    input  logic [31:0] avm_instr_readdata,
    input  logic        avm_instr_waitrequest,
    input  logic        avm_instr_readdatavalid,

    // external interrupt signal, active high
    input  logic        inr_irq,

    // debug ports
    output logic [31:0] debug_register28,
    output logic [31:0] debug_scratch,
    output logic [31:0] debug_pc
);

    // We only have readdatavalid signal to allow the use of pipelining with a fixed 1-cycle latency memory.
    // Longer or variable latency slaves are not supported!

    clarvi #(
        .DATA_ADDR_WIDTH(DATA_ADDR_WIDTH),
        .INSTR_ADDR_WIDTH(INSTR_ADDR_WIDTH),
        .INITIAL_PC(INITIAL_PC),
        .DEFAULT_TRAP_VECTOR(DEFAULT_TRAP_VECTOR)
    ) clarvi (
        .main_address          (avm_main_address),
        .main_byte_enable      (avm_main_byteenable),
        .main_read_enable      (avm_main_read),
        .main_read_data        (avm_main_readdata),
        .main_read_data_valid  (avm_main_readdatavalid),
        .main_write_enable     (avm_main_write),
        .main_write_data       (avm_main_writedata),
        .main_wait             (avm_main_waitrequest),
        .instr_address         (avm_instr_address),
        .instr_read_enable     (avm_instr_read),
        .instr_read_data       (avm_instr_readdata),
        .instr_wait            (avm_instr_waitrequest),
        .clock                 (clock),
        .reset                 (reset),
        .inr_irq               (inr_irq),
        .debug_register28      (debug_register28),
        .debug_scratch         (debug_scratch),
        .debug_pc              (debug_pc)
    );

endmodule
