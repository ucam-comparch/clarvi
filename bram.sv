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


`timescale 1ns/10ps

module dual_port_bram #(
    parameter INIT_FILE = "mem.hex",
    parameter ADDRESS_WIDTH = 12,
    parameter BYTE_WIDTH = 8,
    parameter BYTES_PER_WORD = 4
)(
    input logic clock,
    input logic reset,

    input logic  [ADDRESS_WIDTH-1:0]                  avs_a_address,
    input logic  [BYTES_PER_WORD-1:0]                 avs_a_byteenable,
    input logic                                       avs_a_read,
    output logic [BYTES_PER_WORD-1:0][BYTE_WIDTH-1:0] avs_a_readdata,
    input logic                                       avs_a_write,
    input logic  [BYTES_PER_WORD-1:0][BYTE_WIDTH-1:0] avs_a_writedata,

    input logic  [ADDRESS_WIDTH-1:0]                  avs_b_address,
    input logic  [BYTES_PER_WORD-1:0]                 avs_b_byteenable,
    input logic                                       avs_b_read,
    output logic [BYTES_PER_WORD-1:0][BYTE_WIDTH-1:0] avs_b_readdata,
    input logic                                       avs_b_write,
    input logic  [BYTES_PER_WORD-1:0][BYTE_WIDTH-1:0] avs_b_writedata
);

    // model the RAM with an array of 2D arrays, each representing one word
    logic [BYTES_PER_WORD-1:0][BYTE_WIDTH-1:0] memory [1 << ADDRESS_WIDTH];
    initial $readmemh(INIT_FILE, memory);

    // port A
    always_ff @(posedge clock) begin
        if (avs_a_write)
            for (int i = 0; i < BYTES_PER_WORD; i++)
                if (avs_a_byteenable[i])
                    memory[avs_a_address][i] <= avs_a_writedata[i];
        if (avs_a_read)
            avs_a_readdata <= memory[avs_a_address];
    end

    // port B
    always_ff @(posedge clock) begin
        if (avs_b_write)
            for (int i = 0; i < BYTES_PER_WORD; i++)
                if (avs_b_byteenable[i])
                    memory[avs_b_address][i] <= avs_b_writedata[i];
        if (avs_b_read)
            avs_b_readdata <= memory[avs_b_address];
    end

endmodule
