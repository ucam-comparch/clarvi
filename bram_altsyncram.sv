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

module dual_port_bram_altsyncram #(
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

    altsyncram #(
        .operation_mode  ("BIDIR_DUAL_PORT"),
        .width_a         (BYTE_WIDTH * BYTES_PER_WORD),
        .width_b         (BYTE_WIDTH * BYTES_PER_WORD),
        .widthad_a       (ADDRESS_WIDTH),
        .widthad_b       (ADDRESS_WIDTH),
        .outdata_reg_a   ("UNREGISTERED"),
        .outdata_reg_b   ("UNREGISTERED"),
        .width_byteena_a (BYTES_PER_WORD),
        .width_byteena_b (BYTES_PER_WORD),
        .byte_size       (BYTE_WIDTH),
        .init_file       (INIT_FILE),
        .address_reg_b   ("CLOCK0"),
        .byteena_reg_b   ("CLOCK0"),
        .indata_reg_b    ("CLOCK0"),
        .rdcontrol_reg_b ("CLOCK0"),
        .wrcontrol_wraddress_reg_b ("CLOCK0"),
        .intended_device_family ("Cyclone V")
    ) altsyncram_component (
        .clock0          (clock),
        .address_a       (avs_a_address),
        .address_b       (avs_b_address),
        .rden_a          (avs_a_read),
        .rden_b          (avs_b_read),
        .q_a             (avs_a_readdata),
        .q_b             (avs_b_readdata),
        .byteena_a       (avs_a_byteenable),
        .byteena_b       (avs_b_byteenable),
        .wren_a          (avs_a_write),
        .wren_b          (avs_b_write),
        .data_a          (avs_a_writedata),
        .data_b          (avs_b_writedata),
        .aclr0           (1'b0),
        .aclr1           (1'b0),
        .addressstall_a  (1'b0),
        .addressstall_b  (1'b0),
        .clock1          (1'b1),
        .clocken0        (1'b1),
        .clocken1        (1'b1),
        .clocken2        (1'b1),
        .clocken3        (1'b1),
        .eccstatus       ()
    );

endmodule
