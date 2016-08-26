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


// instruction parts, that are always in the same place
`define opext  [1 : 0]
`define opcode [6 : 2]
`define rd     [11: 7]
`define funct3 [14:12]
`define rs1    [19:15]
`define rs2    [24:20]
`define funct7 [31:25]
`define funct12 [31:20]


// RISC-V opcodes
typedef enum logic [4:0] {
    OPC_LUI      = 5'b01101,
    OPC_AUIPC    = 5'b00101,
    OPC_JAL      = 5'b11011,
    OPC_JALR     = 5'b11001,
    OPC_BRANCH   = 5'b11000,
    OPC_LOAD     = 5'b00000,
    OPC_STORE    = 5'b01000,
    OPC_OP_IMM   = 5'b00100,
    OPC_OP       = 5'b01100,
    OPC_MISC_MEM = 5'b00011,
    OPC_SYSTEM   = 5'b11100
} opcode_t;


// registers
typedef enum logic [4:0] {
    zero,
    ra,
    sp,
    gp,
    tp,
    t0, t1, t2,
    fp, s1,
    a0, a1, a2, a3, a4, a5, a6, a7,
    s2, s3, s4, s5, s6, s7, s8, s9, s10, s11,
    t3, t4, t5, t6
} register_t;

// internal, decoded opcodes
typedef enum logic [4:0] {
    LUI,
    AUIPC,
    JAL,
    JALR,
    BEQ,
    BNE,
    BLT,
    BGE,
    BLTU,
    BGEU,
    LOAD,
    STORE,
    ADD,
    SUB,
    SLT,
    SLTU,
    XOR,
    OR,
    AND,
    SL,
    SRL,
    SRA,
    FENCE,
    FENCE_I,
    ECALL,
    EBREAK,
    MRET,
    WFI,
    CSRRW,
    CSRRS,
    CSRRC,
    INVALID
} operation_t;

// memory operation widths
typedef enum logic [1:0] {
    B = 2'b00,
    H = 2'b01,
    W = 2'b10
} mem_width_t;

// machine mode funct12 codes
typedef enum logic [11:0] {
    F12_ECALL  = 12'b000000000000,
    F12_EBREAK = 12'b000000000001,
    F12_MRET   = 12'b001100000010,
    F12_WFI    = 12'b000100000101
} funct12_t;

// holds all the information about a decoded instruction
typedef struct packed {
    operation_t  op;
    register_t   rd;
    register_t   rs1;
    register_t   rs2;
    logic        rs1_used;
    logic        rs2_used;
    logic        immediate_used;
    logic [31:0] immediate;
    funct12_t    funct12;
    logic        memory_write;
    logic        memory_read;
    logic        memory_read_unsigned;
    mem_width_t  memory_width;
    logic        enable_wb;
    logic [31:0] pc;
} instr_t;


// all the funct3 codes

typedef enum logic [2:0] {
    F3_ADDSUB = 3'b000,
    F3_SLT    = 3'b010,
    F3_SLTU   = 3'b011,
    F3_XOR    = 3'b100,
    F3_OR     = 3'b110,
    F3_AND    = 3'b111,
    F3_SLL    = 3'b001,
    F3_SR     = 3'b101
} funct3_op_t;

typedef enum logic [2:0] {
    F3_BEQ  = 3'b000,
    F3_BNE  = 3'b001,
    F3_BLT  = 3'b100,
    F3_BGE  = 3'b101,
    F3_BLTU = 3'b110,
    F3_BGEU = 3'b111
} funct3_branch_t;

typedef enum logic [2:0] {
    F3_LB  = 3'b000,
    F3_LH  = 3'b001,
    F3_LW  = 3'b010,
    F3_LBU = 3'b100,
    F3_LHU = 3'b101
} funct3_load_t;

typedef enum logic [2:0] {
    F3_SB  = 3'b000,
    F3_SH  = 3'b001,
    F3_SW  = 3'b010
} funct3_store_t;

typedef enum logic [2:0] {
    F3_FENCE  = 3'b000,
    F3_FENCEI = 3'b001
} funct3_misc_mem_t;

typedef enum logic [2:0] {
    F3_CSRRW  = 3'b001,
    F3_CSRRS  = 3'b010,
    F3_CSRRC  = 3'b011,
    F3_CSRRWI = 3'b101,
    F3_CSRRSI = 3'b110,
    F3_CSRRCI = 3'b111,
    F3_PRIV   = 3'b000
} funct3_system_t;

// non-immediate part of the system funct3 codes
typedef enum logic [1:0] {
    F2_PRIV  = 2'b00,
    F2_CSRRW = 2'b01,
    F2_CSRRS = 2'b10,
    F2_CSRRC = 2'b11
} funct2_system_t;


// CSR addresses
typedef enum logic [11:0] {
    CYCLE     = 12'hC00,
    TIME      = 12'hC01,
    INSTRET   = 12'hC02,
    CYCLEH    = 12'hC80,
    TIMEH     = 12'hC81,
    INSTRETH  = 12'hC82,

    MISA      = 12'hF10,
    MVENDORID = 12'hF11,
    MARCHID   = 12'hF12,
    MIMPID    = 12'hF13,
    MHARTID   = 12'hF14,

    MSTATUS   = 12'h300,
    MEDELEG   = 12'h302,
    MIDELEG   = 12'h303,
    MIE       = 12'h304,
    MTVEC     = 12'h305,

    MSCRATCH  = 12'h340,
    MEPC      = 12'h341,
    MCAUSE    = 12'h342,
    MBADADDR  = 12'h343,
    MIP       = 12'h344,

    MCYCLE    = 12'hF00,
    MTIME     = 12'hF01,
    MINSTRET  = 12'hF02,
    MCYCLEH   = 12'hF80,
    MTIMEH    = 12'hF81,
    MINSTRETH = 12'hF82,

    // non-standard but we don't want to memory-map mtimecmp
    MTIMECMP  = 12'h7C1,
    MTIMECMPH = 12'h7C2,

    // provisional debug CSRs
    DCSR      = 12'h7B0,
    DPC       = 12'h7B1,
    DSCRATCH  = 12'h7B2
} csr_t;


// mstatus register
typedef struct packed {
    logic       sd;
    logic [1:0] unused1;
    logic [4:0] vm;
    logic [3:0] unused2;
    logic       mxr;
    logic       pum;
    logic       mprv;
    logic [1:0] xs;
    logic [1:0] fs;
    logic [1:0] mpp;
    logic [1:0] hpp;
    logic       spp;
    logic       mpie;
    logic       hpie;
    logic       spie;
    logic       upie;
    logic       mie;
    logic       hie;
    logic       sie;
    logic       uie;
} mstatus_t;


// mcause register
typedef enum logic [31:0] {
    // interrupt codes have the top bit set to 1.
    USI = {1'b1, 31'd0},
    SSI = {1'b1, 31'd1},
    HSI = {1'b1, 31'd2},
    MSI = {1'b1, 31'd3},
    UTI = {1'b1, 31'd4},
    STI = {1'b1, 31'd5},
    HTI = {1'b1, 31'd6},
    MTI = {1'b1, 31'd7},
    UEI = {1'b1, 31'd8},
    SEI = {1'b1, 31'd9},
    HEI = {1'b1, 31'd10},
    MEI = {1'b1, 31'd11},

    INSTR_MISALIGN = 32'd0,
    INSTR_FAULT    = 32'd1,
    ILLEGAL_INSTR  = 32'd2,
    BREAK          = 32'd3,
    LOAD_MISALIGN  = 32'd4,
    LOAD_FAULT     = 32'd5,
    STORE_MISALIGN = 32'd6,
    STORE_FAULT    = 32'd7,
    ECALL_U        = 32'd8,
    ECALL_S        = 32'd9,
    ECALL_H        = 32'd10,
    ECALL_M        = 32'd11
} mcause_t;


// machine interrupts pending register
typedef struct packed {
    logic [19:0] unused;
    logic meip;
    logic heip;
    logic seip;
    logic ueip;
    logic mtip;
    logic htip;
    logic stip;
    logic utip;
    logic msip;
    logic hsip;
    logic ssip;
    logic usip;
} mip_t;

// machine interrupts enabled register
typedef struct packed {
    logic [19:0] unused;
    logic meie;
    logic heie;
    logic seie;
    logic ueie;
    logic mtie;
    logic htie;
    logic stie;
    logic utie;
    logic msie;
    logic hsie;
    logic ssie;
    logic usie;
} mie_t;
