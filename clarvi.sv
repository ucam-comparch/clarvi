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

/*******************************************************************************

The processor has a 6 stage pipeline, with pipeline registers between stages.
Instruction fetch takes two cycles.

Note that memory accesses are submitted in the execute stage and loaded values
aligned in the memory align stage. Branches are performed in the execute stage.

There are forwarding paths from the output of the execute, memory align and
write back stages into the end of the decode stage.

                 IF/DE      DE/EX       EX/MA            MA/WB
Instruction Fetch --> Decode --> Execute --> Memory Align --> Write Back
                          ^-----------/---------------/---------------/

List of abbreviations/conventions:

instr: instruction              if: instruction fetch
pc:  program counter            de: decode
imm: immediate value            ex: execute
rd:  destination register       alu: arithmetic logic unit
rs1: source register 1          wb: write back
rs2: source register 2
avm: avalon master (memory interface)
inr: interrupt receiver

Pipeline register values are prefixed according to the stages they fall between,
e.g. de_ex_instr is a DE/EX pipeline register storing the decoded instruction.

Combination signals are prefixed with the stage they are used in,
e.g. ex_result is output of the ALU in the execute stage.

The core only supports single cycle latency instruction memory.
Main memory can have arbitrary (>= 1 cycle) latency.

*******************************************************************************/

`include "riscv.svh"

`define MACHINE_MODE    // enable support for machine mode instructions, interrupts and exceptions
`define DEBUG           // enable debug outputs
`ifdef MODEL_TECH
    `define SIMULATION  // enable simulation features
//  `define TRACE       // enable full instruction tracing in simulation.
`endif

`timescale 1ns/10ps

module clarvi #(
    parameter DATA_ADDR_WIDTH = 14,
    parameter INSTR_ADDR_WIDTH = 14,
    parameter INITIAL_PC = 0,
    parameter DEFAULT_TRAP_VECTOR = 0
)(
    input logic clock,
    input logic reset,

    // data memory port (read/write)
    output logic [DATA_ADDR_WIDTH-1:0] main_address,
    output logic [3:0]  main_byte_enable,
    output logic        main_read_enable,
    input  logic [31:0] main_read_data,
    input  logic        main_read_data_valid,
    output logic        main_write_enable,
    output logic [31:0] main_write_data,
    input  logic        main_wait,

    // instruction memory port (read-only)
    output logic [INSTR_ADDR_WIDTH-1:0] instr_address,
    output logic        instr_read_enable,
    input  logic [31:0] instr_read_data,
    input  logic        instr_wait,

    // external interrupt signal, active high
    input  logic        inr_irq,

    // debug ports
    output logic [31:0] debug_register28,
    output logic [31:0] debug_scratch,
    output logic [31:0] debug_pc
);

    localparam TRACE = 1;

    logic [31:0] registers [1:31]; // register file - register zero is hardcoded to 0 when fetching

    logic [63:0] instret = '0;  // number of instructions retired (completed)
    logic [63:0] cycles  = '0;  // cycle counter
    always_ff @(posedge clock) cycles <= reset ? 0 : cycles + 1;

    // Some CSRs (Control and Status Registers)
    mstatus_t mstatus = '0;  // status
    mie_t mie = '0;          // interrupts enabled
    mip_t mip;               // interrupts pending
    logic [31:0] dscratch;   // debug scratch register, used for debug output
    logic [31:0] mtvec = DEFAULT_TRAP_VECTOR;  // trap handler address

    // traps caused by the instruction being fetched or executed
    logic interrupt, if_exception, ex_exception, ex_mem_address_error;
    
    logic main_read_pending = 0; //whether we have sent a memory read which has not yet been replied to
    
    // buffer to hold the last valid main memory read response: valid is set iff this data has not yet been used by MA
    logic[31:0] main_read_data_buffer;
    logic main_read_data_buffer_valid = 0;

    // Stage invalidation flags
    logic if_invalid = 1;
    logic if_de_invalid = 1;
    logic de_ex_invalid = 1;
    logic ex_ma_invalid = 1;
    logic ma_wb_invalid = 1;

    logic stall_for_memory_wait;   // stall everything when main memory or instruction memory isn't ready for load/store/IF
    logic stall_for_load_dep; // stall IF, DE and repeat EX for a load followed by dependent instruction
    logic stall_for_memory_pending; //stall IF, DE and EX when a read request is late being answered
    
    // distribute stall signals to each stage:
    logic stall_if;
    logic stall_de;
    logic stall_ex;
    logic stall_ma;
    logic stall_wb;
    
    always_comb begin
        stall_if = stall_for_memory_wait || stall_for_memory_pending || stall_for_load_dep;
        stall_de = stall_for_memory_wait || stall_for_memory_pending || stall_for_load_dep;
        stall_ex = stall_for_memory_wait || stall_for_memory_pending;
        stall_ma = stall_for_memory_wait;
        stall_wb = stall_for_memory_wait;
    end

    // === Instruction Fetch ===================================================

    logic [31:0] pc = INITIAL_PC;
    logic [31:0] if_pc, if_de_pc, if_de_instr, instr_read_data_buffer;
    logic if_stall_on_prev;

    always_comb begin
        // PC is byte-addressed but our instruction memory is word-addressed
        instr_address = pc[INSTR_ADDR_WIDTH+1:2];
        // read the next instruction on every cycle
        instr_read_enable = '1;
    end

    always_ff @(posedge clock) begin
        // buffer the last instruction read before a stall.
        if_stall_on_prev <= stall_if;
        if (!if_stall_on_prev)
            instr_read_data_buffer <= instr_read_data;

        if (!stall_if) begin
            // if there was a stall on the last cycle, we read from the instruction buffer not the bus.
            // this allows the PC to 'catch up' on the next cycle.
            if_de_instr <= if_stall_on_prev ? instr_read_data_buffer : instr_read_data;
            if_pc <= pc;
            if_de_pc <= if_pc;
        end
    end

    // === Decode ==============================================================

    logic [31:0] de_rs1_fetched, de_rs2_fetched;
    logic [31:0] de_rs1_forward, de_rs2_forward; // forwarding logic appears later
    instr_t      de_instr, de_ex_instr;
    logic [31:0] de_ex_rs1_value, de_ex_rs2_value;

    always_comb begin
        de_instr = decode_instr(if_de_instr, if_de_pc);

        // register fetch
        de_rs1_fetched = fetch(de_instr.rs1);
        de_rs2_fetched = fetch(de_instr.rs2);

        // if the next instruction is a load and this instruction is dependent on its result,
        // stall for one cycle since the result won't be ready yet - unless the load raises an exception.
        stall_for_load_dep = !if_de_invalid && !de_ex_invalid && de_ex_instr.memory_read && !ex_mem_address_error
                          && (de_instr.rs1 == de_ex_instr.rd && de_instr.rs1_used
                           || de_instr.rs2 == de_ex_instr.rd && de_instr.rs2_used);

        // ignore waitrequest unless we are actually reading/writing memory,
        // because the bus is allowed to hold waitrequest high while idle.
        stall_for_memory_wait = (instr_wait && instr_read_enable)
                                || (main_wait && (main_read_enable || main_write_enable));
                                
        stall_for_memory_pending = main_read_pending && !main_read_data_buffer_valid && !main_read_data_valid;

    end

    always_ff @(posedge clock) begin
        if (!stall_de) begin
            de_ex_instr <= de_instr;
            // if the value isn't forwarded, these take the register fetch results.
            de_ex_rs1_value <= de_rs1_forward;
            de_ex_rs2_value <= de_rs2_forward;
        end
    end

    // === Execute =============================================================

    instr_t      ex_ma_instr;
    logic [31:0] ex_mem_address, ex_result, ex_ma_result;
    logic [1:0]  ex_word_offset, ex_ma_word_offset;
    logic [29-DATA_ADDR_WIDTH:0] ex_address_high_bits;  // beyond our address width so should be 0

    always_comb begin
        ex_result = execute(de_ex_instr, de_ex_rs1_value, de_ex_rs2_value);

        // --- Memory Access ---------------------------------------------------

        main_read_enable  = !de_ex_invalid && !interrupt && !ex_mem_address_error && de_ex_instr.memory_read && !stall_for_memory_pending;
        main_write_enable = !de_ex_invalid && !interrupt && !ex_mem_address_error && de_ex_instr.memory_write && !stall_for_memory_pending;

        // do address calculation
        ex_mem_address = de_ex_rs1_value + de_ex_instr.immediate;
        // our memory is word addressed, so cut off the bottom two bits (this becomes the word offset),
        // and the higher bits beyond our address range which should be 0.
        {ex_address_high_bits, main_address, ex_word_offset} = ex_mem_address;

        // set byte_enable mask according to whether we are loading/storing a word, half word or byte.
        main_byte_enable = compute_byte_enable(de_ex_instr.memory_width, ex_word_offset);

        // shift the store value into the correct position in the 32-bit word
        main_write_data = de_ex_rs2_value << ex_word_offset*8;
    end

    always_ff @(posedge clock)
        if (reset) begin
            main_read_pending <= 0;
        end else begin
            if (!stall_ex) begin
                ex_ma_instr       <= de_ex_instr;
                ex_ma_result      <= ex_result;
                ex_ma_word_offset <= ex_word_offset;
                main_read_pending <= main_read_enable;
            end
        end

    // === Branching or Reset ==================================================

    logic ex_branch_taken;
    logic [31:0] ex_branch_target, ex_next_pc;

    always_comb begin
        ex_branch_taken = !de_ex_invalid && is_branch_taken(de_ex_instr.op, de_ex_rs1_value, de_ex_rs2_value);
        ex_branch_target = target_pc(de_ex_instr, de_ex_rs1_value);
        ex_next_pc = ex_branch_taken ? ex_branch_target : pc + 4; //note that pc + 4 is actually a prediction for 3 instructions' time
    end

    always_ff @(posedge clock)
        if (reset) begin
            pc <= INITIAL_PC;
            if_invalid <= 1;
            if_de_invalid <= 1;
            de_ex_invalid <= 1;
            ex_ma_invalid <= 1;
            ma_wb_invalid <= 1;
        end else begin
            // logic for stage invalidation upon taking a branch or stalling
            // don't change the registers if the corresponding stage is stalled
            
            if (!stall_if) begin
                // if a trap is taken, go to the handler instead
                pc <= (if_exception || ex_exception) ? mtvec : ex_next_pc;
            
                // invalidate on any exception, interrupt or branch.
                if_invalid <= interrupt || ex_exception || if_exception || ex_branch_taken;
                    
                // invalidate on an EX exception, interrupt or branch.
                // an IF exception can only happen after a branch so this stage would already be invalid.
                if_de_invalid <= if_invalid || interrupt || ex_exception || ex_branch_taken;
            end
         
            // invalidate in an EX exception, interrupt, branch or load dependency stall.
            // an IF exception can only happen after a branch so this stage would already be invalid.
            if (!stall_de)  de_ex_invalid <= if_de_invalid || interrupt || ex_exception || ex_branch_taken;
            else if (!stall_ex) de_ex_invalid <= 1; // we only stall de but not ex on load dep, so insert a bubble
            
            // invalidate on an interrupt or any EX exception that could be caused by an instruction that writes back.
            // i.e. an exception on a load or an invalid instruction.
            if (!stall_ex)  ex_ma_invalid <= de_ex_invalid || interrupt || ex_mem_address_error && de_ex_instr.memory_read || de_ex_instr.op == INVALID;
            // we only stall ex and not ma when memory pending, so replay (no bubble here)
            
            // if ma received invalid data, insert a bubble into wb
            if (!stall_ma)  ma_wb_invalid <= ex_ma_invalid || stall_for_memory_pending;
        end

    // === Memory Align ========================================================

    instr_t ma_wb_instr;
    logic[31:0] ma_result, ma_load_value, ma_wb_value;

    always_comb begin
        // align the loaded value: if we stalled on last cycle then take buffered data instead
        ma_load_value = load_shift_mask_extend(ex_ma_instr.memory_width,
                                               ex_ma_instr.memory_read_unsigned,
                                               main_read_data_valid ? main_read_data : main_read_data_buffer,
                                               ex_ma_word_offset);
        // if this isn't a load instruction, pass through the ALU result instead
        ma_result = ex_ma_instr.memory_read ? ma_load_value : ex_ma_result;
    end

    always_ff @(posedge clock)
        if (reset) begin
            main_read_data_buffer_valid <= 0;
        end else begin
            if (!stall_ma) begin
                  ma_wb_instr <= ex_ma_instr;
                  ma_wb_value <= ma_result;
                  main_read_data_buffer_valid <= 0;
            end else begin
                  main_read_data_buffer_valid <= main_read_data_buffer_valid || main_read_data_valid;
            end
            //buffer the last data returned in case of stall
            if (main_read_data_valid) begin
                  main_read_data_buffer <= main_read_data;
            end
        end

    // === Write Back ==========================================================

    always_ff @(posedge clock) begin
        if (!stall_wb && !ma_wb_invalid) begin
            if (ma_wb_instr.enable_wb) begin
                registers[ma_wb_instr.rd] <= ma_wb_value;
            end
            instret <= instret + 1;
        end
    end

    // === Forwarding ==========================================================

    logic could_forward_from_ex, could_forward_from_ma, could_forward_from_wb;
    register_t de_rs1, de_rs2;

    always_comb begin
        // check if stages are eligible to have their values forwarded
        // forward from EX result: instruction must not be a load since result won't be ready until end of MA stage
        could_forward_from_ex = !de_ex_invalid && de_ex_instr.enable_wb && !de_ex_instr.memory_read;
        could_forward_from_ma = !ex_ma_invalid && ex_ma_instr.enable_wb;
        could_forward_from_wb = !ma_wb_invalid && ma_wb_instr.enable_wb;

        // now we also check whether source and destination registers match up
        // prioritise forwarding from earlier stages (more recent instructions),
        // since these may overwrite values written by later stages (less recent instructions).
        if      (could_forward_from_ex && de_ex_instr.rd == de_instr.rs1) de_rs1_forward = ex_result;
        else if (could_forward_from_ma && ex_ma_instr.rd == de_instr.rs1) de_rs1_forward = ma_result;
        else if (could_forward_from_wb && ma_wb_instr.rd == de_instr.rs1) de_rs1_forward = ma_wb_value;
        else                                                              de_rs1_forward = de_rs1_fetched;

        if      (could_forward_from_ex && de_ex_instr.rd == de_instr.rs2) de_rs2_forward = ex_result;
        else if (could_forward_from_ma && ex_ma_instr.rd == de_instr.rs2) de_rs2_forward = ma_result;
        else if (could_forward_from_wb && ma_wb_instr.rd == de_instr.rs2) de_rs2_forward = ma_wb_value;
        else                                                              de_rs2_forward = de_rs2_fetched;
    end


`ifdef MACHINE_MODE

    // === Interrupts and Exceptions ===========================================

    logic [31:0] mcause;        // trap cause
    logic [31:0] mepc;          // return address after handling trap
    logic [31:0] mbadaddr;      // address of instruction which caused an access/misaligned fault
    logic [31:0] mscratch;      // machine mode scratch register
    logic [63:0] timecmp = '0;  // time compare register for triggering timer interrupt

    logic [31:0] trap_pc;       // the address of the instruction that caused the trap or suffered the interrupt
    logic [31:0] potential_mepc;

    always_comb begin
        // wire external interrupt signal to the mip.meip register bit
        mip.meip = inr_irq;
        // raise a timer interrupt when time (cycle count) is less than timecmp
        mip.mtip = cycles >= timecmp;
        // interrupt is only raised if appropriate interrupt enable bits are set
        interrupt = mstatus.mie && (mip.meip && mie.meie || mip.msip && mie.msie || mip.mtip && mie.mtie);
        // instruction fetch fault or misaligned exception
        if_exception = pc[31:INSTR_ADDR_WIDTH+2] != '0 || !is_aligned(pc[1:0], W);
        // load/store fault or misaligned exception
        ex_mem_address_error = ex_address_high_bits != '0 || !is_aligned(ex_word_offset, de_ex_instr.memory_width);
        // any exception or trap raised by the currently executing instruction
        ex_exception = !de_ex_invalid && (ex_mem_address_error && (de_ex_instr.memory_read || de_ex_instr.memory_write)
                    || de_ex_instr.op == INVALID || de_ex_instr.op == ECALL || de_ex_instr.op == EBREAK);

        if (if_exception)
            trap_pc = pc;
        else if (de_ex_invalid)
            // if we get an interrupt while the execute stage is invalid, return to the next valid instruction instead.
            trap_pc = potential_mepc;
        else
            trap_pc = de_ex_instr.pc;
    end

    always_ff @(posedge clock) begin
        // In case an interrupt happens while EX is invalid, we must remember what PC to return to after the handler.
        // EX could be invalidated by (a) an interrupt/exception, (b) a branch, or (c) a load dependency.
        // We needn't worry about (a) because interrupts will first be disabled in this case.
        // So whenever EX is valid, we just remember the branch target if a branch is happening, for case (b),
        // or otherwise the PC of the intruction about to be decoded, which would be next up in case (c).
        if (!de_ex_invalid) begin
            potential_mepc <= ex_branch_taken ? ex_branch_target : if_de_pc;
        end

        if (reset) begin
            // reset the CSR state for interrupts/exceptions
            mtvec <= DEFAULT_TRAP_VECTOR;
            mstatus <= '0;
            mie <= '0;
        end
        
        if ((!stall_if && if_exception) || (!stall_ex && ex_exception) || interrupt) begin
            // Entering a trap handler. Push 0 onto the mstatus interrupts-enabled stack
            mstatus.mpie <= mstatus.mie;
            mstatus.mie <= '0;
            // record the address of the instruction that caused the trap or the instruction that got interrupted
            mbadaddr <= trap_pc;
            mepc <= trap_pc;
            // set the trap cause
            mcause <= get_trap_cause();
         end

         if (!stall_ex && !de_ex_invalid && de_ex_instr.op == MRET) begin
            // Returning from trap handler. Pop the mstatus interrupts-enabled stack.
            mstatus.mie <= mstatus.mpie;
            mstatus.mpie <= '1;
         end
         
         // Do CSR write/set/clear operations if we are executing a CSR instruction
         if (!stall_ex && !de_ex_invalid && !interrupt)
            // CSR operations can't cause a trap because they decode into INVALID instead
            execute_csr(de_ex_instr, de_ex_rs1_value);
    end

`else
    // if machine mode is disabled, there are never any interrupts or exceptions.
    always_comb begin
        interrupt = '0;
        if_exception = '0;
        ex_mem_address_error = '0;
        ex_exception = '0;
    end
`endif


    // === Decode functions ====================================================

    function automatic logic [31:0] fetch(register_t register);
        // register zero is wired to constant 0.
        return register == zero ? '0 : registers[register];
    endfunction


    function automatic instr_t decode_instr(logic [31:0] instr, logic [31:0] pc);
        // registers, funct7 and funct3 are in the same place in every instruction type
        decode_instr.rd  = register_t'(instr`rd);
        decode_instr.rs1 = register_t'(instr`rs1);
        decode_instr.rs2 = register_t'(instr`rs2);
        decode_instr.funct12 = funct12_t'(instr`funct12);

        decode_instr.op = decode_opcode(instr);

        // we check whether a register is used for forwarding purposes -- no need to forward the zero register
        decode_instr.rs1_used = decode_instr.rs1 != zero
                             && instr`opcode != OPC_LUI
                             && instr`opcode != OPC_AUIPC
                             && instr`opcode != OPC_JAL
                             && instr`opcode != OPC_MISC_MEM;

        decode_instr.rs2_used = decode_instr.rs2 != zero
                             && (instr`opcode == OPC_BRANCH
                              || instr`opcode == OPC_STORE
                              || instr`opcode == OPC_OP);

        {decode_instr.immediate_used, decode_instr.immediate} = decode_immediate(instr);

        decode_instr.memory_write = instr`opcode == OPC_STORE;
        decode_instr.memory_read  = instr`opcode == OPC_LOAD;
        decode_instr.memory_read_unsigned = instr[14];  // if memory_read is true, this indicates an unsigned read.
        decode_instr.memory_width = mem_width_t'(instr[13:12]);

        // write back for all except s-type instructions and not to register 0
        decode_instr.enable_wb = decode_instr.rd != '0
                              && instr`opcode != OPC_BRANCH
                              && instr`opcode != OPC_STORE;

        decode_instr.pc = pc;
    endfunction


    function automatic operation_t decode_opcode(logic [31:0] instr);
        logic [11:0] funct12 = instr`funct12;
        logic [2:0]  funct3  = instr`funct3;
        logic [4:0]  rs1 = instr`rs1;
        logic legal_csr_op = validate_csr_op(rs1 != zero, csr_t'(funct12));

        // ensure the two LSBs are 1
        if (instr[1:0] != 2'b11)
            return INVALID;

        unique case (instr`opcode)
            OPC_LUI:   return LUI;
            OPC_AUIPC: return AUIPC;
            OPC_JAL:   return JAL;
            OPC_JALR:  return JALR;
            OPC_BRANCH:
                unique case (funct3)
                    F3_BEQ:  return BEQ;
                    F3_BNE:  return BNE;
                    F3_BLT:  return BLT;
                    F3_BLTU: return BLTU;
                    F3_BGE:  return BGE;
                    F3_BGEU: return BGEU;
                endcase
            OPC_LOAD:  return LOAD;
            OPC_STORE: return STORE;
            OPC_OP, OPC_OP_IMM:
                unique case (funct3)
                    // there is no SUBI instruction so also check opcode
                    F3_ADDSUB: return instr[5] && funct12[10] ? SUB : ADD;
                    F3_SLT:  return SLT;
                    F3_SLTU: return SLTU;
                    F3_XOR:  return XOR;
                    F3_OR:   return OR;
                    F3_AND:  return AND;
                    // for immediate shifts we check the 6th bit in the shift amount is 0
                    // for non-immediate shifts this value must be 0 anyway (it is the end of the funct7 code)
                    F3_SLL:  return funct12[5] ? INVALID : SL;
                    F3_SR:   return funct12[5] ? INVALID : (funct12[10] ? SRA : SRL);
                    default: return INVALID;
                endcase
            OPC_MISC_MEM: return funct3[0] ? FENCE_I : FENCE;
            OPC_SYSTEM:
                unique case (funct3[1:0])
                    // when rs1 is zero we are not writing to the CSR. This is used when checking for
                    // an illegal write to a read-only CSR.
                    F2_CSRRW: return legal_csr_op ? CSRRW : INVALID;
                    F2_CSRRS: return legal_csr_op ? CSRRS : INVALID;
                    F2_CSRRC: return legal_csr_op ? CSRRC : INVALID;
                    F2_PRIV:
                        unique case (funct12)
                            F12_ECALL:  return ECALL;
                            F12_EBREAK: return EBREAK;
`ifdef MACHINE_MODE
                            F12_MRET:   return MRET;
                            F12_WFI:    return WFI;
`endif
                            default:    return INVALID;
                        endcase
                endcase
            default: return INVALID;
        endcase
    endfunction

    function automatic logic [32:0] decode_immediate(logic [31:0] instr);
        // returns an extra top bit to indicate whether the immediate is used
        // all except u-type instructions have sign-extended immediates.
        logic [19:0] sign_ext_20 = {20{instr[31]}};
        logic [11:0] sign_ext_12 = {12{instr[31]}};
        unique case (instr`opcode)
            OPC_JALR, OPC_LOAD, OPC_OP_IMM: // i-type
                return {1'b1, sign_ext_20, instr[31:20]};
            OPC_STORE: // s-type
                return {1'b1, sign_ext_20, instr[31:25], instr[11:7]};
            OPC_BRANCH: // sb-type
                return {1'b1, sign_ext_20, instr[7], instr[30:25], instr[11:8], 1'b0};
            OPC_JAL: // uj-type
                return {1'b1, sign_ext_12, instr[19:12], instr[20], instr[30:21], 1'b0};
            OPC_LUI, OPC_AUIPC: // u-type
                return {1'b1, instr[31:12], 12'b0};
            OPC_SYSTEM: // no ordinary immediate but possibly a csr zimm (5-bit immediate)
                return {instr[14], 32'bx};
            default: // no immediate
                return {1'b0, 32'bx};
        endcase
    endfunction

    function automatic logic validate_csr_op(logic write, csr_t csr);
        // first check we aren't writing to a read-only CSR
        if (write && csr[11:10] == 2'b11)
            return '0;
        unique case (csr)
`ifdef MACHINE_MODE
            // all the CSRs we support in machine mode...
            MVENDORID, MARCHID, MIMPID, MHARTID, MEDELEG, MIDELEG,
            MISA, MTVEC, MSTATUS, MIP, MIE, MSCRATCH, MEPC, MCAUSE, MBADADDR, DSCRATCH,
            MCYCLE, MTIME, MINSTRET, MCYCLEH, MTIMEH, MINSTRETH, MTIMECMP, MTIMECMPH,
`endif
            // in user-mode only timer CSRs can be read
            CYCLE, TIME, CYCLEH, TIMEH:
                     return '1;
            default: return '0;
        endcase

    endfunction


    // === Execute functions ===================================================

    function automatic logic [31:0] execute(instr_t instr, logic [31:0] rs1_value, logic [31:0] rs2_value);

        logic [31:0] rs2_value_or_imm = instr.immediate_used ? instr.immediate : rs2_value;

        // implement both logical and arithmetic as an arithmetic right shift, with a 33rd bit set to 0 or 1 as required.
        // logic signed [32:0] rshift_operand = {(instr.funct7_bit & rs1_value[31]), rs1_value};

        // shifts use the lower 5 bits of the intermediate or rs2 value
        logic [4:0] shift_amount = rs2_value_or_imm[4:0];

        unique case (instr.op)
            ADD:   return rs1_value + rs2_value_or_imm;
            SUB:   return rs1_value - rs2_value;
            SLT:   return $signed(rs1_value) < $signed(rs2_value_or_imm);
            SLTU:  return rs1_value < rs2_value_or_imm;
            XOR:   return rs1_value ^ rs2_value_or_imm;
            OR:    return rs1_value | rs2_value_or_imm;
            AND:   return rs1_value & rs2_value_or_imm;
            SL:    return rs1_value << shift_amount;
            SRL:   return rs1_value >> shift_amount;
            SRA:   return $signed(rs1_value) >>> shift_amount;
            LUI:   return instr.immediate;
            AUIPC: return instr.immediate + instr.pc;
            // JAL(R) stores the address of the instruction that followed the jump
            JAL, JALR: return instr.pc + 4;
            CSRRW, CSRRS, CSRRC: return read_csr(csr_t'(instr.funct12));
            default: return 'x;
        endcase
    endfunction


    function automatic logic is_branch_taken(operation_t operation, logic [31:0] rs1_value, logic [31:0] rs2_value);
        unique case (operation)
            BEQ:  return rs1_value == rs2_value;
            BNE:  return rs1_value != rs2_value;
            BGEU: return rs1_value >= rs2_value;
            BLTU: return rs1_value <  rs2_value;
            BGE:  return $signed(rs1_value) >= $signed(rs2_value);
            BLT:  return $signed(rs1_value) <  $signed(rs2_value);
            // we implement fence.i (sync instruction and data memory) by doing a branch to reload the next instruction
            JAL, JALR, FENCE_I, MRET: return '1;
            default: return '0;
        endcase
    endfunction


    function automatic logic [31:0] target_pc(instr_t instr, logic [31:0] rs1_value);
        unique case (instr.op)
            JAL, BEQ, BNE, BLT, BGE, BLTU, BGEU: return instr.pc + instr.immediate;
            JALR:    return (rs1_value + instr.immediate) & 32'h_ff_ff_ff_fe; // set LSB to 0
            FENCE_I: return instr.pc + 4;
`ifdef MACHINE_MODE
            MRET:    return mepc;  // return from interrupt handler
`endif
            default: return 'x;
        endcase
    endfunction


    // === Memory Access functions =============================================

    function automatic logic [3:0] compute_byte_enable(mem_width_t width, logic [1:0] word_offset);
        unique case (width)
            B: return 4'b0001 << word_offset;
            H: return 4'b0011 << word_offset;
            W: return 4'b1111 << word_offset;
            default: return 'x;
        endcase
    endfunction

    function automatic logic [31:0] load_shift_mask_extend(mem_width_t width, logic is_unsigned, logic [31:0] value, logic [1:0] word_offset);
        logic [31:0] masked_value = load_mask(width, value, word_offset);
        unique case (width)
            B: return is_unsigned
                    ? {24'b0, masked_value[7:0]}
                    : {{24{masked_value[7]}}, masked_value[7:0]};
            H: return is_unsigned
                    ? {16'b0, masked_value[15:0]}
                    : {{16{masked_value[15]}}, masked_value[15:0]};
            W: return value;
            default: return 'x;
        endcase
    endfunction

    function automatic logic [31:0] load_mask(mem_width_t width, logic [31:0] value, logic [1:0] word_offset);
        unique case (width)
            B: return (value >> word_offset*8) & 32'h_00_00_00_ff;
            H: return (value >> word_offset*8) & 32'h_00_00_ff_ff;
            default: return 'x;
        endcase
    endfunction



    // === CSR functions =======================================================

    function automatic logic [31:0] read_csr(csr_t csr_addr);
        case (csr_addr)
`ifdef MACHINE_MODE
            MVENDORID, MARCHID, MIMPID, MHARTID, MEDELEG, MIDELEG: return '0;
            MISA:      return 32'b01000000_00000000_00000001_00000000;
            MTVEC:     return {mtvec[31:2], 2'b0}; // must be aligned on a 4-byte boundary
            MSTATUS:   return {19'b0, 2'b11, 3'b0, mstatus.mpie, 3'b0, mstatus.mie, 3'b0};
            MIP:       return {20'b0, mip.meip, 3'b0, mip.mtip, 3'b0, mip.msip, 3'b0};
            MIE:       return {20'b0, mie.meie, 3'b0, mie.mtie, 3'b0, mie.msie, 3'b0};
            MSCRATCH:  return mscratch;
            MEPC:      return {mepc[31:2], 2'b0}; // must be aligned on a 4-byte boundary
            MCAUSE:    return {mcause[31], 27'b0, mcause[3:0]};
            MBADADDR:  return mbadaddr;
            DSCRATCH:  return dscratch;
            MINSTRET:  return instret[31:0];
            MINSTRETH: return instret[63:32];
            MTIMECMP:  return timecmp[31:0];
            MTIMECMPH: return timecmp[63:32];
            // since we have a fixed frequency, we can say time = cycle count.
            MCYCLE,  MTIME:  return cycles[31:0];
            MCYCLEH, MTIMEH: return cycles[63:32];
`endif
            CYCLE,  TIME:  return cycles[31:0];
            CYCLEH, TIMEH: return cycles[63:32];
            default:   return 'x;
        endcase
    endfunction

`ifdef MACHINE_MODE
    `define write_csr(operation, value, csr) \
        case (operation)                     \
            CSRRW: csr <= value;             \
            CSRRS: csr <= csr | value;       \
            CSRRC: csr <= csr & ~value;      \
        endcase

    task automatic execute_csr(instr_t instr, logic [31:0] rs1_value);
        // for immediate versions of the CSR instructions, the rs1 field contains a 5-bit immediate.
        logic[31:0] value = instr.immediate_used ? instr.rs1 : rs1_value;
        logic[11:0] csr_addr = instr.funct12;
        case (csr_addr)
            MTVEC:     `write_csr(instr.op, value, mtvec)
            MSTATUS:   `write_csr(instr.op, value, mstatus)
            MIE:       `write_csr(instr.op, value, mie)
            MSCRATCH:  `write_csr(instr.op, value, mscratch)
            MEPC:      `write_csr(instr.op, value, mepc)
            MCAUSE:    `write_csr(instr.op, value, mcause)
            MBADADDR:  `write_csr(instr.op, value, mbadaddr)
            MTIMECMP:  `write_csr(instr.op, value, timecmp[31:0])
            MTIMECMPH: `write_csr(instr.op, value, timecmp[63:32])
            DSCRATCH:  `write_csr(instr.op, value, dscratch)
        endcase
    endtask


    // == Exception functions =======================================================

    function automatic mcause_t get_trap_cause();
        // we return a struct containing a bit indicating whether to trap, then the cause.
        if (mstatus.mie) begin
            if (mip.meip && mie.meie) return MEI;
            if (mip.msip && mie.msie) return MSI;
            if (mip.mtip && mie.mtie) return MTI;
        end

        if (pc[31:INSTR_ADDR_WIDTH+2] != '0)
            return INSTR_FAULT;
        if (!is_aligned(pc[1:0], W))
            return INSTR_MISALIGN;

        unique case (de_ex_instr.op)
            INVALID: return ILLEGAL_INSTR;
            LOAD:
                if (ex_address_high_bits != '0)
                    return LOAD_FAULT;
                else if (!is_aligned(ex_word_offset, de_ex_instr.memory_width))
                    return LOAD_MISALIGN;
            STORE:
                if (ex_address_high_bits != '0)
                    return STORE_FAULT;
                else if (!is_aligned(ex_word_offset, de_ex_instr.memory_width))
                    return STORE_MISALIGN;

            ECALL:  return ECALL_M;
            EBREAK: return BREAK;
            default: ;
        endcase

        return mcause_t'('x);
    endfunction


    function automatic logic is_aligned(logic [1:0] word_offset, mem_width_t width);
        unique case (width)
            W: return word_offset == '0;
            H: return word_offset[0] == '0;
            default: return '1;
        endcase
    endfunction
`endif

    // === Simluation and Debugging ============================================

    // debug output from the MA stage
    always_comb begin
        debug_register28 = registers[28];
        debug_scratch = dscratch;
        debug_pc = ex_ma_instr.pc;
    end

`ifdef SIMULATION
`include "clarvi_debug.sv"
`endif

endmodule
