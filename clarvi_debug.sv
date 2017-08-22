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


   always_ff @(posedge clock) begin
        // it is most convenient to catch instructions in the MA stage.
        if (!ex_ma_invalid && !stall_wb) begin
            case (ex_ma_instr.op)
                CSRRW, CSRRS, CSRRC:
                    // catch writes to dscratch and output them
                    if (ex_ma_instr.funct12 == DSCRATCH)
                        $display("Debug output: %s = 0x%h", ex_ma_instr.rs1, dscratch);
            endcase
        end

        // However, if we are going to stop the simulator, it is better to catch instructions
        // in the WB stage so that the preceeding instruction will have written back and retired.
        if (!ma_wb_invalid && !stall_wb) begin
            case (ma_wb_instr.op)
                ECALL, INVALID: begin
                    $display("time: %0d, %p instruction at %h, stopping simulator", $time, ma_wb_instr, ma_wb_instr.pc);
                    $stop();
                end
            endcase
        end
    end

`ifdef TRACE

    logic [31:0] db_rs1_value, db_rs2_value, db_result, db_mem_address, db_branch_target;
    logic db_invalid, db_branch_taken;
    instr_t db_instr;

    always_comb begin
        db_instr = ex_ma_instr;
        db_invalid = ex_ma_invalid || stall_ma;
        db_result = ma_result;
    end

    always_ff @(posedge clock) begin
        if (!stall_ma) begin
            db_rs1_value <= de_ex_rs1_value;
            db_rs2_value <= de_ex_rs2_value;
            db_mem_address <= ex_mem_address;
            db_branch_target <= ex_branch_target;
            db_branch_taken <= ex_branch_taken;
        end
        print_instruction();
    end

    task automatic print_instruction();
        string register_result = "";
        string register_values = "";
        csr_t csr = csr_t'(db_instr.funct12);

        // first output the PC
        $write("0x%h:   ", db_instr.pc);

        if (db_invalid) begin
            // the stage is invalid (e.g. because we took a branch)
            $display("---");
            return;
        end

        // write the destination register assignment to a string
        if (db_instr.enable_wb)
            register_result = $sformatf("%s := 0x%h", db_instr.rd, db_result);

        // write the source register values to a string
        if (db_instr.rs1_used && db_instr.rs2_used)
            register_values = $sformatf("%s = 0x%h, %s = 0x%h", db_instr.rs1, db_rs1_value, db_instr.rs2, db_rs2_value);
        else if (db_instr.rs1_used)
            register_values = $sformatf("%s = 0x%h", db_instr.rs1, db_rs1_value);
        else if (db_instr.rs2_used)
            register_values = $sformatf("%s = 0x%h", db_instr.rs2, db_rs2_value);

        // for each kind of instruction, output an appropriate trace, perhaps using the source/designation register strings from above.
        case (db_instr.op)
            INVALID: $write("%s", db_instr.op);
            JAL:  $write("%s\t%s, %0d\t\t%s, target = 0x%h", db_instr.op, db_instr.rd, $signed(db_instr.immediate), register_result, db_branch_target);
            JALR: $write("%s\t%s, %s, %0d\t\t%s, %s, target = 0x%h", db_instr.op, db_instr.rd, db_instr.rs1, $signed(db_instr.immediate), register_result, register_values, db_branch_target);
            ADD, SUB, SLT, SLTU, XOR, OR, AND, SL, SRL, SRA: begin
                if (db_instr.immediate_used)
                    $write("%s%s\t%s, %s, %0d\t\t%s, %s", db_instr.op, "I", db_instr.rd, db_instr.rs1, $signed(db_instr.immediate), register_result, register_values);
                else
                    $write("%s\t%s, %s, %s\t\t%s, %s", db_instr.op, db_instr.rd, db_instr.rs1, db_instr.rs2, register_result, register_values);
            end
            BEQ, BNE, BLT, BGE, BLTU, BGEU:
                $write("%s\t%s, %s, 0x%h\t%s, branch %s", db_instr.op, db_instr.rs1, db_instr.rs2, db_branch_target, register_values, db_branch_taken ? "taken" : "not taken");
            LOAD:  $write("L%s%s\t%s, %0d(%s)\t\t%s = mem[0x%h], %s",  db_instr.memory_width, db_instr.memory_read_unsigned ? "U" : "", db_instr.rd,  $signed(db_instr.immediate), db_instr.rs1, register_result, db_mem_address, register_values);
            STORE: $write("S%s\t%s, %0d(%s)\t\tmem[%h] := 0x%h, %s",  db_instr.memory_width, db_instr.rs2, $signed(db_instr.immediate), db_instr.rs1, db_mem_address, db_rs2_value, register_values);
            ECALL, EBREAK, MRET, WFI, FENCE, FENCE_I:
                $write("%s", db_instr.op);
            CSRRW, CSRRS, CSRRC:
                $write("%s\t%s, %s, %s", db_instr.op, db_instr.rd, csr, db_instr.immediate_used ? $sformatf("%0d", db_instr.rs1) : db_instr.rs1.name);
            default: $write("%s\t%s, 0x%h\t\t%s", db_instr.op, db_instr.rd, db_instr.immediate, register_result);
        endcase

        // new line
        $display();
    endtask
`endif // TRACE
