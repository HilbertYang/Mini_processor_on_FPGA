// =============================================================================
// tb_pipeline.v  ?  Self-checking testbench for the 5-stage ARM pipeline
//
// Strategy
// --------
// Since the BRAMs (I_M_32bit_512depth, D_M_64bit_256) are Xilinx simulation
// black-boxes we CANNOT pre-load them with $readmemh.  Instead we use the
// same programming ports exposed by the pipeline itself:
//   ? imem_prog_we / imem_prog_addr / imem_prog_wdata  ? write instructions
//   ? dmem_prog_en / dmem_prog_we / dmem_prog_addr     ? seed / read-back data memory
//
// Test programs
// -------------
//  TEST 1 ? Arithmetic & Logic (ADD, SUB, AND, ORR)
//      MOV R1, #10        ; R1 = 10
//      MOV R2, #3         ; R2 = 3
//      NOP x5             ; data-hazard guard (no forwarding)
//      ADD R3, R1, R2     ; R3 = 13   expected 0x0000_000D
//      NOP x5
//      SUB R4, R1, R2     ; R4 = 7    expected 0x0000_0007
//      NOP x5
//      AND R5, R1, R2     ; R5 = 2    expected 0x0000_0002
//      NOP x5
//      ORR R6, R1, R2     ; R6 = 11   expected 0x0000_000B
//      NOP x10            ; let WB commit
//
//  TEST 2 ? Load / Store
//      MOV  R7, #0        ; base address = 0  (byte addr 0 ? dmem word 0)
//      MOV  R8, #0xAB     ; value to store
//      NOP x5
//      STR  R8, [R7, #0]  ; dmem[0] = 0xAB
//      NOP x5
//      LDR  R9, [R7, #0]  ; R9 = dmem[0] = 0xAB  expected 0x0000_00AB
//      NOP x10
//
//  TEST 3 ? Branch (B)
//      A simple forward branch: B over a poisoned ADD, land on a safe MOV.
//      MOV  R10, #0       ; R10 = 0
//      B    +2            ; skip next 2 instructions (offset = +2 in pipeline)
//      ADD  R10, R10, #1  ; should be skipped ? R10 stays 0  if branch works
//      ADD  R10, R10, #1  ; should be skipped
//      MOV  R10, #0xFF    ; R10 = 255  ? branch lands here
//      NOP x10
//
// Instruction encoding (ARM32, cond=1110 always):
// -----------------------------------------------
//  MOV Rd, #imm8:  1110 001 1101 0 0000 Rd 0000 imm8
//    op=00, I=1, opcode=1101(MOV), S=0, Rn=0000, Rd, rot=0000, imm8
//
//  ADD Rd, Rn, Rm: 1110 000 0100 0 Rn Rd 00000000 Rm
//    op=00, I=0, opcode=0100(ADD), S=0, Rn, Rd, shift=0, Rm
//
//  ADD Rd, Rn, #imm8: 1110 001 0100 0 Rn Rd 0000 imm8
//
//  SUB Rd, Rn, Rm: 1110 000 0010 0 Rn Rd 00000000 Rm
//
//  AND Rd, Rn, Rm: 1110 000 0000 0 Rn Rd 00000000 Rm
//
//  ORR Rd, Rn, Rm: 1110 000 1100 0 Rn Rd 00000000 Rm
//
//  STR Rd, [Rn, #off]: 1110 010 1 1000 Rn Rd offset12
//    op=01, I=0(imm offset), P=1, U=1, B=0, W=0, L=0
//    bits: 1110_0101_1000_Rn_Rd_offset12
//    [27:20] = 0101_1000  for P=1,U=1,B=0,W=0,L=0
//
//  LDR Rd, [Rn, #off]: 1110 010 1 1001 Rn Rd offset12
//    [27:20] = 0101_1001  for P=1,U=1,B=0,W=0,L=1
//
//  B  offset:  1110 1010 signed_offset24
//    [31:24]=1110_1010, [23:0]=offset
//    offset in ARM is (target - PC - 2) for our pipeline
//
//  NOP:  AND R0, R0, R0  = 32'hE000_0000
//
// =============================================================================

`timescale 1ns/1ps

module tb_pipeline;

    // =========================================================================
    // Parameters
    // =========================================================================
    parameter CLK_PERIOD      = 10;   // 10 ns ? 100 MHz
    parameter IMEM_ADDR_WIDTH = 9;
    parameter DATA_WIDTH      = 32;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg         clk;
    reg         reset;
    reg         run;
    reg         step;
    reg         pc_reset_pulse;

    reg         imem_prog_we;
    reg  [8:0]  imem_prog_addr;
    reg  [31:0] imem_prog_wdata;

    reg         dmem_prog_en;
    reg         dmem_prog_we;
    reg  [7:0]  dmem_prog_addr;
    reg  [63:0] dmem_prog_wdata;
    wire [63:0] dmem_prog_rdata;

    wire [8:0]  pc_dbg;
    wire [31:0] if_instr_dbg;

    // =========================================================================
    // Test tracking
    // =========================================================================
    integer pass_count;
    integer fail_count;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    pipeline #(
        .DATA_WIDTH      (DATA_WIDTH),
        .REG_ADDR_WIDTH  (4),
        .IMEM_ADDR_WIDTH (IMEM_ADDR_WIDTH)
    ) dut (
        .clk             (clk),
        .reset           (reset),
        .run             (run),
        .step            (step),
        .pc_reset_pulse  (pc_reset_pulse),
        .imem_prog_we    (imem_prog_we),
        .imem_prog_addr  (imem_prog_addr),
        .imem_prog_wdata (imem_prog_wdata),
        .dmem_prog_en    (dmem_prog_en),
        .dmem_prog_we    (dmem_prog_we),
        .dmem_prog_addr  (dmem_prog_addr),
        .dmem_prog_wdata (dmem_prog_wdata),
        .dmem_prog_rdata (dmem_prog_rdata),
        .pc_dbg          (pc_dbg),
        .if_instr_dbg    (if_instr_dbg)
    );

    // =========================================================================
    // Convenience handle to the internal register file
    // (hierarchical reference ? works in simulation)
    // =========================================================================
    // Path: dut.u_id.regfile.regFile[n]
    `define REGFILE dut.u_id.regfile.regFile

    // =========================================================================
    // Clock generation
    // =========================================================================
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // Instruction encoding helper macros
    // =========================================================================
    // NOP: AND R0,R0,R0  (all-zero instruction body, cond=1110)
    `define NOP          32'hE000_0000

    // MOV Rd, #imm8 :  cond=1110, op=001, opcode=1101, S=0, Rn=0000
    //   [31:28]=1110 [27:26]=00 [25]=1 [24:21]=1101 [20]=0 [19:16]=0000
    //   [15:12]=Rd   [11:8]=0000  [7:0]=imm8
    `define MOV_IMM(Rd,imm8) \
        {4'hE, 3'b001, 4'b1101, 1'b0, 4'b0000, (Rd), 4'b0000, (imm8)}

    // ADD Rd, Rn, Rm :  cond=1110, op=000, opcode=0100, S=0
    //   [31:28]=1110 [27:26]=00 [25]=0 [24:21]=0100 [20]=0
    //   [19:16]=Rn [15:12]=Rd [11:4]=0 [3:0]=Rm
    `define ADD_REG(Rd,Rn,Rm) \
        {4'hE, 3'b000, 4'b0100, 1'b0, (Rn), (Rd), 8'h00, (Rm)}

    // ADD Rd, Rn, #imm8
    `define ADD_IMM(Rd,Rn,imm8) \
        {4'hE, 3'b001, 4'b0100, 1'b0, (Rn), (Rd), 4'h0, (imm8)}

    // SUB Rd, Rn, Rm
    `define SUB_REG(Rd,Rn,Rm) \
        {4'hE, 3'b000, 4'b0010, 1'b0, (Rn), (Rd), 8'h00, (Rm)}

    // AND Rd, Rn, Rm
    `define AND_REG(Rd,Rn,Rm) \
        {4'hE, 3'b000, 4'b0000, 1'b0, (Rn), (Rd), 8'h00, (Rm)}

    // ORR Rd, Rn, Rm
    `define ORR_REG(Rd,Rn,Rm) \
        {4'hE, 3'b000, 4'b1100, 1'b0, (Rn), (Rd), 8'h00, (Rm)}

    // STR Rd,[Rn,#off] : op=01, P=1,U=1,B=0,W=0,L=0  ? [27:20]=0101_1000
    `define STR_IMM(Rd,Rn,off12) \
        {4'hE, 8'b0101_1000, (Rn), (Rd), (off12)}

    // LDR Rd,[Rn,#off] : op=01, P=1,U=1,B=0,W=0,L=1  ? [27:20]=0101_1001
    `define LDR_IMM(Rd,Rn,off12) \
        {4'hE, 8'b0101_1001, (Rn), (Rd), (off12)}

    // B offset24 : cond=1110 [27:24]=1010 [23:0]=offset24
    // In our pipeline the instruction in EX is 2 fetches ahead of PC,
    // so effective branch offset = desired_word_delta (already accounted for in ID).
    // We encode the raw offset24 we want the branch_target calculation to use.
    `define B_OFFSET(off24) \
        {4'hE, 4'b1010, (off24)}

    // =========================================================================
    // Task: write one instruction into instruction memory
    // =========================================================================
    task write_imem;
        input [8:0]  addr;
        input [31:0] instr;
        begin
            @(negedge clk);
            imem_prog_we    = 1'b1;
            imem_prog_addr  = addr;
            imem_prog_wdata = instr;
            @(negedge clk);
            imem_prog_we    = 1'b0;
        end
    endtask

    // =========================================================================
    // Task: run pipeline for N cycles
    // =========================================================================
    task run_cycles;
        input integer n;
        integer i;
        begin
            run = 1'b1;
            repeat(n) @(posedge clk);
            #1;          // settle combinational outputs
            run = 1'b0;
        end
    endtask

    // =========================================================================
    // Task: check register value
    // =========================================================================
    task check_reg;
        input [3:0]  reg_num;
        input [31:0] expected;
        input [127:0] test_name;  // packed string label
        reg   [31:0]  actual;
        begin
            actual = `REGFILE[reg_num];
            if (actual === expected) begin
                $display("  PASS  %-20s  R%0d = 0x%08X", test_name, reg_num, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL  %-20s  R%0d  got=0x%08X  expected=0x%08X",
                         test_name, reg_num, actual, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // =========================================================================
    // Task: read data memory via debug port and check
    // =========================================================================
    task check_dmem;
        input [7:0]  addr;
        input [31:0] expected;
        input [127:0] test_name;
        reg   [31:0]  actual;
        begin
            // Enable port B read
            @(negedge clk);
            dmem_prog_en   = 1'b1;
            dmem_prog_we   = 1'b0;
            dmem_prog_addr = addr;
            @(posedge clk); #1;           // one cycle for BRAM synchronous read
            actual = dmem_prog_rdata[31:0];
            dmem_prog_en = 1'b0;
            if (actual === expected) begin
                $display("  PASS  %-20s  dmem[%0d] = 0x%08X", test_name, addr, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL  %-20s  dmem[%0d]  got=0x%08X  expected=0x%08X",
                         test_name, addr, actual, expected);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // =========================================================================
    // Task: full reset sequence
    // =========================================================================
    task do_reset;
        begin
            reset           = 1'b1;
            run             = 1'b0;
            step            = 1'b0;
            pc_reset_pulse  = 1'b0;
            imem_prog_we    = 1'b0;
            imem_prog_addr  = 9'b0;
            imem_prog_wdata = 32'b0;
            dmem_prog_en    = 1'b0;
            dmem_prog_we    = 1'b0;
            dmem_prog_addr  = 8'b0;
            dmem_prog_wdata = 64'b0;
            repeat(4) @(posedge clk);
            @(negedge clk);
            reset = 1'b0;
        end
    endtask

    // =========================================================================
    // Main test body
    // =========================================================================
    integer addr;

    initial begin
        pass_count = 0;
        fail_count = 0;

        $dumpfile("tb_pipeline.vcd");
        $dumpvars(0, tb_pipeline);

        $display("======================================================");
        $display("  ARM Pipeline Testbench");
        $display("======================================================");

        do_reset;

        // =====================================================================
        // ?? TEST 1: Arithmetic & Logic ????????????????????????????????????????
        // =====================================================================
        $display("\n--- TEST 1: Arithmetic & Logic ---");

        // --- Load program into instruction memory ---
        // We write while reset=0, run=0 (pipeline halted).
        // PC is held because run=0.
        //
        // Word  Instruction
        //  0    MOV R1, #10
        //  1    MOV R2, #3
        //  2-6  NOP x5
        //  7    ADD R3, R1, R2
        //  8-12 NOP x5
        //  13   SUB R4, R1, R2
        //  14-18 NOP x5
        //  19   AND R5, R1, R2
        //  20-24 NOP x5
        //  25   ORR R6, R1, R2
        //  26-35 NOP x10

        addr = 0;
        write_imem(addr, `MOV_IMM(4'd1, 8'd10));  addr=addr+1; // R1=10
        write_imem(addr, `MOV_IMM(4'd2, 8'd3));   addr=addr+1; // R2=3
        // 5 NOPs ? hazard guard between MOV and ADD (no forwarding)
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `ADD_REG(4'd3, 4'd1, 4'd2)); addr=addr+1; // R3=R1+R2=13
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `SUB_REG(4'd4, 4'd1, 4'd2)); addr=addr+1; // R4=R1-R2=7
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `AND_REG(4'd5, 4'd1, 4'd2)); addr=addr+1; // R5=R1&R2=2
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `ORR_REG(4'd6, 4'd1, 4'd2)); addr=addr+1; // R6=R1|R2=11
        // 10 NOPs to flush WB through the pipeline
        repeat(10) begin write_imem(addr, `NOP); addr=addr+1; end

        // --- Execute ---
        run_cycles(50);

        // --- Check results ---
        check_reg(4'd3, 32'h0000_000D, "ADD R3=R1+R2");
        check_reg(4'd4, 32'h0000_0007, "SUB R4=R1-R2");
        check_reg(4'd5, 32'h0000_0002, "AND R5=R1&R2");
        check_reg(4'd6, 32'h0000_000B, "ORR R6=R1|R2");

        // =====================================================================
        // ?? TEST 2: Immediate ADD ?????????????????????????????????????????????
        // =====================================================================
        $display("\n--- TEST 2: Immediate ADD ---");
        do_reset;
        addr = 0;
        // MOV R1, #20 then ADD R2, R1, #5  ? R2 = 25
        write_imem(addr, `MOV_IMM(4'd1, 8'd20));         addr=addr+1;
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `ADD_IMM(4'd2, 4'd1, 8'd5));    addr=addr+1;
        repeat(10) begin write_imem(addr, `NOP); addr=addr+1; end

        run_cycles(25);
        check_reg(4'd2, 32'h0000_0019, "ADD_IMM R2=R1+5");

        // =====================================================================
        // ?? TEST 3: Load / Store ??????????????????????????????????????????????
        // =====================================================================
        $display("\n--- TEST 3: Load / Store ---");
        do_reset;
        addr = 0;
        //  MOV R7, #0       base address = byte addr 0
        //  MOV R8, #0xAB    value to store
        //  NOP x5
        //  STR R8,[R7,#0]   dmem word[0] ? 0xAB
        //  NOP x5
        //  LDR R9,[R7,#0]   R9 ? dmem word[0]
        //  NOP x10
        write_imem(addr, `MOV_IMM(4'd7, 8'h00));              addr=addr+1;
        write_imem(addr, `MOV_IMM(4'd8, 8'hAB));              addr=addr+1;
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `STR_IMM(4'd8, 4'd7, 12'h000));      addr=addr+1;
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        write_imem(addr, `LDR_IMM(4'd9, 4'd7, 12'h000));      addr=addr+1;
        repeat(10) begin write_imem(addr, `NOP); addr=addr+1; end

        run_cycles(40);

        // Check via register file that LDR loaded the value back
        check_reg(4'd9, 32'h0000_00AB, "LDR R9=dmem[0]");
        // Also check data memory directly via debug port
        // BRAM word address 0 corresponds to byte address 0 (addr[9:2]=0)
        check_dmem(8'h00, 32'h0000_00AB, "STR dmem[0]=0xAB");

        // =====================================================================
        // ?? TEST 4: Branch (B) ? forward skip ????????????????????????????????
        // =====================================================================
        // Program layout (word addresses):
        //   0   MOV  R10, #0
        //   1   NOP x5  (1?5) ? hazard guard before the branch itself
        //   6   B  +2          skip words 7 and 8
        //   7   ADD R10,R10,#1  ? should be skipped
        //   8   ADD R10,R10,#1  ? should be skipped
        //   9   MOV R10, #0xFF  ? branch lands here
        //  10   NOP x10
        //
        // Branch offset encoding:
        //   branch_target = pc_in + 2 + offset_sext  (from stage_ID)
        //   pc_in at word 6 = 6
        //   desired target  = 9
        //   offset_sext = 9 - 6 - 2 = 1
        //   So encode offset24 = 24'd1
        $display("\n--- TEST 4: Branch (B) forward skip ---");
        do_reset;
        addr = 0;
        write_imem(addr, `MOV_IMM(4'd10, 8'd0));              addr=addr+1; // word 0
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end           // words 1-5
        write_imem(addr, `B_OFFSET(24'd1));                    addr=addr+1; // word 6  B +1
        write_imem(addr, `ADD_IMM(4'd10,4'd10, 8'd1));        addr=addr+1; // word 7  SKIP
        write_imem(addr, `ADD_IMM(4'd10,4'd10, 8'd1));        addr=addr+1; // word 8  SKIP
        write_imem(addr, `MOV_IMM(4'd10, 8'hFF));             addr=addr+1; // word 9  LAND
        repeat(10) begin write_imem(addr, `NOP); addr=addr+1; end

        run_cycles(30);

        // R10 should be 0xFF (branch taken), not 1 or 2 (branch not taken)
        check_reg(4'd10, 32'h0000_00FF, "B fwd: R10=0xFF");

        // =====================================================================
        // ?? TEST 5: Pipeline stall / step mode ???????????????????????????????
        // =====================================================================
        // Verify that run=0 holds the pipeline, and single-step advances by 1.
        $display("\n--- TEST 5: Step mode (PC advances only on step pulse) ---");
        do_reset;
        addr = 0;
        // Simple: MOV R0,#42 at word 0
        write_imem(0, `MOV_IMM(4'd0, 8'd42));
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end

        // With run=0 and step=0, pipeline must stay at PC=0
        repeat(5) @(posedge clk); #1;
        if (pc_dbg === 9'd0) begin
            $display("  PASS  Step: PC held at 0 when run=0,step=0");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL  Step: PC should be 0, got %0d", pc_dbg);
            fail_count = fail_count + 1;
        end

        // One step pulse should advance PC by 1
        @(negedge clk); step = 1'b1;
        @(posedge clk); #1; step = 1'b0;
        @(posedge clk); #1;

        if (pc_dbg === 9'd1) begin
            $display("  PASS  Step: PC advanced to 1 after one step");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL  Step: PC should be 1, got %0d", pc_dbg);
            fail_count = fail_count + 1;
        end

        // =====================================================================
        // ?? TEST 6: pc_reset_pulse ????????????????????????????????????????????
        // =====================================================================
        $display("\n--- TEST 6: pc_reset_pulse ---");
        // Run a few cycles to move PC away from 0
        run_cycles(5);
        // Now pulse pc_reset
        @(negedge clk);
        pc_reset_pulse = 1'b1;
        @(posedge clk); #1;
        pc_reset_pulse = 1'b0;
        @(posedge clk); #1;

        if (pc_dbg === 9'd0) begin
            $display("  PASS  pc_reset_pulse: PC back to 0");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL  pc_reset_pulse: PC should be 0, got %0d", pc_dbg);
            fail_count = fail_count + 1;
        end

        // =====================================================================
        // ?? TEST 7: Data memory preload via debug port ????????????????????????
        // =====================================================================
        // Write a known value into dmem via prog port, then LDR and check register.
        $display("\n--- TEST 7: Data memory preload + LDR ---");
        do_reset;

        // Pre-load dmem word address 5 (byte address 20 = 0x14) with 0xDEAD_BEEF
        @(negedge clk);
        dmem_prog_en   = 1'b1;
        dmem_prog_we   = 1'b1;
        dmem_prog_addr = 8'd5;
        dmem_prog_wdata = 64'h0000_0000_DEAD_BEEF;
        @(negedge clk);
        dmem_prog_we   = 1'b0;
        dmem_prog_en   = 1'b0;

        addr = 0;
        // MOV R1, #20  (byte address = 20 ? BRAM word 5 since addr[9:2]=5)
        write_imem(addr, `MOV_IMM(4'd1, 8'd20));   addr=addr+1;
        repeat(5) begin write_imem(addr, `NOP); addr=addr+1; end
        // LDR R2, [R1, #0]
        write_imem(addr, `LDR_IMM(4'd2, 4'd1, 12'h000)); addr=addr+1;
        repeat(10) begin write_imem(addr, `NOP); addr=addr+1; end

        run_cycles(25);
        check_reg(4'd2, 32'hDEAD_BEEF, "LDR preloaded dmem");

        // =====================================================================
        // ?? SUMMARY ???????????????????????????????????????????????????????????
        // =====================================================================
        $display("\n======================================================");
        $display("  Results:  %0d PASSED   %0d FAILED", pass_count, fail_count);
        $display("======================================================");

        if (fail_count == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  SOME TESTS FAILED ? review waveform (tb_pipeline.vcd)");

        $display("======================================================\n");
        $finish;
    end

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #500000;
        $display("TIMEOUT ? simulation exceeded 500 us");
        $finish;
    end

    // =========================================================================
    // Continuous pipeline state monitor (prints every 10 cycles when running)
    // =========================================================================
    integer cycle_count;
    initial cycle_count = 0;
    always @(posedge clk) begin
        cycle_count = cycle_count + 1;
        if (run && (cycle_count % 10 == 0))
            $display("  [cyc %4d]  PC=%0d  IF_INSTR=0x%08X",
                     cycle_count, pc_dbg, if_instr_dbg);
    end

endmodule