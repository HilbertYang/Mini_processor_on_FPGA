// =============================================================================
// pipeline.v â Top-level pipeline module
//
// Instantiates all five pipeline stages and the four inter-stage registers.
// Port list matches exactly what pipeline_top_regs.v expects.
//
// Pipeline structure:
//
//   ââââââââââ  IF/ID  ââââââââââ  ID/EX  ââââââââââ  EX/MEM  ââââââââââ  MEM/WB  ââââââââââ
//   â  IF    ââââââââââ¶â  ID    âââââââââââ¶â  EX    ââââââââââââ¶â  MEM   ââââââââââââ¶â  WB    â
//   ââââââââââ  reg    ââââââââââ  reg     ââââââââââ  reg      ââââââââââ  reg      ââââââââââ
//        â²                                      â pc_write / new_pc
//        ââââââââââââââââââââââââââââââââââââââââ  (branch/jump redirect)
//
//                            WB write-back
//        ââââââââââââââââââââââââââââââââââââââââââââââââââââââââ
//        â   reg_wen, rd_addr, wdata  âââ WB stage âââ MEM/WB  â
//        ââââââââââââââââââââ ID stage register file âââââââââââ
//
// Clock enable strategy:
//   run=1, step=0 : pipeline runs freely (enable = 1 every cycle)
//   run=0, step=1 : single-step (enable is a one-cycle pulse)
//   run=0, step=0 : pipeline halted
//
// Branch flush:
//   When a taken branch/jump is detected in EX, the IF and IF/ID stages must
//   be flushed (their instructions were fetched speculatively and are wrong).
//   We do this by asserting reset_ifid for one cycle, which inserts a NOP
//   bubble.  The ID/EX register is also flushed similarly.
//   (No branch prediction; two bubbles are inserted per taken branch.)
//
// Parameters:
//   DATA_WIDTH      â 32 for ARM-32
//   REG_ADDR_WIDTH  â 4  for 16 ARM registers (R0-R15)
//   IMEM_ADDR_WIDTH â 9  matches I_M_32bit_512depth (512 words)
// =============================================================================

`timescale 1ns/1ps

module pipeline #(
    parameter DATA_WIDTH      = 32,
    parameter REG_ADDR_WIDTH  = 4,
    parameter IMEM_ADDR_WIDTH = 9
)(
    input  wire        clk,
    input  wire        reset,

    // Run / step control (from SW registers)
    input  wire        run,             // 1 = free-run
    input  wire        step,            // pulse = single step
    input  wire        pc_reset_pulse,  // pulse = reset PC to 0 without full reset

    // Instruction memory programming port
    input  wire        imem_prog_we,
    input  wire [8:0]  imem_prog_addr,
    input  wire [31:0] imem_prog_wdata,

    // Data memory programming / readback port
    input  wire        dmem_prog_en,
    input  wire        dmem_prog_we,
    input  wire [7:0]  dmem_prog_addr,
    input  wire [63:0] dmem_prog_wdata,
    output wire [63:0] dmem_prog_rdata,

    // Debug outputs
    output wire [8:0]  pc_dbg,
    output wire [31:0] if_instr_dbg
);

    // =========================================================================
    // Clock enable generation
    // =========================================================================
    wire pipe_enable = run | step;

    // =========================================================================
    // Inter-stage wires
    // =========================================================================

    // ââ IF ââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââ
    wire [31:0]                if_inst;
    wire [IMEM_ADDR_WIDTH-1:0] if_pc;

    // ââ IF/ID register outputs ââââââââââââââââââââââââââââââââââââââââââââââââ
    wire [31:0]                ifid_inst;
    wire [IMEM_ADDR_WIDTH-1:0] ifid_pc;

    // ââ ID ââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââ
    wire [DATA_WIDTH-1:0]      id_r1_data;
    wire [DATA_WIDTH-1:0]      id_r2_data;
    wire [REG_ADDR_WIDTH-1:0]  id_rd_addr;
    wire [3:0]                 id_alu_ctrl;
    wire [DATA_WIDTH-1:0]      id_imm32;
    wire                       id_use_imm;
    wire                       id_reg_wen;
    wire                       id_mem_wen;
    wire                       id_is_mem_inst;
    wire                       id_is_load;
    wire                       id_is_branch;
    wire                       id_is_jump;
    wire [IMEM_ADDR_WIDTH-1:0] id_branch_target;
    wire [IMEM_ADDR_WIDTH-1:0] id_pc_out;

    // ââ ID/EX register outputs ââââââââââââââââââââââââââââââââââââââââââââââââ
    wire [DATA_WIDTH-1:0]      idex_r1_data;
    wire [DATA_WIDTH-1:0]      idex_r2_data;
    wire [DATA_WIDTH-1:0]      idex_imm32;
    wire                       idex_use_imm;
    wire [REG_ADDR_WIDTH-1:0]  idex_rd_addr;
    wire [3:0]                 idex_alu_ctrl;
    wire                       idex_reg_wen;
    wire                       idex_mem_wen;
    wire                       idex_is_mem_inst;
    wire                       idex_is_load;
    wire                       idex_is_branch;
    wire                       idex_is_jump;
    wire [IMEM_ADDR_WIDTH-1:0] idex_branch_target;
    wire [IMEM_ADDR_WIDTH-1:0] idex_pc;

    // ââ EX ââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââ
    wire [DATA_WIDTH-1:0]      ex_alu_result;
    wire [DATA_WIDTH-1:0]      ex_store_data;
    wire [REG_ADDR_WIDTH-1:0]  ex_rd_addr;
    wire                       ex_reg_wen;
    wire                       ex_mem_wen;
    wire                       ex_is_mem_inst;
    wire                       ex_is_load;
    wire                       ex_pc_write;        // branch/jump redirect
    wire [IMEM_ADDR_WIDTH-1:0] ex_new_pc;

    // ââ EX/MEM register outputs âââââââââââââââââââââââââââââââââââââââââââââââ
    wire [DATA_WIDTH-1:0]      exmem_alu_result;
    wire [DATA_WIDTH-1:0]      exmem_store_data;
    wire [REG_ADDR_WIDTH-1:0]  exmem_rd_addr;
    wire                       exmem_reg_wen;
    wire                       exmem_mem_wen;
    wire                       exmem_is_mem_inst;
    wire                       exmem_is_load;
    wire                       exmem_clk;          // wire clock feed-through

    // ââ MEM âââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââ
    wire [REG_ADDR_WIDTH-1:0]  mem_rd_addr;
    wire                       mem_reg_wen;
    wire [DATA_WIDTH-1:0]      mem_wdata;

    // ââ MEM/WB register outputs âââââââââââââââââââââââââââââââââââââââââââââââ
    wire                       memwb_reg_wen;
    wire [REG_ADDR_WIDTH-1:0]  memwb_rd_addr;
    wire [DATA_WIDTH-1:0]      memwb_wdata;

    // ââ WB ââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââââ
    wire                       wb_reg_wen;
    wire [REG_ADDR_WIDTH-1:0]  wb_rd_addr;
    wire [DATA_WIDTH-1:0]      wb_wdata;

    // =========================================================================
    // Branch flush logic
    // When EX detects a taken branch/jump:
    //   - Flush IF/ID register  â replace fetched instruction with NOP
    //   - Flush ID/EX register  â replace decoded instruction with NOP
    //   - Redirect PC in IF stage (pc_write / new_pc)
    // We hold the flush for one cycle using a registered copy of pc_write.
    // =========================================================================
    reg branch_flush_r;
    always @(posedge clk or posedge reset) begin
        if (reset)
            branch_flush_r <= 1'b0;
        else
            branch_flush_r <= ex_pc_write & pipe_enable;
    end

    wire flush_ifid = (ex_pc_write | branch_flush_r) & pipe_enable;
    wire flush_idex = (ex_pc_write | branch_flush_r) & pipe_enable;

    // =========================================================================
    // Stage IF
    // =========================================================================
    stage_IF #(
        .IMEM_ADDR_WIDTH (IMEM_ADDR_WIDTH)
    ) u_if (
        .clk             (clk),
        .reset           (reset | pc_reset_pulse),
        .enable          (pipe_enable),
        .pc_write        (ex_pc_write),
        .pc_in           (ex_new_pc),
        .imem_prog_we    (imem_prog_we),
        .imem_prog_addr  (imem_prog_addr),
        .imem_prog_wdata (imem_prog_wdata),
        .inst            (if_inst),
        .pc_out          (if_pc)
    );

    // Debug outputs
    assign pc_dbg       = if_pc;
    assign if_instr_dbg = if_inst;

    // =========================================================================
    // Pipeline Register IF/ID
    // =========================================================================
    stage_IF_ID #(
        .IMEM_ADDR_WIDTH (IMEM_ADDR_WIDTH)
    ) u_if_id (
        .clk      (clk),
        .reset    (reset | flush_ifid),
        .enable   (pipe_enable),
        .inst_in  (if_inst),
        .pc_in    (if_pc),
        .inst_out (ifid_inst),
        .pc_out   (ifid_pc)
    );

    // =========================================================================
    // Stage ID
    // =========================================================================
    stage_ID #(
        .DATA_WIDTH      (DATA_WIDTH),
        .REG_ADDR_WIDTH  (REG_ADDR_WIDTH),
        .IMEM_ADDR_WIDTH (IMEM_ADDR_WIDTH)
    ) u_id (
        .clk            (clk),
        .reset          (reset),
        .inst           (ifid_inst),
        .pc_in          (ifid_pc),
        .wb_wen         (wb_reg_wen),
        .wb_waddr       (wb_rd_addr),
        .wb_wdata       (wb_wdata),
        .r1_data_out    (id_r1_data),
        .r2_data_out    (id_r2_data),
        .rd_addr_out    (id_rd_addr),
        .alu_ctrl       (id_alu_ctrl),
        .imm32          (id_imm32),
        .use_imm        (id_use_imm),
        .reg_wen        (id_reg_wen),
        .mem_wen        (id_mem_wen),
        .is_mem_inst    (id_is_mem_inst),
        .is_load        (id_is_load),
        .is_branch      (id_is_branch),
        .is_jump        (id_is_jump),
        .branch_target  (id_branch_target),
        .pc_out         (id_pc_out)
    );

    // =========================================================================
    // Pipeline Register ID/EX
    // =========================================================================
    stage_ID_EX #(
        .DATA_WIDTH      (DATA_WIDTH),
        .REG_ADDR_WIDTH  (REG_ADDR_WIDTH),
        .IMEM_ADDR_WIDTH (IMEM_ADDR_WIDTH)
    ) u_id_ex (
        .clk               (clk),
        .reset             (reset | flush_idex),
        .enable            (pipe_enable),
        .r1_data_in        (id_r1_data),
        .r2_data_in        (id_r2_data),
        .imm32_in          (id_imm32),
        .use_imm_in        (id_use_imm),
        .rd_addr_in        (id_rd_addr),
        .alu_ctrl_in       (id_alu_ctrl),
        .reg_wen_in        (id_reg_wen),
        .mem_wen_in        (id_mem_wen),
        .is_mem_inst_in    (id_is_mem_inst),
        .is_load_in        (id_is_load),
        .is_branch_in      (id_is_branch),
        .is_jump_in        (id_is_jump),
        .branch_target_in  (id_branch_target),
        .pc_in             (id_pc_out),
        .r1_data_out       (idex_r1_data),
        .r2_data_out       (idex_r2_data),
        .imm32_out         (idex_imm32),
        .use_imm_out       (idex_use_imm),
        .rd_addr_out       (idex_rd_addr),
        .alu_ctrl_out      (idex_alu_ctrl),
        .reg_wen_out       (idex_reg_wen),
        .mem_wen_out       (idex_mem_wen),
        .is_mem_inst_out   (idex_is_mem_inst),
        .is_load_out       (idex_is_load),
        .is_branch_out     (idex_is_branch),
        .is_jump_out       (idex_is_jump),
        .branch_target_out (idex_branch_target),
        .pc_out            (idex_pc)
    );

    // =========================================================================
    // Stage EX
    // =========================================================================
    stage_EX #(
        .DATA_WIDTH      (DATA_WIDTH),
        .REG_ADDR_WIDTH  (REG_ADDR_WIDTH),
        .IMEM_ADDR_WIDTH (IMEM_ADDR_WIDTH)
    ) u_ex (
        .r1_data         (idex_r1_data),
        .r2_data         (idex_r2_data),
        .imm32           (idex_imm32),
        .use_imm         (idex_use_imm),
        .rd_addr_in      (idex_rd_addr),
        .alu_ctrl        (idex_alu_ctrl),
        .reg_wen_in      (idex_reg_wen),
        .mem_wen_in      (idex_mem_wen),
        .is_mem_inst_in  (idex_is_mem_inst),
        .is_load_in      (idex_is_load),
        .is_branch_in    (idex_is_branch),
        .is_jump_in      (idex_is_jump),
        .branch_target_in(idex_branch_target),
        .pc_in           (idex_pc),
        .alu_result      (ex_alu_result),
        .store_data      (ex_store_data),
        .rd_addr_out     (ex_rd_addr),
        .reg_wen_out     (ex_reg_wen),
        .mem_wen_out     (ex_mem_wen),
        .is_mem_inst_out (ex_is_mem_inst),
        .is_load_out     (ex_is_load),
        .pc_write        (ex_pc_write),
        .new_pc          (ex_new_pc)
    );

    // =========================================================================
    // Pipeline Register EX/MEM
    // =========================================================================
    stage_EX_MEM #(
        .DATA_WIDTH     (DATA_WIDTH),
        .REG_ADDR_WIDTH (REG_ADDR_WIDTH)
    ) u_ex_mem (
        .clk             (clk),
        .reset           (reset),
        .enable          (pipe_enable),
        .alu_result_in   (ex_alu_result),
        .store_data_in   (ex_store_data),
        .rd_addr_in      (ex_rd_addr),
        .reg_wen_in      (ex_reg_wen),
        .mem_wen_in      (ex_mem_wen),
        .is_mem_inst_in  (ex_is_mem_inst),
        .is_load_in      (ex_is_load),
        .alu_result_out  (exmem_alu_result),
        .store_data_out  (exmem_store_data),
        .rd_addr_out     (exmem_rd_addr),
        .reg_wen_out     (exmem_reg_wen),
        .mem_wen_out     (exmem_mem_wen),
        .is_mem_inst_out (exmem_is_mem_inst),
        .is_load_out     (exmem_is_load),
        .clk_out         (exmem_clk)
    );

    // =========================================================================
    // Stage MEM
    // =========================================================================
    stage_MEM #(
        .DATA_WIDTH     (DATA_WIDTH),
        .REG_ADDR_WIDTH (REG_ADDR_WIDTH)
    ) u_mem (
        .clk             (exmem_clk),
        .alu_result      (exmem_alu_result),
        .store_data      (exmem_store_data),
        .rd_addr_in      (exmem_rd_addr),
        .reg_wen_in      (exmem_reg_wen),
        .mem_wen         (exmem_mem_wen),
        .is_mem_inst     (exmem_is_mem_inst),
        .is_load         (exmem_is_load),
        .dmem_prog_en    (dmem_prog_en),
        .dmem_prog_we    (dmem_prog_we),
        .dmem_prog_addr  (dmem_prog_addr),
        .dmem_prog_wdata (dmem_prog_wdata),
        .dmem_prog_rdata (dmem_prog_rdata),
        .rd_addr_out     (mem_rd_addr),
        .reg_wen_out     (mem_reg_wen),
        .wdata_out       (mem_wdata)
    );

    // =========================================================================
    // Pipeline Register MEM/WB
    // =========================================================================
    stage_MEM_WB #(
        .DATA_WIDTH     (DATA_WIDTH),
        .REG_ADDR_WIDTH (REG_ADDR_WIDTH)
    ) u_mem_wb (
        .clk         (clk),
        .reset       (reset),
        .enable      (pipe_enable),
        .reg_wen_in  (mem_reg_wen),
        .rd_addr_in  (mem_rd_addr),
        .wdata_in    (mem_wdata),
        .reg_wen_out (memwb_reg_wen),
        .rd_addr_out (memwb_rd_addr),
        .wdata_out   (memwb_wdata)
    );

    // =========================================================================
    // Stage WB
    // =========================================================================
    stage_WB #(
        .DATA_WIDTH     (DATA_WIDTH),
        .REG_ADDR_WIDTH (REG_ADDR_WIDTH)
    ) u_wb (
        .reg_wen_in  (memwb_reg_wen),
        .rd_addr_in  (memwb_rd_addr),
        .wdata_in    (memwb_wdata),
        .reg_wen_out (wb_reg_wen),
        .rd_addr_out (wb_rd_addr),
        .wdata_out   (wb_wdata)
    );

endmodule
