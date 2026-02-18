// =============================================================================
// Stage EX – Execute
//
// - Selects ALU operand B from register or immediate (use_imm)
// - Computes ALU result; result is either a data value or a memory address
// - Resolves branch condition and outputs pc_write / new_pc to steer IF
// - For BL: writes return address (pc+1) to Rd=R14; handled by passing
//   (pc+1) as alu_result and asserting reg_wen
// - For BX (is_jump): branch target = r2_data[IMEM_ADDR_WIDTH-1:0]
// - For B/BL (is_branch): branch target comes from ID-stage computation
// - Flags (N/Z/C/V simplified): zero and overflow only, enough for B/BL
//
// NOTE: condition-code-based conditional execution is not implemented;
// all branches are treated as unconditional (always taken).
// =============================================================================
module stage_EX #(
    parameter DATA_WIDTH      = 32,
    parameter REG_ADDR_WIDTH  = 4,
    parameter IMEM_ADDR_WIDTH = 9
)(
    input  wire [DATA_WIDTH-1:0]       r1_data,        // Rn value
    input  wire [DATA_WIDTH-1:0]       r2_data,        // Rm / store-data value
    input  wire [DATA_WIDTH-1:0]       imm32,
    input  wire                        use_imm,
    input  wire [REG_ADDR_WIDTH-1:0]   rd_addr_in,
    input  wire [3:0]                  alu_ctrl,

    // Control in
    input  wire                        reg_wen_in,
    input  wire                        mem_wen_in,
    input  wire                        is_mem_inst_in,
    input  wire                        is_load_in,
    input  wire                        is_branch_in,
    input  wire                        is_jump_in,

    input  wire [IMEM_ADDR_WIDTH-1:0]  branch_target_in,
    input  wire [IMEM_ADDR_WIDTH-1:0]  pc_in,

    // ALU / address result
    output wire [DATA_WIDTH-1:0]       alu_result,     // goes to MEM as addr or WB as data
    output wire [DATA_WIDTH-1:0]       store_data,     // r2_data forwarded for STR

    // Control out (pass-through + branch signals)
    output wire [REG_ADDR_WIDTH-1:0]   rd_addr_out,
    output wire                        reg_wen_out,
    output wire                        mem_wen_out,
    output wire                        is_mem_inst_out,
    output wire                        is_load_out,

    // Branch / jump to IF
    output wire                        pc_write,       // 1 = redirect PC
    output wire [IMEM_ADDR_WIDTH-1:0]  new_pc          // target to load into PC
);

    // -------------------------------------------------------------------------
    // ALU operand B mux
    // -------------------------------------------------------------------------
    wire [DATA_WIDTH-1:0] alu_B = use_imm ? imm32 : r2_data;

    // -------------------------------------------------------------------------
    // ALU instance
    // -------------------------------------------------------------------------
    wire [DATA_WIDTH-1:0] alu_out;
    wire                  alu_ovf;

    ALU #(.data_width(DATA_WIDTH)) EX_ALU (
        .A        (r1_data),
        .B        (alu_B),
        .aluctrl  (alu_ctrl),
        .Z        (alu_out),
        .overflow (alu_ovf)
    );

    // -------------------------------------------------------------------------
    // Result selection
    // For BL the writeback data should be the return address (pc_in + 1)
    // -------------------------------------------------------------------------
    wire is_bl = is_branch_in & reg_wen_in;  // BL sets reg_wen
    assign alu_result = is_bl ? {{(DATA_WIDTH-IMEM_ADDR_WIDTH){1'b0}}, pc_in + 9'd1}
                               : alu_out;

    // Store data is always r2_data (the value to write to memory for STR)
    assign store_data = r2_data;

    // -------------------------------------------------------------------------
    // Branch / jump resolution
    // All branches taken unconditionally (no condition code evaluation)
    // -------------------------------------------------------------------------
    assign pc_write = is_branch_in | is_jump_in;

    // BX target comes from the lower bits of r2_data (register Rm)
    assign new_pc   = is_jump_in   ? r2_data[IMEM_ADDR_WIDTH-1:0]
                                   : branch_target_in;

    // -------------------------------------------------------------------------
    // Pass-through control signals
    // -------------------------------------------------------------------------
    assign rd_addr_out      = rd_addr_in;
    assign reg_wen_out      = reg_wen_in;
    assign mem_wen_out      = mem_wen_in;
    assign is_mem_inst_out  = is_mem_inst_in;
    assign is_load_out      = is_load_in;

endmodule
