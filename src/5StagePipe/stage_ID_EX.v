// =============================================================================
// Pipeline Register ID/EX
//
// Latches all signals decoded in ID and passes them to the EX stage.
// On reset (or flush from a taken branch) all control signals go to 0 so
// that the instruction in flight becomes a harmless NOP bubble.
// =============================================================================
module stage_ID_EX #(
    parameter DATA_WIDTH      = 32,
    parameter REG_ADDR_WIDTH  = 4,
    parameter IMEM_ADDR_WIDTH = 9
)(
    input  wire                        clk,
    input  wire                        reset,
    input  wire                        enable,

    // Data inputs
    input  wire [DATA_WIDTH-1:0]       r1_data_in,
    input  wire [DATA_WIDTH-1:0]       r2_data_in,
    input  wire [DATA_WIDTH-1:0]       imm32_in,
    input  wire                        use_imm_in,
    input  wire [REG_ADDR_WIDTH-1:0]   rd_addr_in,

    // ALU control
    input  wire [3:0]                  alu_ctrl_in,

    // Control signals
    input  wire                        reg_wen_in,
    input  wire                        mem_wen_in,
    input  wire                        is_mem_inst_in,
    input  wire                        is_load_in,
    input  wire                        is_branch_in,
    input  wire                        is_jump_in,

    // Branch target and PC
    input  wire [IMEM_ADDR_WIDTH-1:0]  branch_target_in,
    input  wire [IMEM_ADDR_WIDTH-1:0]  pc_in,

    // Outputs
    output reg  [DATA_WIDTH-1:0]       r1_data_out,
    output reg  [DATA_WIDTH-1:0]       r2_data_out,
    output reg  [DATA_WIDTH-1:0]       imm32_out,
    output reg                         use_imm_out,
    output reg  [REG_ADDR_WIDTH-1:0]   rd_addr_out,

    output reg  [3:0]                  alu_ctrl_out,

    output reg                         reg_wen_out,
    output reg                         mem_wen_out,
    output reg                         is_mem_inst_out,
    output reg                         is_load_out,
    output reg                         is_branch_out,
    output reg                         is_jump_out,

    output reg  [IMEM_ADDR_WIDTH-1:0]  branch_target_out,
    output reg  [IMEM_ADDR_WIDTH-1:0]  pc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Insert NOP bubble
            r1_data_out      <= {DATA_WIDTH{1'b0}};
            r2_data_out      <= {DATA_WIDTH{1'b0}};
            imm32_out        <= {DATA_WIDTH{1'b0}};
            use_imm_out      <= 1'b0;
            rd_addr_out      <= {REG_ADDR_WIDTH{1'b0}};
            alu_ctrl_out     <= 4'b0000;
            reg_wen_out      <= 1'b0;
            mem_wen_out      <= 1'b0;
            is_mem_inst_out  <= 1'b0;
            is_load_out      <= 1'b0;
            is_branch_out    <= 1'b0;
            is_jump_out      <= 1'b0;
            branch_target_out<= {IMEM_ADDR_WIDTH{1'b0}};
            pc_out           <= {IMEM_ADDR_WIDTH{1'b0}};
        end else if (enable) begin
            r1_data_out      <= r1_data_in;
            r2_data_out      <= r2_data_in;
            imm32_out        <= imm32_in;
            use_imm_out      <= use_imm_in;
            rd_addr_out      <= rd_addr_in;
            alu_ctrl_out     <= alu_ctrl_in;
            reg_wen_out      <= reg_wen_in;
            mem_wen_out      <= mem_wen_in;
            is_mem_inst_out  <= is_mem_inst_in;
            is_load_out      <= is_load_in;
            is_branch_out    <= is_branch_in;
            is_jump_out      <= is_jump_in;
            branch_target_out<= branch_target_in;
            pc_out           <= pc_in;
        end
    end

endmodule
