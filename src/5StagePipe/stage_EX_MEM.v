// =============================================================================
// Pipeline Register EX/MEM
//
// Latches the ALU result (used as memory address for LDR/STR, or as write-back
// data for arithmetic), the store data, and all control signals.
//
// clk_out is a simple wire (not a register) so the data memory receives the
// real clock without any additional latency or metastability.
// =============================================================================
module stage_EX_MEM #(
    parameter DATA_WIDTH     = 32,
    parameter REG_ADDR_WIDTH = 4
)(
    input  wire                      clk,
    input  wire                      reset,
    input  wire                      enable,

    // From EX
    input  wire [DATA_WIDTH-1:0]     alu_result_in,
    input  wire [DATA_WIDTH-1:0]     store_data_in,
    input  wire [REG_ADDR_WIDTH-1:0] rd_addr_in,
    input  wire                      reg_wen_in,
    input  wire                      mem_wen_in,
    input  wire                      is_mem_inst_in,
    input  wire                      is_load_in,

    // To MEM
    output reg  [DATA_WIDTH-1:0]     alu_result_out,
    output reg  [DATA_WIDTH-1:0]     store_data_out,
    output reg  [REG_ADDR_WIDTH-1:0] rd_addr_out,
    output reg                       reg_wen_out,
    output reg                       mem_wen_out,
    output reg                       is_mem_inst_out,
    output reg                       is_load_out,

    // Clock feed-through for data memory (must be a wire, not registered)
    output wire                      clk_out
);

    assign clk_out = clk;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            alu_result_out  <= {DATA_WIDTH{1'b0}};
            store_data_out  <= {DATA_WIDTH{1'b0}};
            rd_addr_out     <= {REG_ADDR_WIDTH{1'b0}};
            reg_wen_out     <= 1'b0;
            mem_wen_out     <= 1'b0;
            is_mem_inst_out <= 1'b0;
            is_load_out     <= 1'b0;
        end else if (enable) begin
            alu_result_out  <= alu_result_in;
            store_data_out  <= store_data_in;
            rd_addr_out     <= rd_addr_in;
            reg_wen_out     <= reg_wen_in;
            mem_wen_out     <= mem_wen_in;
            is_mem_inst_out <= is_mem_inst_in;
            is_load_out     <= is_load_in;
        end
    end

endmodule
