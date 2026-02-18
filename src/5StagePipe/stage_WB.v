// =============================================================================
// Stage WB – Write Back
//
// Purely combinational stage: passes reg_wen, destination register address,
// and write-back data directly to the ID stage's register file write port.
// No logic changes needed here; kept as a named stage for structural clarity.
// =============================================================================
module stage_WB #(
    parameter DATA_WIDTH     = 32,
    parameter REG_ADDR_WIDTH = 4
)(
    input  wire                      reg_wen_in,
    input  wire [REG_ADDR_WIDTH-1:0] rd_addr_in,
    input  wire [DATA_WIDTH-1:0]     wdata_in,

    output wire                      reg_wen_out,
    output wire [REG_ADDR_WIDTH-1:0] rd_addr_out,
    output wire [DATA_WIDTH-1:0]     wdata_out
);

    assign reg_wen_out  = reg_wen_in;
    assign rd_addr_out  = rd_addr_in;
    assign wdata_out    = wdata_in;

endmodule
