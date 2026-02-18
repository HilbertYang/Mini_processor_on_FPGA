// =============================================================================
// Pipeline Register MEM/WB
//
// Latches the write-back data and destination register address from the MEM
// stage and presents them to the WB stage one cycle later.
// Reset drives all outputs to known-zero values (not 'x').
// =============================================================================
module stage_MEM_WB #(
    parameter DATA_WIDTH     = 32,
    parameter REG_ADDR_WIDTH = 4
)(
    input  wire                      clk,
    input  wire                      reset,
    input  wire                      enable,

    input  wire                      reg_wen_in,
    input  wire [REG_ADDR_WIDTH-1:0] rd_addr_in,
    input  wire [DATA_WIDTH-1:0]     wdata_in,

    output reg                       reg_wen_out,
    output reg  [REG_ADDR_WIDTH-1:0] rd_addr_out,
    output  [DATA_WIDTH-1:0]     wdata_out
);

	assign wdata_out = wdata_in;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            reg_wen_out <= 1'b0;
            rd_addr_out <= {REG_ADDR_WIDTH{1'b0}};
            //wdata_out   <= {DATA_WIDTH{1'b0}};
        end else if (enable) begin
            reg_wen_out <= reg_wen_in;
            rd_addr_out <= rd_addr_in;
            //wdata_out   <= wdata_in;
        end
    end

endmodule
