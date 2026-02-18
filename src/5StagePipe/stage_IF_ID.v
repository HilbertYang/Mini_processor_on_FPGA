// =============================================================================
// Pipeline Register IF/ID
//
// Latches the fetched instruction and the PC that produced it.
// The PC is needed in ID so the branch target (PC + offset) can be computed.
// On reset (or flush), outputs NOP (all zeros).
// =============================================================================
module stage_IF_ID #(
    parameter IMEM_ADDR_WIDTH = 9
)(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       enable,

    input  wire [31:0]                inst_in,
    input  wire [IMEM_ADDR_WIDTH-1:0] pc_in,

    output [31:0]                inst_out,
    output reg  [IMEM_ADDR_WIDTH-1:0] pc_out
);

	 assign inst_out = reset ? inst_in : 32'b0;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            //inst_out <= 32'b0;
            pc_out   <= {IMEM_ADDR_WIDTH{1'b0}};
        end else if (enable) begin
            //inst_out <= inst_in;
            pc_out   <= pc_in;
        end
    end

endmodule
