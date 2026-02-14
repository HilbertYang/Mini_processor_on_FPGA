// Code your design here

module REG_FILE #(parameter data_width = 32, parameter addr_width = 2) (
  input clk, wena,
  input [addr_width-1:0] r0addr, r1addr, waddr,
  input [data_width-1:0] wdata,
  output [data_width-1:0] r0data, r1data);
  
  reg [data_width-1:0] regFile [2**addr_width];
  
  assign r0data = regFile[r0addr];
  assign r1data = regFile[r1addr];
  
  localparam zero_value = {data_width{1'b0}};
  
  always @ (posedge clk) begin
    if (wena) begin
    	regFile[waddr] <= wdata;  
    end
  end
  
  
endmodule
