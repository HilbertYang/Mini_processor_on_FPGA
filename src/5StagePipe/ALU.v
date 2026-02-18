module ALU #(parameter data_width = 32) (
  input [data_width-1:0] A, B,
  input [3:0] aluctrl,
  output reg [data_width-1:0] Z,
  output reg overflow);
  
  localparam ALU_ADD 	= 4'b0001;
  localparam ALU_SUB 	= 4'b0010;
  localparam ALU_AND 	= 4'b0011;
  localparam ALU_OR 	= 4'b0100;
  localparam ALU_XNOR 	= 4'b0101;
  localparam ALU_SHIFTL = 4'b0110;
  localparam ALU_SHIFTR = 4'b0111;
  
  always @ (*) begin
    overflow = 1'b0;
    case (aluctrl)
      ALU_ADD : {overflow, Z} = A + B;
      ALU_SUB : {overflow, Z} = A - B;
      ALU_AND : Z = A & B;
      ALU_OR : Z = A | B;
      ALU_XNOR : Z = ~(A ^ B);
      ALU_SHIFTL : Z = A << 1;
      ALU_SHIFTR : Z = A >> 1;
      default : Z = {data_width{1'b0}};
      endcase
  end
endmodule
