// Code your testbench here
// or browse Examples

module ALU_TEST ();
  localparam ALU_ADD 	= 4'b0001;
  localparam ALU_SUB 	= 4'b0010;
  localparam ALU_AND 	= 4'b0011;
  localparam ALU_OR 	= 4'b0100;
  localparam ALU_XNOR 	= 4'b0101;
  localparam ALU_SHIFTL = 4'b0110;
  localparam ALU_SHIFTR = 4'b0111;
  
  localparam data_width = 32;

  
  class genData;
    randc bit [data_width-1:0] A, B;
  	rand bit [3:0] aluctrl;
  	constraint aluIN {aluctrl < 8; aluctrl > 0;}
  endclass
  
function automatic logic [32:0] aluOUT (input logic [31:0] A_in, input logic [31:0] B_in, input logic [3:0] ctrl);
    logic        ovf;
    logic [31:0] Z_ref;
    begin
      ovf = 1'b0;
      case (ctrl)
        ALU_ADD : {ovf, Z_ref} = A_in + B_in;
        ALU_SUB : Z_ref = A_in - B_in;
        ALU_AND : Z_ref = A_in & B_in;
        ALU_OR  : Z_ref = A_in | B_in;
        ALU_XNOR: Z_ref = ~(A_in ^ B_in);
        ALU_SHIFTL: Z_ref = A_in << 1;
        ALU_SHIFTR: Z_ref = A_in >> 1;
        default: Z_ref = '0;
      endcase
      aluOUT = {ovf, Z_ref};
    end
  endfunction
  
  
  genData aluD = new();
  logic [data_width-1:0] A, B, Z;
  logic [3:0] aluctrl;
  logic clk, overflow;
  logic [32:0] dut_out, ref_out, ref_out_prev;
  assign dut_out = {overflow, Z};
  ALU #(.data_width(data_width)) ALU_DUT(A, B, aluctrl, clk, Z, overflow);

  //aluD = new ();
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end
  initial begin
    for (integer i = 0; i < 100; i++) begin
      aluD.randomize();
      A = aluD.A;
      B = aluD.B;
      aluctrl = aluD.aluctrl;
      @(posedge clk);
      ref_out_prev = ref_out;
      ref_out = aluOUT(A, B, aluctrl);
      
      if (dut_out !== ref_out_prev) begin
        $display("Mismatch at iter %0d: ctrl=%0d A=%0h B=%0h | DUT={ovf=%0b, Z=%0h} REF={ovf=%0b, Z=%0h} time = %t",
                 i, aluctrl, A, B, dut_out[32], dut_out[31:0], ref_out[32], ref_out[31:0], $time);
      end
      else begin
        $display("OK iter %0d ctrl=%0d Z=%08h ovf=%0b, time = %t", i, aluctrl, Z, overflow, $time);
      end
      #10;
    end
    
  end
endmodule
