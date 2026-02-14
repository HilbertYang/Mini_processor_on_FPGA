`timescale 1ns/1ps

module tb_pipeline;

  reg         clk;
  reg         reset;

  reg         run;
  reg         step;
  reg         pc_reset_pulse;

  reg         imem_prog_we;
  reg  [8:0]  imem_prog_addr;
  reg  [31:0] imem_prog_wdata;

  reg         dmem_prog_en;
  reg         dmem_prog_we;
  reg  [7:0]  dmem_prog_addr;
  reg  [63:0] dmem_prog_wdata;
  wire [63:0] dmem_prog_rdata;

  wire [8:0]  pc_dbg;
  wire [31:0] if_instr_dbg;

  pipeline dut (
    .clk(clk),
    .reset(reset),

    .run(run),
    .step(step),
    .pc_reset_pulse(pc_reset_pulse),

    .imem_prog_we(imem_prog_we),
    .imem_prog_addr(imem_prog_addr),
    .imem_prog_wdata(imem_prog_wdata),

    .dmem_prog_en(dmem_prog_en),
    .dmem_prog_we(dmem_prog_we),
    .dmem_prog_addr(dmem_prog_addr),
    .dmem_prog_wdata(dmem_prog_wdata),
    .dmem_prog_rdata(dmem_prog_rdata),

    .pc_dbg(pc_dbg),
    .if_instr_dbg(if_instr_dbg)
  );

//=========================CLK GENERATION============================
  localparam CLK_PERIOD = 10;//10ns -> 100MHz
  initial clk = 1'b0;
  always #(CLK_PERIOD/2) clk = ~clk;


//=========================TASKS============================
  //Write to I-mem 
  task program_imem_word;
    input [8:0]  addr;
    input [31:0] data;
    begin

      run  = 1'b0;
      step = 1'b0;

      imem_prog_addr  = addr;
      imem_prog_wdata = data;
      imem_prog_we    = 1'b1;

      @(posedge clk);
      #1;
      imem_prog_we    = 1'b0;
    end
  endtask

  // Write to D-mem
  task program_dmem_word;
    input [7:0]  addr;
    input [63:0] data;
    begin
      run  = 1'b0;
      step = 1'b0;

      dmem_prog_addr  = addr;
      dmem_prog_wdata = data;
      dmem_prog_en    = 1'b1;
      dmem_prog_we    = 1'b1;

      @(posedge clk);
      #1;
      dmem_prog_we    = 1'b0;
      dmem_prog_en    = 1'b0;
    end
  endtask

  // Read from D-mem
  task read_dmem_word;
    input  [7:0]  addr;
    output [63:0] data;
    begin
      run  = 1'b0;
      step = 1'b0;

      dmem_prog_addr = addr;
      dmem_prog_en   = 1'b1;
      dmem_prog_we   = 1'b0;
      dmem_prog_wdata= 64'h0;

      @(posedge clk);
      #1;
      data = dmem_prog_rdata;
      dmem_prog_en   = 1'b0;


    end
  endtask

  // pcrest
  task pulse_pc_reset;
    begin
      pc_reset_pulse = 1'b1;
      @(posedge clk);
      #1;
      pc_reset_pulse = 1'b0;
      @(posedge clk);
      #1;
    end
  endtask

//=========================FUNCTIONS============================
  // -----------------------
  // Instruction encoding helper
  // [31] WMemEn, [30] WRegEn,
  // [28:27] Reg1 (addr source, low 2 bits),
  // [25:24] Reg2 (store data source, low 2 bits),
  // [22:21] WReg (dest, low 2 bits)
  // -----------------------
  function [31:0] make_instr;
    input wmem_en;
    input wreg_en;
    input [1:0] reg1;
    input [1:0] reg2;
    input [1:0] wreg;
    begin
      make_instr = 32'h0;
      make_instr[31]    = wmem_en;
      make_instr[30]    = wreg_en;
      make_instr[28:27] = reg1;
      make_instr[25:24] = reg2;
      make_instr[22:21] = wreg;
    end
  endfunction


  // Monitor writeback
  always @(posedge clk) begin
    if (!reset) begin
      if (dut.wb_wen) begin
        $display("[%0t] WB: wen=1 waddr=%0d wdata=0x%016h  (pc=%0d ifid_instr=0x%08h)",
                 $time, dut.wb_waddr, dut.wb_wdata, pc_dbg, if_instr_dbg);
      end
    end
  end

//=========================TEST SEQUENCE============================
  reg [63:0] rd0, rd4_before, rd4_after;
  initial begin
    reset          = 1'b1;
    run            = 1'b0;
    step           = 1'b0;
    pc_reset_pulse = 1'b0;

    imem_prog_we    = 1'b0;
    imem_prog_addr  = 9'd0;
    imem_prog_wdata = 32'h0;

    dmem_prog_en    = 1'b0;
    dmem_prog_we    = 1'b0;
    dmem_prog_addr  = 8'd0;
    dmem_prog_wdata = 64'h0;

    // reset a few cycles
    repeat (3) @(posedge clk);
    #1 reset = 1'b0;
    @(posedge clk);
    #1;

    // Program D-mem
    // D-mem[0] = 4
    // D-mem[4] = 100
    $display("== Program D-mem[0]=4, D-mem[4]=100 ==");
    program_dmem_word(8'd0, 64'd4);
    program_dmem_word(8'd4, 64'd100);

    // Read back and check
    read_dmem_word(8'd0, rd0);
    read_dmem_word(8'd4, rd4_before);
    $display("[%0t] Readback: D[0]=%0d (0x%h), D[4]=%0d (0x%h)",
             $time, rd0, rd0, rd4_before, rd4_before);

    if (rd0 !== 64'd4) begin
      $display("ERROR: D[0] not 4!");
      $stop;
    end
    if (rd4_before !== 64'd100) begin
      $display("ERROR: D[4] not 100!");
      $stop;
    end

//==========================INSTRUCTION PROGRAMMING===========================
    // Program I-mem
    // Addr0: Load D-mem[Reg0=0] -> Reg2
    // Addr1: Load D-mem[Reg0=0] -> Reg3
    // Addr2: NOP
    // Addr3: NOP
    // Addr4: NOP
    // Addr5: Store Reg3 -> D-mem[Reg2]
    // Instruction format (32 bits):
    // [31] WMemEn, 
    // [30] WRegEn, 
    // [29:27] Reg1(dmem_addr), 
    // [26:24] Reg2(dmem_store_data), 
    // [23:21] WReg1(reg_w_target_addr), 
    // rest unused

    $display("== Program I-mem (doc sequence) ==");
    // load to Reg2
    program_imem_word(9'd0, make_instr(1'b0, 1'b1, 2'b00, 2'b00, 2'b10));
    // load to Reg3
    program_imem_word(9'd1, make_instr(1'b0, 1'b1, 2'b00, 2'b00, 2'b11));
    // NOPs
    program_imem_word(9'd2, 32'h0);
    program_imem_word(9'd3, 32'h0);
    program_imem_word(9'd4, 32'h0);
    program_imem_word(9'd5, 32'h0);
    program_imem_word(9'd6, 32'h0);
    program_imem_word(9'd7, 32'h0);
    // store Reg3 -> D-mem[Reg2]
    program_imem_word(9'd8, make_instr(1'b1, 1'b0, 2'b10, 2'b11, 2'b00));

//==========================TEST SEQUENCE CONTINUED===========================
    $display("== Pulse PC reset ==");
    pulse_pc_reset();

    //Run PC
    $display("== Run CPU ==");
    run = 1'b1;
    repeat (20) @(posedge clk);
    #1;
    run = 1'b0;

//================================TEST SEQUENCE END===========================
    //Read back
    $display("== Check result: D-mem[4] should be 4 ==");
    read_dmem_word(8'd4, rd4_after);
    $display("[%0t] After run: D[4]=%0d (0x%h)", $time, rd4_after, rd4_after);
    if (rd4_after !== 64'd4) begin
      $display("ERROR: Expected D[4]=4 after store, got %0d (0x%h)", rd4_after, rd4_after);
      $stop;
    end
    $display("PASS:Program worked! D[4] changed 100 -> 4.");
    $finish;
  end

endmodule
