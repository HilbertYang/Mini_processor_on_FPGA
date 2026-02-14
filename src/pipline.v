`timescale 1ns/1ps
module pipeline (
  input  wire        clk,
  input  wire        reset,

  input  wire        run,           
  input  wire        step,          
  input  wire        pc_reset_pulse, 

  //I-mem programming interface
  input  wire        imem_prog_we,
  input  wire [8:0]  imem_prog_addr,
  input  wire [31:0] imem_prog_wdata,

  //D-mem programming interface
  input  wire        dmem_prog_en,
  input  wire        dmem_prog_we,
  input  wire [7:0]  dmem_prog_addr,
  input  wire [63:0] dmem_prog_wdata,
  output wire [63:0] dmem_prog_rdata,

  output wire [8:0]  pc_dbg,
  output wire [31:0] if_instr_dbg
);

//=========================PIPELINE CONTROL LOGIC============================
  reg step_d;

  always @(posedge clk) begin
    if (reset || pc_reset_pulse) begin
        step_d <= 1'b0;
    end else begin       
        step_d <= step;
    end
  end
  
  wire step_pulse = step & ~step_d; //Step pulse is generated when step goes from 0 to 1
  wire advance    = run | step_pulse; 
//=========================IF STAGE============================

  reg  [8:0] pc;
  assign pc_dbg = pc;

  // I-mem instance with programming interface
  wire [8:0]  imem_addr_mux = imem_prog_we ? imem_prog_addr : pc;
  wire [31:0] imem_din_mux  = imem_prog_wdata;
  wire        imem_we_mux   = imem_prog_we;
  wire [31:0] imem_dout;
  


  //----------------------------------------------------------------------------------
  // Instruction Memory: 512x32-bit, single port, with separate programming interface
  //  Inputs:
  //   - addr: 9-bit address (for 512 depth)
  //   - clk: clock signal
  //   - din: 32-bit data input (for programming)
  //   - en: enable signal (always 1 for now)
  //   - we: write enable (1 for programming, 0 for normal read)
  //  Output:
  //   - dout: 32-bit data output (instruction data)
  //  Instruction format (32 bits):
  //   - [31] WMemEn, [30] WRegEn, [29:27] Reg1, [26:24] Reg2, [23:21] WReg1, rest unused
  //-----------------------------------------------------------------------------------
  I_M_32bit_512depth u_imem (
    .addr(imem_addr_mux),
    .clk (clk),
    .din (imem_din_mux),
    .dout(imem_dout),
    .en  (1'b1),
    .we  (imem_we_mux)
  );
  assign if_instr_dbg = imem_dout;//output for debug

  //pc update
  always @(posedge clk) begin
    if (reset) begin
      pc <= 9'd0;
    end else if (pc_reset_pulse) begin
      pc <= 9'd0;
    end else if (advance) begin
      pc <= pc + 9'd1;
    end
  end

// =======================IF/ID STAGE===========================

  // IF/ID -> ID pipeline register
  reg  [31:0] ifid_instr;

  always @(posedge clk) begin
    if (reset) begin
      ifid_instr <= 32'h0;
    end else if (pc_reset_pulse) begin
      ifid_instr <= 32'h0;
    end else if (advance) begin
      ifid_instr <= imem_dout;
    end
  end

//===========================================================
// Instruction format (32 bits):
// [31] WMemEn, [30] WRegEn, [29:27] Reg1, [26:24] Reg2, [23:21] WReg1, rest unused
// =======================ID STAGE===========================



  reg         id_wmem_en;
  reg         id_wreg_en;
  wire [1:0]  id_reg1    = ifid_instr[28:27]; // low 2 bits of [29:27]
  wire [1:0]  id_reg2    = ifid_instr[25:24]; // low 2 bits of [26:24]
  reg  [1:0]  id_wreg;
  reg  [63:0] id_r1data;
  reg  [63:0] id_r2data;
  wire [63:0] rf_r1data;
  wire [63:0] rf_r2data;

  // WB signals (directly from MEM/WB, gated by advance)
  reg        memwb_wreg_en;
  reg [1:0]  memwb_wreg;
  reg [63:0] memwb_dmem_rdata; 

  wire        wb_wen;
  wire [1:0]  wb_waddr;
  wire [63:0] wb_wdata;

  assign wb_wen = (~reset) & (~pc_reset_pulse) & advance & memwb_wreg_en;
  assign wb_waddr = memwb_wreg;
  assign wb_wdata = memwb_dmem_rdata;

  //---------------------------------------------------------------------------------
  // Register File: 4 registers x 64-bit, with 2 read ports and 1 write port
  // Inputs:
  // - clk: clock signal
  // - reset: reset signal
  // - r0addr: 2-bit read address for port 0
  // - r1addr: 2-bit read address for port 1
  // - waddr: 2-bit write address
  // - wdata: 64-bit write data
  // - wen: write enable
  // Outputs:
  // - r0data: 64-bit read data from port 0
  // - r1data: 64-bit read data from port 1
  //---------------------------------------------------------------------------------
  register_file u_rf (
    .clk   (clk),
    .reset (reset),
    .r0addr(id_reg1),
    .r1addr(id_reg2),
    .waddr0(wb_waddr[0]),
    .waddr1(wb_waddr[1]),
    .wdata (wb_wdata),
    .wen   (wb_wen),
    .r0data(rf_r1data),
    .r1data(rf_r2data)
  ); 

  //ID -> ID/EX pipeline registers
  always @(posedge clk) begin
    if (reset) begin
      id_wmem_en <= 1'd0;
      id_wreg_en <= 1'd0;
      id_wreg    <= 2'b00;
    end else if (pc_reset_pulse) begin
      id_wmem_en <= 1'd0;
      id_wreg_en <= 1'd0;
      id_wreg    <= 2'b00;
    end else if (advance) begin
      id_wmem_en <= ifid_instr[31];     
      id_wreg_en <= ifid_instr[30];
      id_r1data <= rf_r1data;
      id_r2data <= rf_r2data;
      id_wreg    <= ifid_instr[22:21];
    end
  end

// ====================ID/EX STAGE========================

  //ID/EX -> EX pipeline registers
  reg        idex_wmem_en;
  reg        idex_wreg_en;
  reg [1:0]  idex_wreg;
  reg [63:0] idex_r1data;
  reg [63:0] idex_r2data;

  always @(posedge clk) begin
    if (reset) begin
      idex_wmem_en <= 1'b0;
      idex_wreg_en <= 1'b0;
      idex_wreg    <= 2'b00;
      idex_r1data   <= 64'h0;
      idex_r2data   <= 64'h0;
    end else if (pc_reset_pulse) begin
      idex_wmem_en <= 1'b0;
      idex_wreg_en <= 1'b0;
      idex_wreg    <= 2'b00;
      idex_r1data   <= 64'h0;
      idex_r2data   <= 64'h0;
    end else if (advance) begin
      idex_wmem_en <= id_wmem_en;
      idex_wreg_en <= id_wreg_en;
      idex_wreg    <= id_wreg;
      idex_r1data   <= id_r1data; 
      idex_r2data   <= id_r2data; 
    end
  end

// ==================EX STAGE========================
  wire [7:0] ex_r1data  = idex_r1data[7:0];
  wire [63:0] ex_r2data = idex_r2data;

  // EX->MEM pipeline registers
  reg        exmem_wmem_en;
  reg        exmem_wreg_en;
  reg [1:0]  exmem_wreg;
  reg [7:0]  exmem_dmem_addr;
  reg [63:0] exmem_store_data;

  always @(posedge clk) begin
    if (reset) begin
      exmem_wmem_en     <= 1'b0;
      exmem_wreg_en     <= 1'b0;
      exmem_wreg        <= 2'b00;
      exmem_dmem_addr        <= 8'h00;
      exmem_store_data  <= 64'h0;
    end else if (pc_reset_pulse) begin
      exmem_wmem_en     <= 1'b0;
      exmem_wreg_en     <= 1'b0;
      exmem_wreg        <= 2'b00;
      exmem_dmem_addr        <= 8'h00;
      exmem_store_data  <= 64'h0;
    end else if (advance) begin
      exmem_wmem_en     <= idex_wmem_en;
      exmem_wreg_en     <= idex_wreg_en;
      exmem_wreg        <= idex_wreg;
      exmem_dmem_addr        <= ex_r1data;
      exmem_store_data  <= ex_r2data;
    end
  end

// ================MEM STAGE========================
  reg mem_wreg_en;
  reg [1:0] mem_wreg;
  wire [63:0] dmem_douta;
  wire [63:0] dmem_doutb;

  //----------------------------------------------------------------------------------
  // Data Memory: 256x64-bit, single port, with separate programming interface
  //  Inputs:
  //   - addra: 8-bit address (for 256 depth)
  //   - clka: clock signal
  //   - dina: 64-bit data input (for programming and store)
  //   - ena: enable signal (always 1 for now)
  //   - wea: write enable (1 for programming/store, 0 for normal read)
  //   - addrb: 8-bit address for programming
  //   - clkb: clock signal for programming
  //   - dinb: 64-bit data input for programming
  //   - enb: enable signal for programming
  //   - web: write enable for programming (1 for write, 0 for read)
  //  Output:
  //   - douta: 64-bit data output for MEM stage
  //   - doutb: 64-bit data output for programming
  //-----------------------------------------------------------------------------------
  D_M_64bit_256 u_dmem (
    // Port A: pipeline
    .addra(exmem_dmem_addr),
    .clka (clk),
    .ena  (1'b1),
    .wea  (exmem_wmem_en),
    .dina (exmem_store_data),
    .douta(dmem_douta),

    // Port B: programming
    .addrb(dmem_prog_addr),
    .clkb (clk),
    .enb  (dmem_prog_en),
    .web  (dmem_prog_we),
    .dinb (dmem_prog_wdata),
    .doutb(dmem_doutb)
  );

  assign dmem_prog_rdata = dmem_doutb;
  
  // MEM->MEM/WB pipeline registers
  always @(posedge clk) begin
    if (reset) begin
      mem_wreg_en   <= 1'b0;
      mem_wreg      <= 2'b00;
    end else if (pc_reset_pulse) begin
      mem_wreg_en   <= 1'b0;
      mem_wreg      <= 2'b00;
    end else if (advance) begin
      mem_wreg_en   <= exmem_wreg_en;
      mem_wreg      <= exmem_wreg;
    end
  end

//===============MEM/WB STAGE========================
// MEM/WB stage is just pipeline registers to hold control signals for WB stage, which is done by connecting EX/MEM pipeline register outputs to MEM/WB pipeline registers, no separate logic needed here.

  //MEM/WB -> WB pipeline registers
  always @(posedge clk) begin
    if (reset) begin
      memwb_dmem_rdata   <= 64'h0;
      memwb_wreg_en   <= 1'b0;
      memwb_wreg      <= 2'b00;
    end else if (pc_reset_pulse) begin
      memwb_dmem_rdata   <= 64'h0;
      memwb_wreg_en   <= 1'b0;
      memwb_wreg      <= 2'b00;
    end else if (advance) begin
      memwb_dmem_rdata   <= dmem_douta;
      memwb_wreg_en   <= mem_wreg_en;
      memwb_wreg      <= mem_wreg;
    end
  end

//===============WB STAGE========================
// WB stage is just writing back to register file, which is done by connecting MEM/WB pipeline register outputs to regfile write port, no separate logic needed here.


endmodule