// =============================================================================
// pipeline_top_regs.v – Top-level wrapper: pipeline + memory-mapped registers
//
// SW (software) registers – written by host, read by pipeline:
//   [0] sw_ctrl          bit[0]=run, bit[1]=step, bit[2]=pc_reset,
//                        bit[3]=imem_we, bit[4]=dmem_prog_en, bit[5]=dmem_prog_we
//   [1] sw_imem_addr     instruction memory write address (9-bit)
//   [2] sw_imem_wdata    instruction memory write data (32-bit)
//   [3] sw_dmem_addr     data memory address (8-bit)
//   [4] sw_dmem_wdata_lo data memory write data [31:0]
//   [5] sw_dmem_wdata_hi data memory write data [63:32]
//   [6] sw_reserved      (unused)
//
// HW (hardware) registers – written by pipeline, read by host:
//   [0] hw_pc_dbg        current PC value
//   [1] hw_if_instr      instruction currently in IF stage
//   [2] hw_dmem_rdata_lo data memory read data [31:0]
//   [3] hw_dmem_rdata_hi data memory read data [63:32]
// =============================================================================

`timescale 1ns/1ps
`define UDP_REG_ADDR_WIDTH              23
`define CPCI_NF2_DATA_WIDTH  32

module pipeline_top_regs #(
    parameter DATA_WIDTH        = 64,
    parameter CTRL_WIDTH        = DATA_WIDTH/8,
    parameter UDP_REG_SRC_WIDTH = 2
)(
    input  wire                              clk,
    input  wire                              reset,

    input  wire                              reg_req_in,
    input  wire                              reg_ack_in,
    input  wire                              reg_rd_wr_L_in,
    input  wire [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_in,
    input  wire [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_in,
    input  wire [UDP_REG_SRC_WIDTH-1:0]      reg_src_in,

    output wire                              reg_req_out,
    output wire                              reg_ack_out,
    output wire                              reg_rd_wr_L_out,
    output wire [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_out,
    output wire [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_out,
    output wire [UDP_REG_SRC_WIDTH-1:0]      reg_src_out,

    output wire [8:0]                        pc_dbg,
    output wire [31:0]                       if_instr_dbg
);

    // =========================================================================
    // Software registers (host → pipeline)
    // =========================================================================
    wire [31:0] sw_ctrl;
    wire [31:0] sw_imem_addr;
    wire [31:0] sw_imem_wdata;
    wire [31:0] sw_dmem_addr;
    wire [31:0] sw_dmem_wdata_lo;
    wire [31:0] sw_dmem_wdata_hi;
    wire [31:0] sw_reserved;
    reg  [31:0] sw_ctrl_d;

    // Edge-detect for pulse signals
    always @(posedge clk) begin
        if (reset) sw_ctrl_d <= 32'h0;
        else       sw_ctrl_d <= sw_ctrl;
    end

    wire run_level      =  sw_ctrl[0];
    wire step_pulse     =  sw_ctrl[1] & ~sw_ctrl_d[1];
    wire pc_reset_pulse =  sw_ctrl[2] & ~sw_ctrl_d[2];
    wire imem_we_pulse  =  sw_ctrl[3] & ~sw_ctrl_d[3];
    wire dmem_prog_en   =  sw_ctrl[4];
    wire dmem_prog_we   =  sw_ctrl[5];

    wire        imem_prog_we    = imem_we_pulse;
    wire [8:0]  imem_prog_addr  = sw_imem_addr[8:0];
    wire [31:0] imem_prog_wdata = sw_imem_wdata;
    wire [7:0]  dmem_prog_addr  = sw_dmem_addr[7:0];
    wire [63:0] dmem_prog_wdata = {sw_dmem_wdata_hi, sw_dmem_wdata_lo};
    wire [63:0] dmem_prog_rdata;

    // =========================================================================
    // Pipeline instantiation
    // =========================================================================
    pipeline u_pipeline (
        .clk             (clk),
        .reset           (reset),
        .run             (run_level),
        .step            (step_pulse),
        .pc_reset_pulse  (pc_reset_pulse),
        .imem_prog_we    (imem_prog_we),
        .imem_prog_addr  (imem_prog_addr),
        .imem_prog_wdata (imem_prog_wdata),
        .dmem_prog_en    (dmem_prog_en),
        .dmem_prog_we    (dmem_prog_we),
        .dmem_prog_addr  (dmem_prog_addr),
        .dmem_prog_wdata (dmem_prog_wdata),
        .dmem_prog_rdata (dmem_prog_rdata),
        .pc_dbg          (pc_dbg),
        .if_instr_dbg    (if_instr_dbg)
    );

    // =========================================================================
    // Hardware register bus (pipeline → host)
    // FIX: assign individual hw_ wires FROM their sources, then pack into bus.
    // The original code had the assignment direction backwards.
    // =========================================================================
    wire [31:0] hw_pc_dbg        = {23'h0, pc_dbg};
    wire [31:0] hw_if_instr      = if_instr_dbg;
    wire [31:0] hw_dmem_rdata_lo = dmem_prog_rdata[31:0];
    wire [31:0] hw_dmem_rdata_hi = dmem_prog_rdata[63:32];

    // Pack HW regs into bus: index 0 at LSB, index 3 at MSB
    wire [4*32-1:0] hardware_regs_bus = {hw_dmem_rdata_hi,
                                         hw_dmem_rdata_lo,
                                         hw_if_instr,
                                         hw_pc_dbg};

    // Pack SW regs (written by generic_regs, index 0 at LSB)
    wire [7*32-1:0] software_regs_bus;
    assign {sw_reserved,
            sw_dmem_wdata_hi,
            sw_dmem_wdata_lo,
            sw_dmem_addr,
            sw_imem_wdata,
            sw_imem_addr,
            sw_ctrl}       = software_regs_bus;

    // =========================================================================
    // Generic register interface
    // =========================================================================
    generic_regs #(
        .UDP_REG_SRC_WIDTH (UDP_REG_SRC_WIDTH),
        .TAG               (`PIPE_BLOCK_ADDR),
        .REG_ADDR_WIDTH    (`PIPE_REG_ADDR_WIDTH),
        .NUM_COUNTERS      (0),
        .NUM_SOFTWARE_REGS (7),
        .NUM_HARDWARE_REGS (4)
    ) u_regs (
        .reg_req_in        (reg_req_in),
        .reg_ack_in        (reg_ack_in),
        .reg_rd_wr_L_in    (reg_rd_wr_L_in),
        .reg_addr_in       (reg_addr_in),
        .reg_data_in       (reg_data_in),
        .reg_src_in        (reg_src_in),

        .reg_req_out       (reg_req_out),
        .reg_ack_out       (reg_ack_out),
        .reg_rd_wr_L_out   (reg_rd_wr_L_out),
        .reg_addr_out      (reg_addr_out),
        .reg_data_out      (reg_data_out),
        .reg_src_out       (reg_src_out),

        .counter_updates   (),
        .counter_decrement (),

        .software_regs     (software_regs_bus),
        .hardware_regs     (hardware_regs_bus),

        .clk               (clk),
        .reset             (reset)
    );

endmodule
