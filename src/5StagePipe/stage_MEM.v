// =============================================================================
// Stage MEM – Data Memory Access
//
// Uses D_M_64bit_256 – a Xilinx dual-port BRAM:
//   Port A (pipeline): 8-bit address, 64-bit data, synchronous read/write
//   Port B (debug):    8-bit address, 64-bit data, host read/write
//
// The pipeline works with 32-bit data.  The 64-bit BRAM stores one 32-bit
// word in bits [31:0]; bits [63:32] are always written as zero and ignored
// on reads.  This keeps the design simple while using the available IP.
//
// LDR: alu_result is the byte address; upper bits select the BRAM word.
//      mem_data_out = lower 32 bits of BRAM output.
// STR: store_data_in written to bits [31:0]; upper 32 bits forced to 0.
//
// For non-memory instructions (is_mem_inst=0) the ALU result passes through
// unchanged as the write-back data.
//
// dmem_prog_* ports allow the host to read/write the data memory directly
// (used by pipeline_top_regs for initialisation and result readback).
// =============================================================================
module stage_MEM #(
    parameter DATA_WIDTH     = 32,
    parameter REG_ADDR_WIDTH = 4
)(
    input  wire                      clk,

    // From EX/MEM pipeline register
    input  wire [DATA_WIDTH-1:0]     alu_result,      // effective address (byte addr)
    input  wire [DATA_WIDTH-1:0]     store_data,      // data to write (STR)
    input  wire [REG_ADDR_WIDTH-1:0] rd_addr_in,
    input  wire                      reg_wen_in,
    input  wire                      mem_wen,         // 1 = STR (write memory)
    input  wire                      is_mem_inst,     // 1 = LDR or STR
    input  wire                      is_load,         // 1 = LDR (read memory)

    // Host debug port B – allows host to read/write data memory
    input  wire                      dmem_prog_en,
    input  wire                      dmem_prog_we,
    input  wire [7:0]                dmem_prog_addr,
    input  wire [63:0]               dmem_prog_wdata,
    output wire [63:0]               dmem_prog_rdata,

    // To MEM/WB pipeline register
    output wire [REG_ADDR_WIDTH-1:0] rd_addr_out,
    output wire                      reg_wen_out,
    output wire [DATA_WIDTH-1:0]     wdata_out        // data to write back to reg file
);

    // -------------------------------------------------------------------------
    // Pipeline port A signals
    // -------------------------------------------------------------------------
    // Use bits [9:2] of the byte address as the 8-bit BRAM word address.
    // (Byte address / 4 → lower 8 bits of the result)
    wire [7:0]  pa_addr  = alu_result[9:2];
    wire [63:0] pa_din   = {32'b0, store_data};   // zero-extend to 64 bits
    wire        pa_we    = mem_wen;
    wire        pa_en    = is_mem_inst;
    wire [63:0] pa_dout;

    // -------------------------------------------------------------------------
    // Data memory – D_M_64bit_256
    // Port A = pipeline, Port B = host debug
    // -------------------------------------------------------------------------
    D_M_64bit_256 dmem (
        // Port A – pipeline
        .addra (pa_addr),
        .clka  (clk),
        .dina  (pa_din),
        .douta (pa_dout),
        .ena   (pa_en | pa_we),   // enable on any access
        .wea   (pa_we),
        // Port B – host debug
        .addrb (dmem_prog_addr),
        .clkb  (clk),
        .dinb  (dmem_prog_wdata),
        .doutb (dmem_prog_rdata),
        .enb   (dmem_prog_en),
        .web   (dmem_prog_we)
    );

    // -------------------------------------------------------------------------
    // Write-back data mux
    //   LDR  → lower 32 bits from BRAM
    //   ALU  → alu_result passes through
    // -------------------------------------------------------------------------
    assign wdata_out   = (is_mem_inst && is_load) ? pa_dout[31:0] : alu_result;

    // Pass-through control signals
    assign rd_addr_out = rd_addr_in;
    assign reg_wen_out = reg_wen_in;

endmodule
