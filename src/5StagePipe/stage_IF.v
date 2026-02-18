// =============================================================================
// Stage IF – Instruction Fetch
//
// - Uses I_M_32bit_512depth (Xilinx BRAM, 9-bit word address, 32-bit data)
// - PC is a 9-bit word address (one 32-bit instruction per location)
// - Branch/jump: loads pc_in when pc_write asserted (takes priority over increment)
// - Programming mode: host can write instructions via imem_prog_* port;
//   address/data/we are muxed into the BRAM and the PC is held
// - pc_out exposed so IF/ID register can latch it alongside the instruction
// =============================================================================
module stage_IF #(
    parameter IMEM_ADDR_WIDTH = 9   // 512-deep word-addressed instruction memory
)(
    input  wire                       clk,
    input  wire                       reset,
    input  wire                       enable,          // 1 = pipeline may advance

    // Branch / jump redirect (from EX stage)
    input  wire                       pc_write,
    input  wire [IMEM_ADDR_WIDTH-1:0] pc_in,

    // Host programming port
    input  wire                       imem_prog_we,
    input  wire [IMEM_ADDR_WIDTH-1:0] imem_prog_addr,
    input  wire [31:0]                imem_prog_wdata,

    // Outputs
    output wire [31:0]                inst,
    output wire [IMEM_ADDR_WIDTH-1:0] pc_out
);

    reg [IMEM_ADDR_WIDTH-1:0] pc;
    assign pc_out = pc;

    // Mux BRAM inputs: host program port overrides pipeline read
    wire [IMEM_ADDR_WIDTH-1:0] bram_addr = imem_prog_we ? imem_prog_addr : pc;
    wire [31:0]                bram_din  = imem_prog_we ? imem_prog_wdata : 32'b0;
    wire                       bram_we   = imem_prog_we;

    I_M_32bit_512depth imem (
        .addr (bram_addr),
        .clk  (clk),
        .din  (bram_din),
        .dout (inst),
        .en   (1'b1),
        .we   (bram_we)
    );

    // PC register: reset → 0, else branch target or +1 (word-addressed)
    always @(posedge clk or posedge reset) begin
        if (reset)
            pc <= {IMEM_ADDR_WIDTH{1'b0}};
        else if (!imem_prog_we && enable) begin
            if (pc_write)
                pc <= pc_in;        // branch / jump taken
            else
                pc <= pc + 9'd1;    // normal advance
        end
        // hold PC while host is programming or pipeline is stalled
    end

endmodule
