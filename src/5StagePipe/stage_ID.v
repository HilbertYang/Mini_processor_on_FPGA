// =============================================================================
// Stage ID – Instruction Decode & Register File Read
//
// ARM-32 instruction encoding handled:
//
//   [31:28] cond  (ignored – no condition codes in this implementation)
//   [27:26] op    00 = data-proc / misc, 01 = load/store, 10 = branch
//
// Data-processing (op=00):
//   [25]    I      1 = Operand2 is 8-bit immediate rotated, 0 = register
//   [24:21] opcode (AND/EOR/SUB/RSB/ADD/ADC/SBC/RSC/TST/TEQ/CMP/CMN/ORR/MOV/BIC/MVN)
//   [20]    S      set condition flags (not fully implemented here)
//   [19:16] Rn     first source register
//   [15:12] Rd     destination register
//   [11:0]  shifter_operand
//
// Load/Store (op=01):
//   [20]    L      1 = load (LDR), 0 = store (STR)
//   [19:16] Rn     base address register
//   [15:12] Rd     source/dest register
//   [11:0]  offset12 (unsigned immediate offset)
//   [23]    U      1 = offset added, 0 = offset subtracted
//
// Branch (op=10):
//   [24]    L      1 = BL (branch-with-link saves PC+1 to R14)
//   [23:0]  signed offset (word offset from PC+2 in pipeline)
//
// BX (branch-and-exchange) encoded as:
//   inst[27:4] == 24'h12FFF1, inst[3:0] = Rm
//
// ALU control values (must match ALU.v):
//   ADD=0001  SUB=0010  AND=0011  OR=0100  XNOR=0101  SHIFTL=0110  SHIFTR=0111
//
// WB write-back inputs are fed directly to the register file so the latest
// committed value is always visible on the next read (no forwarding – the
// programmer must insert NOPs to avoid data hazards).
// =============================================================================
module stage_ID #(
    parameter DATA_WIDTH      = 32,
    parameter REG_ADDR_WIDTH  = 4,   // 16 ARM registers (R0-R15)
    parameter IMEM_ADDR_WIDTH = 9
)(
    input  wire                        clk,
    input  wire                        reset,

    // Instruction and PC from IF/ID register
    input  wire [31:0]                 inst,
    input  wire [IMEM_ADDR_WIDTH-1:0]  pc_in,

    // Write-back bus (from WB stage, fed directly into register file)
    input  wire                        wb_wen,
    input  wire [REG_ADDR_WIDTH-1:0]   wb_waddr,
    input  wire [DATA_WIDTH-1:0]       wb_wdata,

    // Register read results
    output wire [DATA_WIDTH-1:0]       r1_data_out,   // Rn value
    output wire [DATA_WIDTH-1:0]       r2_data_out,   // Rm / Rd(store) value

    // Destination register address forwarded to EX/MEM/WB for writeback
    output wire [REG_ADDR_WIDTH-1:0]   rd_addr_out,

    // ALU control
    output reg  [3:0]                  alu_ctrl,

    // Immediate value (32-bit zero/sign-extended) and select
    output reg  [DATA_WIDTH-1:0]       imm32,
    output reg                         use_imm,       // 1 = use imm32 instead of r2

    // Control signals
    output reg                         reg_wen,       // writeback to register file
    output reg                         mem_wen,       // write to data memory (STR)
    output reg                         is_mem_inst,   // LDR or STR
    output reg                         is_load,       // specifically LDR (read from mem)
    output reg                         is_branch,     // B or BL
    output reg                         is_jump,       // BX

    // Computed branch / jump target (word address)
    output reg  [IMEM_ADDR_WIDTH-1:0]  branch_target,

    // PC passthrough for EX stage
    output wire [IMEM_ADDR_WIDTH-1:0]  pc_out
);

    // -------------------------------------------------------------------------
    // Instruction field extraction
    // -------------------------------------------------------------------------
    wire [1:0]  op          = inst[27:26];
    wire        imm_flag    = inst[25];   // I bit
    wire [3:0]  opcode      = inst[24:21];
    wire [3:0]  rn          = inst[19:16];
    wire [3:0]  rd          = inst[15:12];
    wire [3:0]  rm          = inst[3:0];
    wire [11:0] operand2    = inst[11:0];
    wire [23:0] offset24    = inst[23:0];
    wire        u_bit       = inst[23];   // add/sub offset in LDR/STR
    wire        load_bit    = inst[20];   // L bit
    wire        link_bit    = inst[24];   // BL

    assign rd_addr_out = rd;
    assign pc_out      = pc_in;

    // -------------------------------------------------------------------------
    // Register file
    // -------------------------------------------------------------------------
    // r0addr reads Rn (always first source / base address)
    // r1addr reads Rm for data-proc, or Rd for STR (store source)
    // For stores (op=01, L=0) we need to read Rd as the data to store.
    wire [REG_ADDR_WIDTH-1:0] r1addr_sel = (op == 2'b01 && !load_bit) ? rd : rm;

    REG_FILE #(
        .data_width (DATA_WIDTH),
        .addr_width (REG_ADDR_WIDTH)
    ) regfile (
        .clk    (clk),
        .wena   (wb_wen),
        .r0addr (rn),
        .r1addr (r1addr_sel),
        .waddr  (wb_waddr),
        .wdata  (wb_wdata),
        .r0data (r1_data_out),
        .r1data (r2_data_out)
    );

    // -------------------------------------------------------------------------
    // Branch target: PC + 2 + sign_extend(offset24)
    // +2 accounts for the two pipeline stages between fetch and execute
    // -------------------------------------------------------------------------
    // wire signed [IMEM_ADDR_WIDTH-1:0] offset_sext =  {{(IMEM_ADDR_WIDTH-24){offset24[23]}}, offset24};
	 wire signed [IMEM_ADDR_WIDTH-1:0] offset_sext =  {offset24[23], offset24[IMEM_ADDR_WIDTH-2:0]};

    // -------------------------------------------------------------------------
    // Decoder
    // -------------------------------------------------------------------------
    // ALU opcode constants (match ALU.v)
    localparam ALU_ADD    = 4'b0001;
    localparam ALU_SUB    = 4'b0010;
    localparam ALU_AND    = 4'b0011;
    localparam ALU_OR     = 4'b0100;
    localparam ALU_XNOR   = 4'b0101;
    localparam ALU_SHIFTL = 4'b0110;
    localparam ALU_SHIFTR = 4'b0111;
    localparam ALU_NOP    = 4'b0000;

    always @(*) begin
        // Safe defaults – treat unknown instruction as NOP
        alu_ctrl      = ALU_NOP;
        imm32         = {DATA_WIDTH{1'b0}};
        use_imm       = 1'b0;
        reg_wen       = 1'b0;
        mem_wen       = 1'b0;
        is_mem_inst   = 1'b0;
        is_load       = 1'b0;
        is_branch     = 1'b0;
        is_jump       = 1'b0;
        branch_target = {IMEM_ADDR_WIDTH{1'b0}};

        case (op)

            // ------------------------------------------------------------------
            // 2'b00 – Data-processing & Miscellaneous
            // ------------------------------------------------------------------
            2'b00: begin
                // BX: inst[27:4] == 24'h12FFF1
                if (inst[27:4] == 24'h12FFF1) begin
                    is_jump = 1'b1;
                    // Target comes from Rm (r2_data_out), resolved in EX stage
                end else begin
                    // Immediate operand (I=1): 8-bit immediate zero-extended
                    // (Full ARM rotation not implemented; sufficient for basic immediates)
                    if (imm_flag) begin
                        use_imm = 1'b1;
                        imm32   = {{24{1'b0}}, operand2[7:0]};
                    end

                    case (opcode)
                        4'b0100: begin // ADD
                            alu_ctrl = ALU_ADD;
                            reg_wen  = 1'b1;
                        end
                        4'b0010: begin // SUB
                            alu_ctrl = ALU_SUB;
                            reg_wen  = 1'b1;
                        end
                        4'b0011: begin // RSB  (reverse subtract: Rm - Rn)
                            // Handled in EX by swapping A/B; use SUB with swapped operands
                            alu_ctrl = ALU_SUB;
                            reg_wen  = 1'b1;
                        end
                        4'b0000: begin // AND
                            alu_ctrl = ALU_AND;
                            reg_wen  = 1'b1;
                        end
                        4'b1100: begin // ORR
                            alu_ctrl = ALU_OR;
                            reg_wen  = 1'b1;
                        end
                        4'b0001: begin // EOR (XOR – mapped to XNOR for available op)
                            alu_ctrl = ALU_XNOR;
                            reg_wen  = 1'b1;
                        end
                        4'b1101: begin // MOV  (Rd = Operand2 = Rm or imm)
                            // Implement as ADD with Rn=R0 (zero) when Rn=0, or use imm
                            alu_ctrl = ALU_ADD;
                            reg_wen  = 1'b1;
                        end
                        4'b1111: begin // MVN  (Rd = NOT Rm)
                            // XNOR with all-ones ≡ NOT; handled via ALU_XNOR + one's comp
                            alu_ctrl = ALU_XNOR;
                            reg_wen  = 1'b1;
                        end
                        4'b1010: begin // CMP  (SUB, discard result, update flags)
                            alu_ctrl = ALU_SUB;
                            reg_wen  = 1'b0;  // result not written back
                        end
                        4'b1000: begin // TST  (AND, discard result)
                            alu_ctrl = ALU_AND;
                            reg_wen  = 1'b0;
                        end
                        4'b1001: begin // TEQ  (EOR, discard result)
                            alu_ctrl = ALU_XNOR;
                            reg_wen  = 1'b0;
                        end
                        4'b1010: begin // CMN (ADD, discard result) -- overlaps CMP above
                            alu_ctrl = ALU_ADD;
                            reg_wen  = 1'b0;
                        end
                        4'b0101: begin // ADC (ADD + carry, treat as ADD)
                            alu_ctrl = ALU_ADD;
                            reg_wen  = 1'b1;
                        end
                        4'b0110: begin // SBC (SUB - borrow, treat as SUB)
                            alu_ctrl = ALU_SUB;
                            reg_wen  = 1'b1;
                        end
                        default: begin
                            alu_ctrl = ALU_NOP;
                            reg_wen  = 1'b0;
                        end
                    endcase
                end
            end

            // ------------------------------------------------------------------
            // 2'b01 – Load / Store (LDR / STR)
            // ------------------------------------------------------------------
            2'b01: begin
                is_mem_inst = 1'b1;
                // Effective address = Rn +/- offset12
                alu_ctrl = u_bit ? ALU_ADD : ALU_SUB;
                use_imm  = 1'b1;
                imm32    = {{20{1'b0}}, operand2};  // 12-bit unsigned offset

                if (load_bit) begin
                    is_load = 1'b1;
                    reg_wen = 1'b1;
                    mem_wen = 1'b0;
                end else begin
                    is_load = 1'b0;
                    reg_wen = 1'b0;
                    mem_wen = 1'b1;
                end
            end

            // ------------------------------------------------------------------
            // 2'b10 – Branch (B / BL)
            // ------------------------------------------------------------------
            2'b10: begin
                if (inst[27:25] == 3'b101) begin
                    is_branch     = 1'b1;
                    branch_target = pc_in + {{(IMEM_ADDR_WIDTH-2){offset_sext[IMEM_ADDR_WIDTH-1]}}, offset_sext} + 9'd2;
                    alu_ctrl      = ALU_NOP;
                    // BL: save return address (pc_in + 1) in R14 – handled in EX
                    reg_wen = link_bit;
                end
            end

            default: begin /* NOP */ end
        endcase
    end

endmodule
