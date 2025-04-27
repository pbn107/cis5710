`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31:0

// insns are 32 bits in RV32IM
`define INSN_SIZE 31:0

// RV opcodes are 7 bits
`define OPCODE_SIZE 6:0

`define ADDR_WIDTH 32
`define DATA_WIDTH 32

`ifndef DIVIDER_STAGES
`define DIVIDER_STAGES 8
`endif

`ifndef SYNTHESIS
  `include "../hw3-singlecycle/RvDisassembler.sv"
`endif
`include "../hw2b-cla/cla.sv"
`include "../hw4-multicycle/DividerUnsignedPipelined.sv"
`include "../hw5-pipelined/cycle_status.sv"
`include "AxilCache_ref.sv"

module Disasm #(
    PREFIX = "D"
) (
    input wire [31:0] insn,
    output wire [(8*32)-1:0] disasm
);
`ifndef RISCV_FORMAL
`ifndef SYNTHESIS
  // this code is only for simulation, not synthesis
  string disasm_string;
  always_comb begin
    disasm_string = rv_disasm(insn);
  end
  // HACK: get disasm_string to appear in GtkWave, which can apparently show only wire/logic. Also,
  // string needs to be reversed to render correctly.
  genvar i;
  for (i = 3; i < 32; i = i + 1) begin : gen_disasm
    assign disasm[((i+1-3)*8)-1-:8] = disasm_string[31-i];
  end
  assign disasm[255-:8] = PREFIX;
  assign disasm[247-:8] = ":";
  assign disasm[239-:8] = " ";
`endif
`endif
endmodule

// TODO: copy over your RegFile and pipeline structs from HW5
module RegFile (
    input logic [4:0] rd,
    input logic [`REG_SIZE] rd_data,
    input logic [4:0] rs1,
    output logic [`REG_SIZE] rs1_data,
    input logic [4:0] rs2,
    output logic [`REG_SIZE] rs2_data,

    input logic clk,
    input logic we,
    input logic rst
);
  localparam int NumRegs = 32;

  logic [`REG_SIZE] regs[NumRegs];

  // TODO: your code here
  assign regs[0] = 32'b0;
  assign rs1_data = regs[rs1];
  assign rs2_data = regs[rs2];

  always_ff @(posedge clk) begin
    if (rst) begin
      for (int j = 0; j < NumRegs; j = j + 1) begin
        regs[j] <= 32'b0;
      end
    end else if (we && rd != 0) begin
      regs[rd] <= rd_data;
    end
  end
endmodule

/** state at the start of Decode stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
} stage_decode_t;

/** state at the start of Execute stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  cycle_status_e cycle_status;
  logic stall;
  logic [4:0] rd;
  logic [4:0] rs1;
  logic [4:0] rs2;
  logic [3:0] div_cycle_count;
  logic [`REG_SIZE] rs1_data;
  logic [`REG_SIZE] rs2_data;
  logic [`REG_SIZE] imm12_izext;
  logic [`REG_SIZE] imm12_isext;
  logic [`REG_SIZE] imm20;
  logic [`REG_SIZE] imm_b_sext;
  logic [`REG_SIZE] imm_j_sext;
  logic [`REG_SIZE] imm_s_sext;
  logic [`OPCODE_SIZE] opcode;
} stage_execute_t;

/** state at the start of Memory stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  logic [4:0] rd;
  logic [4:0] rs2;
  //logic stall;
  logic we;
  logic memOp;
  logic regOp;
  logic [`REG_SIZE] rd_data;
  logic mx_bypass;
  logic [`REG_SIZE] imm12_izext;
  logic [`REG_SIZE] imm12_isext;
  logic [`REG_SIZE] addr_to_dmem;
  logic [`REG_SIZE] store_data_to_dmem;
  logic [3:0] store_we_to_dmem;
  logic [`REG_SIZE] imm20;
  logic halt;
  logic [`OPCODE_SIZE] opcode;
  cycle_status_e cycle_status;
} stage_memory_t;

/** state at the start of Writeback stage */
typedef struct packed {
  logic [`REG_SIZE] pc;
  logic [`INSN_SIZE] insn;
  //logic [`REG_SIZE] alu_result;
  //logic [`REG_SIZE] mem_result;
  logic [4:0] rd;
  logic we;
  //logic stall;
  logic memOp;
  logic regOp;
  logic halt;
  logic [`REG_SIZE] rd_data;
  cycle_status_e cycle_status;
} stage_writeback_t;

module DatapathPipelinedCache (
    input wire clk,
    input wire rst,

    // AXIL interface to insn memory
    axi_if.manager icache,
    // AXIL interface to data memory/cache
    axi_if.manager dcache,

    output logic halt,

    // The PC of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`REG_SIZE] trace_writeback_pc,
    // The bits of the insn currently in Writeback. 0 if not a valid insn.
    output logic [`INSN_SIZE] trace_writeback_insn,
    // The status of the insn (or stall) currently in Writeback. See the cycle_status.sv file for valid values.
    output cycle_status_e trace_writeback_cycle_status
);

  localparam bit True = 1'b1;
  localparam bit False = 1'b0;

  // opcodes - see section 19 of RiscV spec
  localparam bit [`OPCODE_SIZE] OpLoad = 7'b00_000_11;
  localparam bit [`OPCODE_SIZE] OpStore = 7'b01_000_11;
  localparam bit [`OPCODE_SIZE] OpBranch = 7'b11_000_11;
  localparam bit [`OPCODE_SIZE] OpJalr = 7'b11_001_11;
  localparam bit [`OPCODE_SIZE] OpMiscMem = 7'b00_011_11;
  localparam bit [`OPCODE_SIZE] OpJal = 7'b11_011_11;

  localparam bit [`OPCODE_SIZE] OpRegImm = 7'b00_100_11;
  localparam bit [`OPCODE_SIZE] OpRegReg = 7'b01_100_11;
  localparam bit [`OPCODE_SIZE] OpEnviron = 7'b11_100_11;

  localparam bit [`OPCODE_SIZE] OpAuipc = 7'b00_101_11;
  localparam bit [`OPCODE_SIZE] OpLui = 7'b01_101_11;

  // cycle counter
  logic [`REG_SIZE] cycles_current;
  always_ff @(posedge clk) begin
    if (rst) begin
      cycles_current <= 0;
    end else begin
      cycles_current <= cycles_current + 1;
    end
  end

  // TODO: copy in your HW5B datapath as a starting point

  /***************/
  /* FETCH STAGE */
  /***************/

  logic [`REG_SIZE] f_pc_current, f_pc_next;
  wire [`REG_SIZE] f_insn;
  cycle_status_e f_cycle_status;
  logic branchOp;

  // program counter
  always_ff @(posedge clk) begin
    if (rst) begin
      f_pc_current <= 32'd0;
      f_cycle_status <= CYCLE_NO_STALL;
    end else begin
      f_pc_current <= f_pc_next;
      f_cycle_status <= CYCLE_NO_STALL;
    end
  end

  // Here's how to disassemble an insn into a string you can view in GtkWave.
  // Use PREFIX to provide a 1-character tag to identify which stage the insn comes from.
  wire [255:0] f_disasm;
  Disasm #(
      .PREFIX("F")
  ) disasm_0fetch (
      .insn  (f_insn),
      .disasm(f_disasm)
  );

  always_comb begin
      // if (branchOp) begin
      //   icache.ARADDR = 0;
      //   icache.ARVALID = False;
      //   icache.RREADY = False;
      // end else 
      
      if (load_stall) begin
      icache.ARADDR = 0;
      icache.ARVALID = False;
      icache.RREADY = False;
      end else begin
        icache.ARADDR = f_pc_current;
        icache.ARVALID = True;
        icache.RREADY = True;
      end
  end


   /****************/
  /* DECODE STAGE */
  /****************/

  // this shows how to package up state in a `struct packed`, and how to pass it between stages
  stage_decode_t decode_state;
  logic [4:0] branch_rd_temp;
  logic BranchStall;
  logic [`INSN_SIZE] insn_from_imem;


  assign insn_from_imem = icache.RDATA;

  always_ff @(posedge clk) begin
    if (rst) begin
      decode_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET
      };
      branch_rd_temp <= 5'b0;
    end else begin
      BranchStall <= branchOp;
      if (!load_stall) begin
        if (branchOp) begin
          decode_state <= '{
            pc: 0,
            insn: 32'b0,
            cycle_status: CYCLE_TAKEN_BRANCH
          };
        end else begin
          decode_state <= '{
            pc: f_pc_current,
            insn: insn_from_imem,
            cycle_status: f_cycle_status
          };
          branch_rd_temp <= 5'b0;
        end
      end
    end
  end

  wire [255:0] d_disasm;
  Disasm #(
      .PREFIX("D")
  ) disasm_1decode (
      .insn  (decode_state.insn),
      .disasm(d_disasm)
  );

    // TODO: your code here, though you will also need to modify some of the code above
  /***************/
  /* EXECUTE STAGE */
  /***************/

  wire [         6:0] insn_funct7;
  wire [         4:0] insn_rs2;
  wire [         4:0] insn_rs1;
  wire [         2:0] insn_funct3;
  wire [         4:0] insn_rd;
  wire [`OPCODE_SIZE] insn_opcode;
  logic div_decOp;
  logic [3:0] div_cycle_count;
  logic [`INSN_SIZE]pending_exec_insn;
  logic exec_insn_pending;

  // split R-type instruction - see section 2.2 of RiscV spec
  assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = insn_from_imem;

  // setup for I, S, B, J & U type instructions
  // I - short immediates and loads
  wire [11:0] imm12_i;
  wire [`REG_SIZE]imm12_isext;
  wire [`REG_SIZE]imm12_izext;

  assign imm12_i =  insn_from_imem[31:20];
  assign imm12_izext = {{20'b0}, imm12_i};
  assign imm12_isext = {{20{imm12_i[11]}}, imm12_i};
  wire [ 4:0] imm_shamt =  insn_from_imem[24:20];

  // S - stores
  wire [11:0] imm12_s;
  assign imm12_s[11:5] = insn_funct7, imm12_s[4:0] = insn_rd;

  // B - conditionals
  wire [12:0] imm_b;
  assign {imm_b[12], imm_b[10:5]} = insn_funct7, {imm_b[4:1], imm_b[11]} = insn_rd, imm_b[0] = 1'b0;

  // J - unconditional jumps
  wire [20:0] imm_j;
  assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {insn_from_imem[31:12], 1'b0};

  // L - loads
  wire [11:0] imm_l;
  assign imm_l = insn_from_imem[31:20];

  wire [`REG_SIZE] imm_i_sext = {{20{imm12_i[11]}}, imm12_i[11:0]};
  wire [`REG_SIZE] imm12_s_sext = {{20{imm12_s[11]}}, imm12_s[11:0]};
  wire [`REG_SIZE] imm_b_sext = {{19{imm_b[12]}}, imm_b[12:0]};
  wire [`REG_SIZE] imm_j_sext = {{11{imm_j[20]}}, imm_j[20:0]};

  // U - upper immediates
  wire [31:0] imm20;
  assign imm20 = {insn_from_imem[31:12], 12'b0};

  logic [`REG_SIZE] rs1_data_dec, rs2_data_dec;

  wire load_stall;
  assign load_stall = (execute_state.opcode == OpLoad && execute_state.rd != 0 &&
    ((insn_rs1 == execute_state.rd) || (insn_rs2 == execute_state.rd && insn_opcode != OpStore)));

  wire fence_stall;
  assign fence_stall = ((insn_opcode == OpMiscMem) && ((execute_state.opcode == OpStore) ||
   (memory_state.opcode == OpStore))) || ((execute_state.opcode == OpMiscMem) && (memory_state.opcode == OpStore));


  always_comb begin
    if (insn_rs1 == writeback_state.rd && writeback_state.rd != 0) begin
      rs1_data_dec = writeback_state.rd_data;
    end else begin
      rs1_data_dec = rs1_data_reg;
    end

    if (insn_rs2 == writeback_state.rd && writeback_state.rd != 0) begin
      rs2_data_dec = writeback_state.rd_data;
    end else begin
      rs2_data_dec = rs2_data_reg;
    end
  end

  wire [4:0] effective_rd;
  assign effective_rd = (insn_opcode == OpBranch) ? 5'b0 : insn_rd;

  stage_execute_t execute_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      execute_state <= '{
        pc: 0,
        insn: 0,
        cycle_status: CYCLE_RESET,
        stall: load_stall,
        rd: 0,
        rs1: 0,
        rs2: 0,
        div_cycle_count: 0,
        rs1_data: 0,
        rs2_data: 0,
        imm12_izext: 0,
        imm12_isext: 0,
        imm_b_sext: 0,
        imm_j_sext:0,
        imm_s_sext:0,
        imm20: 0,
        opcode: 0
      };
    end else begin
         if (branchOp || BranchStall ) begin
           execute_state <= '{
            pc: 0,
            insn: 32'b0,
            cycle_status: CYCLE_TAKEN_BRANCH,
            stall: load_stall,
            rd: 0,
            rs1: 0,
            rs2: 0,
            div_cycle_count: 0,
            rs1_data: 0,  
            rs2_data: 0,
            imm12_izext: 0,
            imm12_isext: 0,
            imm_b_sext: 0,
            imm_j_sext:0,
            imm_s_sext:0,
            imm20: 0,
            opcode: 0
          };
        end else if (fenceOp) begin
            execute_state <= '{
            pc: 0,
            insn: 32'b0,
            cycle_status: CYCLE_FENCEI,
            stall: load_stall,
            rd: 0,
            rs1: 0,
            rs2: 0,
            div_cycle_count: 0,
            rs1_data: 0,  
            rs2_data: 0,
            imm12_izext: 0,
            imm12_isext: 0,
            imm_b_sext: 0,
            imm_j_sext:0,
            imm_s_sext:0,
            imm20: 0,
            opcode: 0
          };
          end else if (load_stall) begin 
                execute_state <= '{
                pc: 0,
                insn: 32'b0,
                cycle_status: CYCLE_LOAD2USE,
                stall: load_stall,
                rd: 0,
                rs1: 0,
                rs2: 0,
                div_cycle_count: 0,
                rs1_data: 0,  
                rs2_data: 0,
                imm12_izext: 0,
                imm12_isext: 0,
                imm_b_sext: 0,
                imm_j_sext:0,
                imm_s_sext:0,
                imm20: 0,
                opcode: 0
              };
              pending_exec_insn <= insn_from_imem;
              exec_insn_pending <= True;
           end else if (!div_stall) begin
                execute_state <= '{
                  pc: decode_state.pc,
                  insn: !load_stall ?insn_from_imem : decode_state.insn,
                  stall: load_stall,
                  rd: effective_rd,
                  rs1: insn_rs1,
                  rs2: insn_rs2,
                  div_cycle_count: div_cycle_count,
                  rs1_data: rs1_data_dec,
                  rs2_data: rs2_data_dec,
                  imm12_izext: imm12_izext,
                  imm12_isext: imm12_isext,
                  imm_b_sext: imm_b_sext,
                  imm_j_sext: imm_j_sext,
                  imm_s_sext: imm12_s_sext,
                  imm20: imm20,
                  opcode: insn_opcode,
                  cycle_status: decode_state.cycle_status
                };
        end
      
        // else begin
        //   execute_state <= '{
        //     pc: decode_state.pc,
        //     insn:  insn_from_imem,
        //     rd: effective_rd,
        //     rs1: insn_rs1,
        //     rs2: insn_rs2,
        //     rs1_data: rs1_data_dec,
        //     rs2_data: rs2_data_dec,
        //     imm12_izext: imm12_izext,
        //     imm12_isext: imm12_isext,
        //     imm_b_sext: imm_b_sext,
        //     imm20: imm20,
        //     opcode: insn_opcode,
        //     cycle_status: decode_state.cycle_status
        //   };
        // //end
      end
    end

  wire [255:0] e_disasm;
  Disasm #(
      .PREFIX("E")
  ) disasm_2execute (
      .insn  (execute_state.insn),
      .disasm(e_disasm)
  );

  wire insn_lui = execute_state.opcode == OpLui;
  wire insn_auipc =  execute_state.opcode == OpAuipc;
  wire insn_jal =  execute_state.opcode == OpJal;
  wire insn_jalr =  execute_state.opcode == OpJalr;

  wire insn_beq = execute_state.opcode == OpBranch && execute_state.insn[14:12] == 3'b000;
  wire insn_bne = execute_state.opcode == OpBranch && execute_state.insn[14:12] == 3'b001;
  wire insn_blt = execute_state.opcode == OpBranch && execute_state.insn[14:12] == 3'b100;
  wire insn_bge = execute_state.opcode == OpBranch && execute_state.insn[14:12] == 3'b101;
  wire insn_bltu = execute_state.opcode == OpBranch && execute_state.insn[14:12] == 3'b110;
  wire insn_bgeu = execute_state.opcode == OpBranch && execute_state.insn[14:12] == 3'b111;

  wire insn_lb = execute_state.opcode == OpLoad && execute_state.insn[14:12] == 3'b000;
  wire insn_lh = execute_state.opcode == OpLoad && execute_state.insn[14:12] == 3'b001;
  wire insn_lw = execute_state.opcode == OpLoad && execute_state.insn[14:12] == 3'b010;
  wire insn_lbu = execute_state.opcode == OpLoad && execute_state.insn[14:12] == 3'b100;
  wire insn_lhu = execute_state.opcode == OpLoad && execute_state.insn[14:12] == 3'b101;

  wire insn_sb = execute_state.opcode == OpStore && execute_state.insn[14:12] == 3'b000;
  wire insn_sh = execute_state.opcode == OpStore && execute_state.insn[14:12] == 3'b001;
  wire insn_sw = execute_state.opcode == OpStore && execute_state.insn[14:12] == 3'b010;

  wire insn_addi = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b000;
  wire insn_slti = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b010;
  wire insn_sltiu = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b011;
  wire insn_xori = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b100;
  wire insn_ori = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b110;
  wire insn_andi = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b111;

  wire insn_slli = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b001 && execute_state.insn[31:25] == 7'd0;
  wire insn_srli = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'd0;
  wire insn_srai = execute_state.opcode == OpRegImm && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'b0100000;

  wire insn_add  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b000 && execute_state.insn[31:25] == 7'd0;
  wire insn_sub  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b000 && execute_state.insn[31:25] == 7'b0100000;
  wire insn_sll  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b001 && execute_state.insn[31:25] == 7'd0;
  wire insn_slt  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b010 && execute_state.insn[31:25] == 7'd0;
  wire insn_sltu = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b011 && execute_state.insn[31:25] == 7'd0;
  wire insn_xor  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b100 && execute_state.insn[31:25] == 7'd0;
  wire insn_srl  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'd0;
  wire insn_sra  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b101 && execute_state.insn[31:25] == 7'b0100000;
  wire insn_or   = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b110 && execute_state.insn[31:25] == 7'd0;
  wire insn_and  = execute_state.opcode == OpRegReg && execute_state.insn[14:12] == 3'b111 && execute_state.insn[31:25] == 7'd0;

  wire insn_mul    = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b000;
  wire insn_mulh   = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b001;
  wire insn_mulhsu = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b010;
  wire insn_mulhu  = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b011;
  wire insn_div    = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b100;
  wire insn_divu   = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b101;
  wire insn_rem    = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b110;
  wire insn_remu   = execute_state.opcode == OpRegReg && execute_state.insn[31:25] == 7'd1 && execute_state.insn[14:12] == 3'b111;

  wire insn_ecall = execute_state.opcode == OpEnviron && insn_from_imem[31:7] == 25'd0;
  wire insn_fence = execute_state.opcode == OpMiscMem;

  logic [`REG_SIZE] rs1_data_reg, rs2_data_reg;
  logic [`REG_SIZE] rd_data_sum, rs1_data_sum, rs2_data_sum;
  logic [`REG_SIZE] rs2_data, rs1_data;
  logic we,we_reg;
  logic [4:0] rd, rs1, rs2;
  logic [`REG_SIZE] rd_data,rd_data_reg;
  logic illegal_insn;
  logic [3:0] load_strb_comb;
  logic [`REG_SIZE] cla_a, cla_b, cla_sum;
  logic cla_in, memOp, regOp;
  logic [63:0] mul_result;

  logic [`REG_SIZE] addr_to_dmem_ex;
  logic [`REG_SIZE]store_data_to_dmem_ex;
  logic [3:0] store_we_to_dmem_ex;

  logic [`REG_SIZE] remainder;
  logic [`REG_SIZE] quotient;
  logic [`REG_SIZE] dividend;
  logic [`REG_SIZE] divisor;
  logic [`REG_SIZE] dividend_temp;
  logic [`REG_SIZE] divisor_temp;
  logic [`REG_SIZE] quotient_temp;
  logic [`REG_SIZE] remainder_temp;
  logic [`REG_SIZE] f_pc_next_temp;
  logic [`REG_SIZE] value;
  logic [`REG_SIZE]rd_data_temp;
  logic [31:0] byte_offset;
  logic [1:0] lb;
  logic [`REG_SIZE] branch_pc;

  logic div_sign;
  logic [4:0] imm_i_shift;
  logic halt_temp;
  logic fenceOp;
  logic jalOp;
  logic div_stall;
  
   always_comb begin
    rs1 = insn_rs1;
    rs2 = insn_rs2;
    memOp = 0;
    regOp = 0;
    we = 0;
    illegal_insn = 0;
    mul_result = 0;
    dividend = 0;
    divisor = 0;
    dividend_temp = 0;
    divisor_temp = 0;
    div_sign = 0;
    quotient_temp = 0;
    remainder_temp = 0;
    value = 0;
    f_pc_next_temp = f_pc_current;
    rd_data = 0;
    rd_data_temp = 0;
    cla_in = 1'b0;
    cla_a = 32'b0;
    cla_b = 32'b0;
    branchOp = 0;
    jalOp = 0;
    halt_temp = 0;
    branch_pc = 0;

    addr_to_dmem_ex = 0;
    store_data_to_dmem_ex = 0;
    store_we_to_dmem_ex = 0;
    byte_offset = 0;

    dcache.ARADDR = 0;
    dcache.ARVALID = False;
    dcache.RREADY = False;

    rs1_data = execute_state.rs1_data;
    rs2_data = execute_state.rs2_data;
    rs1_data_sum = execute_state.rs1_data;
    rs2_data_sum = execute_state.rs2_data;

    load_strb_comb = 4'b0;

    div_stall = 0;
    fenceOp = 0;

    if (execute_state.rs1 == memory_state.rd && memory_state.rd != 0) begin
      rs1_data = memory_state.rd_data;
      rs1_data_sum = memory_state.rd_data;
    end else if (execute_state.rs1 == writeback_state.rd && writeback_state.rd != 0) begin
      rs1_data = writeback_state.rd_data;
      rs1_data_sum = writeback_state.rd_data;
    end

    if (execute_state.rs2 == memory_state.rd && memory_state.rd != 0) begin
      rs2_data = memory_state.rd_data;
      rs2_data_sum = memory_state.rd_data;
    end else if (execute_state.rs2 == writeback_state.rd && writeback_state.rd != 0) begin
      rs2_data = writeback_state.rd_data;
      rs2_data_sum = writeback_state.rd_data;
    end

    case (execute_state.opcode)
        OpLui: begin
          rd_data = execute_state.imm20;
          we = 1'b1;
          memOp = 1'b0;
          regOp = 1'b1;
          branchOp = 1'b0;
        end
        OpRegImm: begin
          we = 1'b1;
          memOp = 1'b0;
          regOp = 1'b1;
          if (insn_addi) begin
              cla_a = rs1_data_sum;
              cla_b = execute_state.imm12_isext;
              cla_in = 1'b0;
              rd_data = cla_sum;
          end else if (insn_slti) begin
            rd_data = ($signed(rs1_data) < $signed(execute_state.imm12_isext)) ? 32'h0000_0001 : 32'b0;
          end else if (insn_sltiu) begin
            rd_data = rs1_data < execute_state.imm12_isext ? 32'h0000_0001 : 32'b0;
          end else if (insn_xori) begin
            rd_data = rs1_data ^ execute_state.imm12_isext;
          end else if (insn_ori) begin
            rd_data = rs1_data | execute_state.imm12_isext;
          end else if (insn_andi) begin
            rd_data = rs1_data & execute_state.imm12_isext;
          end else if (insn_slli) begin 
            rd_data = rs1_data << execute_state.imm12_izext[4:0];
          end else if (insn_srli) begin
            rd_data = $unsigned(rs1_data) >> execute_state.imm12_izext[4:0];
          end else if (insn_srai) begin
            rd_data = $signed(rs1_data) >>> execute_state.imm12_izext[4:0];
          end else begin
            illegal_insn = 1'b1;
          end 
        end
        OpRegReg: begin
          we = 1'b1;
          memOp = 1'b0;
          regOp = 1'b1;
          branchOp = 1'b0;
          if (insn_add) begin
            cla_a = rs1_data_sum;
            cla_b = rs2_data_sum;
            cla_in = 1'b0;
            rd_data = cla_sum;
          end else if (insn_sub) begin
            cla_a = rs1_data_sum;
            cla_b = ~(rs2_data_sum) + 1;
            cla_in = 1'b0;
            rd_data = cla_sum;
          end else if (insn_sll) begin
            rd_data = execute_state.rs1_data << execute_state.rs2_data[4:0];
          end else if (insn_slt) begin
            rd_data = ($signed(execute_state.rs1_data) < $signed(execute_state.rs2_data)) ? 32'h0000_0001 : 32'b0;
          end else if (insn_sltu) begin
            rd_data = execute_state.rs1_data < execute_state.rs2_data ? 32'h0000_0001 : 32'b0;
          end else if (insn_xor) begin
            rd_data = execute_state.rs1_data ^ execute_state.rs2_data;
          end else if (insn_srl) begin
            rd_data = execute_state.rs1_data >> execute_state.rs2_data[4:0];
          end else if (insn_sra) begin
            rd_data = execute_state.rs1_data >>> execute_state.rs2_data[4:0];
          end else if (insn_or) begin
            rd_data = execute_state.rs1_data | execute_state.rs2_data;
          end else if (insn_and) begin
            rd_data = execute_state.rs1_data & execute_state.rs2_data;
          end else if (insn_mul) begin
            mul_result = execute_state.rs1_data * execute_state.rs2_data;
            rd_data = mul_result[31:0];
          end else if (insn_mulh) begin
            mul_result = $signed(execute_state.rs1_data) * $signed(execute_state.rs2_data);
            rd_data = mul_result[63:32];
          end else if (insn_mulhsu) begin
            mul_result = {{32{execute_state.rs1_data[31]}},execute_state.rs1_data[31]} * {32'b0, execute_state.rs2_data};
            rd_data = mul_result[63:32];
          end else if (insn_mulhu) begin
            mul_result = $unsigned(execute_state.rs1_data) * $unsigned(execute_state.rs2_data);
            rd_data = mul_result[63:32];
          end else if (insn_div) begin
            // div_sign = execute_state.rs1_data[31] ^ execute_state.rs2_data[31];
            // dividend_temp = $signed(execute_state.rs1_data) < 0 ? ~execute_state.rs1_data + 1 : execute_state.rs1_data;
            // divisor_temp = $signed(execute_state.rs2_data) < 0 ? ~execute_state.rs2_data + 1 : execute_state.rs2_data;
            // divisor = divisor_temp;
            // dividend = dividend_temp;
            // quotient_temp = quotient;
            // value = div_sign == 0 ? quotient_temp : ~quotient_temp + 1;
            // rd_data = execute_state.rs2_data == 0 ? 32'h8000_0000 : value;
          end else if (insn_divu) begin
            // dividend = execute_state.rs1_data;
            // divisor = execute_state.rs2_data;
            // rd_data = quotient;
          end else if (insn_rem) begin
            // div_sign = execute_state.rs1_data[31];
            // dividend_temp = $signed(execute_state.rs1_data) < 0 ? ~execute_state.rs1_data + 1 : execute_state.rs1_data;
            // divisor_temp = $signed(execute_state.rs2_data) < 0 ? ~execute_state.rs2_data + 1 : execute_state.rs2_data;
            // divisor = divisor_temp;
            // dividend = dividend_temp;
            // remainder_temp = remainder;
            // value = div_sign == 0 ? remainder_temp : ~remainder_temp + 1;
            // rd_data = execute_state.rs2_data == 0 ? execute_state.rs1_data : value;
          end else if (insn_remu) begin
            // dividend = execute_state.rs1_data;
            // divisor = execute_state.rs2_data;
            // rd_data = remainder;
          end else begin
            illegal_insn = 1'b1;
          end
        end
         OpBranch: begin
          if (insn_beq) begin
            if (rs1_data == rs2_data) begin
                f_pc_next_temp = execute_state.pc + execute_state.imm_b_sext;
                branch_pc = execute_state.pc + execute_state.imm_b_sext;
                branchOp = 1'b1;
            end
          end else if (insn_bne) begin
            if (rs1_data != rs2_data) begin
              f_pc_next_temp = execute_state.pc + execute_state.imm_b_sext;
              branch_pc = execute_state.pc + execute_state.imm_b_sext;
              branchOp = 1'b1;
            end
          end else if (insn_blt) begin
            if ($signed(rs1_data) < $signed(rs2_data)) begin
              f_pc_next_temp = execute_state.pc + execute_state.imm_b_sext;
              branchOp = 1'b1;
            end
          end else if (insn_bge) begin
            if ($signed(rs1_data) >= $signed(rs2_data)) begin
              f_pc_next_temp = execute_state.pc + execute_state.imm_b_sext;
              branchOp = 1'b1;
            end
          end else if (insn_bltu) begin
            if ($unsigned(rs1_data) < $unsigned(rs2_data)) begin
              f_pc_next_temp = execute_state.pc + execute_state.imm_b_sext;
              branchOp = 1'b1;
            end
          end else if (insn_bgeu) begin
            if ($unsigned(rs1_data) >= $unsigned(rs2_data)) begin
              f_pc_next_temp = execute_state.pc + execute_state.imm_b_sext;
              branchOp = 1'b1;
            end
          end else begin
            illegal_insn = 1'b1;
          end
        end
        OpEnviron: begin
          if (insn_ecall) begin
            rd_data = 32'h0000_005d;
            we = 1'b1;
            halt_temp = 1'b1;
          end else if (insn_fence) begin
            illegal_insn = 1'b1;
          end else begin
            illegal_insn = 1'b1;
          end
        end
        OpLoad: begin
          we = 1'b1;
          memOp = 1'b1;
          regOp = 1'b1;
          branchOp = 1'b0;

          if (insn_lw) begin
            dcache.ARADDR = (rs1_data + execute_state.imm12_isext) & 32'hFFFFFFFC;
            dcache.ARVALID = True;
            dcache.RREADY = True;
            load_strb_comb = 4'b1111;
          end
        end
         OpJal: begin 
          if (insn_jal) begin 
            rd_data = execute_state.pc + 4;
            f_pc_next_temp = execute_state.pc + execute_state.imm_j_sext;
            we = 1'b1;
            jalOp = 1;
            regOp = 1'b1;
          end else begin 
              illegal_insn = 1'b1;
          end
        end
        OpJalr: begin
          if (insn_jalr) begin
            rd_data = execute_state.pc + 4;
            we = 1'b1;
            regOp = 1'b1;
            f_pc_next_temp = (rs1_data + execute_state.imm12_isext) & 32'hFFFFFFE;
            jalOp = 1;
          end else begin
            illegal_insn = 1'b1;
          end
        end
        OpStore:
          if (insn_sw) begin
            addr_to_dmem_ex =(rs1_data + execute_state.imm_s_sext) & 32'hFFFFFFFC;
            store_data_to_dmem_ex = rs2_data;
            store_we_to_dmem_ex = 4'b1111;
            memOp = 1'b1;
            regOp = 1'b0;
          end else if (insn_sh) begin
            byte_offset = (rs1_data + execute_state.imm_s_sext) & 32'h3;
            lb = byte_offset[1:0];
            addr_to_dmem_ex = (rs1_data + execute_state.imm_s_sext) & 32'hFFFFFFFC;
            case (lb[1])
              1'b0: begin
                store_data_to_dmem_ex = {16'b0,rs2_data[15:0]};
                store_we_to_dmem_ex = 4'b0011;
              end
              1'b1: begin
                store_data_to_dmem_ex = {rs2_data[15:0], 16'b0};
                store_we_to_dmem_ex = 4'b1100;
              end
            endcase
          end else if (insn_sb) begin
            byte_offset = (rs1_data + execute_state.imm_s_sext) & 32'h3;
            lb = byte_offset[1:0];
            addr_to_dmem_ex = (rs1_data + execute_state.imm_s_sext) & 32'hFFFFFFFC;
            case (lb)
              2'b00: begin
                store_data_to_dmem_ex = {24'b0,rs2_data[7:0]};
                store_we_to_dmem_ex = 4'b0001;
              end
              2'b01: begin
                store_data_to_dmem_ex = {16'b0,rs2_data[7:0], 8'b0};
                store_we_to_dmem_ex = 4'b0010;
              end
              2'b10: begin
                store_data_to_dmem_ex = {8'b0,rs2_data[7:0], 16'b0};
                store_we_to_dmem_ex = 4'b0100;
              end
              2'b11:begin
                store_data_to_dmem_ex = {rs2_data[7:0], 24'b0};
                store_we_to_dmem_ex = 4'b1000;
              end
            endcase
          end
        default: begin
          rd_data = 0;
          we = 1'b0;
          memOp = 1'b0;
          regOp = 1'b0;
          illegal_insn = 1'b1;
        end
    endcase
    f_pc_next = (branchOp || load_stall)? f_pc_next_temp : f_pc_current + 4;
  end

  stage_memory_t memory_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      memory_state <= '{
         pc: 0,
        we: 0,
        insn: 0,
        memOp: 0,
        regOp: 0,
        rd: 0,
        rs2:0,
        rd_data: 0,
        halt: 0,
        mx_bypass:0,
        imm12_izext: 0,
        imm12_isext: 0,
        imm20: 0,
        opcode: 0,
        addr_to_dmem:0,
        store_data_to_dmem:0,
        store_we_to_dmem:0,
        cycle_status: CYCLE_RESET
      };
    end else begin     
           if (fenceOp) begin 
        memory_state <= '{
        pc: 0,
        we: 0,
        insn: 0,
        memOp: 0,
        regOp: 0,
        rd: 0,
        rs2:0,
        rd_data: 0,
        halt: 0,
        mx_bypass:0,
        imm12_izext: 0,
        imm12_isext: 0,
        imm20: 0,
        opcode: 0,
        addr_to_dmem:0,
        store_data_to_dmem:0,
        store_we_to_dmem:0,
        cycle_status: CYCLE_FENCEI
      };
      end else begin
        if (!div_stall) begin
            memory_state <= '{
              pc: execute_state.pc,
              insn: execute_state.insn,
              we: we,
              memOp: memOp,
              regOp: regOp,
              rd: execute_state.rd,
              rs2: execute_state.rs2,
              rd_data: rd_data,
              mx_bypass: 0,
              imm12_izext: execute_state.imm12_izext,
              imm12_isext: execute_state.imm12_isext,
              imm20: execute_state.imm20,
              opcode: execute_state.opcode,
              //TODO: REMOVE THESE REGISTERS
              store_data_to_dmem: store_data_to_dmem_ex,
              addr_to_dmem: addr_to_dmem_ex,
              store_we_to_dmem: store_we_to_dmem_ex,
              halt: halt_temp,
              cycle_status: execute_state.cycle_status
            };
        end
      end   
      end
  end

  wire [255:0] m_disasm;
  Disasm #(
      .PREFIX("M")
  ) disasm_3memory (
      .insn  (memory_state.insn),
      .disasm(m_disasm)
  );

  logic [`REG_SIZE] rd_data_mem;
  logic [`REG_SIZE] awaddr;
  logic awvalid, wvalid;
  logic [`REG_SIZE] wdata;
  logic [3:0] wstrb;
  logic bready;

  always_comb begin
    rd_data_mem = memory_state.rd_data;
    awaddr = 0;
    awvalid = 0;
    wdata = 0;
    wstrb = 0;
    bready = 0;
    wvalid = 0;

    if (memory_state.opcode == OpLoad) begin //&& dcache.RVALID
        // TODO: Ffix for lb, hf and caheck if data is valid
        rd_data_mem = dcache.RDATA;
    end

    if (memory_state.opcode == OpStore) begin
        rd_data_mem = memory_state.store_data_to_dmem;
        if (memory_state.opcode == OpStore && memory_state.rs2 == writeback_state.rd
         && writeback_state.rd != 0) begin
            rd_data_mem = writeback_state.rd_data;
        end 
        // dcache.AWADDR =memory_state.addr_to_dmem;
        // dcache.AWVALID = True;
        // dcache.WDATA = 32'h5d5d_5d5d;
        // dcache.WSTRB = memory_state.store_we_to_dmem;
        // dcache.WVALID = True;
        // dcache.BREADY = True;
        awaddr = memory_state.addr_to_dmem;
        awvalid = True;
        wdata = rd_data_mem;
        wstrb = memory_state.store_we_to_dmem;
        bready = True;
        wvalid = True;
    end

    // if (memory_state.opcode == OpStore && memory_state.rs2 == writeback_state.rd && writeback_state.rd != 0) begin
    //     store_data_to_dmem = writeback_state.rd_data;
    // end 
  end 

  stage_writeback_t writeback_state;
  always_ff @(posedge clk) begin
    if (rst) begin
      writeback_state <= '{
        pc: 0,
        insn: 0,
        rd: 0,
        we: 0,
        memOp: 0,
        regOp: 0,
        rd_data: 0,
        halt: 0,
        cycle_status: CYCLE_RESET
      };
    end else begin
      begin
        writeback_state <= '{
          pc: memory_state.pc,
          insn: memory_state.insn,
          rd: memory_state.rd,
          we: memory_state.we,
          memOp: memory_state.memOp,
          regOp: memory_state.regOp,
          rd_data: rd_data_mem,
          halt: memory_state.halt,
          cycle_status: memory_state.cycle_status
        };
      end

        dcache.AWADDR <= awaddr;
        dcache.AWVALID <= awvalid;
        dcache.WDATA <= wdata;
        dcache.WSTRB <= wstrb;
        dcache.BREADY <= bready;
        dcache.WVALID <= wvalid;
        // dcache.AWVALID = True;
        // dcache.WDATA = 32'h5d5d_5d5d;
        // dcache.WSTRB = memory_state.store_we_to_dmem;
        // dcache.WVALID = True;
        // dcache.BREADY = True;
    end
  end

  always_comb begin
      trace_writeback_pc = writeback_state.pc;
      trace_writeback_insn = writeback_state.insn;
      trace_writeback_cycle_status = writeback_state.cycle_status;
      halt = writeback_state.halt;
    if (writeback_state.regOp && writeback_state.we) begin
        rd = writeback_state.rd;
        rd_data_reg = writeback_state.rd_data;
        we_reg = writeback_state.we;
    end else if (writeback_state.halt) begin
        rd = 17;
        rd_data_reg = 32'h0000_005d;
        we_reg = we;
    end else begin
        rd = 0;
        rd_data_reg = 0;
        we_reg = 0;
    end
    end

  wire [255:0] w_disasm;
  Disasm #(
      .PREFIX("W")
  ) disasm_4writeback (
      .insn  (writeback_state.insn),
      .disasm(w_disasm)
  );

  // TODO: the testbench requires that your register file instance is named `rf`
  // /************************
  // REGISTER FILE INSTANCE
  // *************************/
   RegFile rf (
    .clk(clk),
    .rst(rst),
    .we(we_reg),
    .rd(rd),
    .rd_data(rd_data_reg),
    .rs1(rs1),
    .rs2(rs2),
    .rs1_data(rs1_data_reg),
    .rs2_data(rs2_data_reg));

  /************************
   CARRY LOOKAHEAD INSTANCE
  *************************/
  cla cla (
    .a(cla_a),
    .b(cla_b),
    .cin(cla_in),
    .sum(cla_sum));

  //  /************************
  //  DIVIDER UNSIGNED INSTANCE
  // *************************/
  DividerUnsignedPipelined divider_inst (
    .clk(clk),
    .rst(rst),
    .stall(1'b0),
    .i_dividend(dividend),
    .i_divisor(divisor),
    .o_quotient(quotient),
    .o_remainder(remainder)
  );
endmodule // DatapathPipelinedCache

module Processor (
    input wire                       clk,
    input wire                       rst,
    output logic                     halt,
    output wire [`REG_SIZE]          trace_writeback_pc,
    output wire [`INSN_SIZE]         trace_writeback_insn,
    output                           cycle_status_e trace_writeback_cycle_status
);

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  axi_if axi_data_cache ();
  axi_if axi_insn_cache ();
  // memory is dual-ported, to connect to both I$ and D$
  axi_if axi_mem_ro ();
  axi_if axi_mem_rw ();

AxilMemory #(.NUM_WORDS(8192)) memory (
  .ACLK(clk),
  .ARESETn(~rst),
  .port_ro(axi_mem_ro.subord),
  .port_rw(axi_mem_rw.subord)
);

`ifdef ENABLE_INSN_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) icache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_insn_cache.subord),
    .mem(axi_mem_ro.manager)
  );
`endif
`ifdef ENABLE_DATA_CACHE
  AxilCache #(
    .BLOCK_SIZE_BITS(32),
    .NUM_SETS(16)) dcache (
    .ACLK(clk),
    .ARESETn(~rst),
    .proc(axi_data_cache.subord),
    .mem(axi_mem_rw.manager)
  );
`endif

  DatapathPipelinedCache datapath (
      .clk(clk),
      .rst(rst),
`ifdef ENABLE_INSN_CACHE
      .icache(axi_insn_cache.manager),
`else
      .icache(axi_mem_ro.manager),
`endif
`ifdef ENABLE_DATA_CACHE
      .dcache(axi_data_cache.manager),
`else
      .dcache(axi_mem_rw.manager),
`endif
      .halt(halt),
      .trace_writeback_pc(trace_writeback_pc),
      .trace_writeback_insn(trace_writeback_insn),
      .trace_writeback_cycle_status(trace_writeback_cycle_status)
  );

endmodule

