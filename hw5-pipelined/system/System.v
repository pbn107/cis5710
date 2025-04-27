module MyClockGen (
	input_clk_25MHz,
	clk_proc,
	locked
);
	input input_clk_25MHz;
	output wire clk_proc;
	output wire locked;
	wire clkfb;
	(* FREQUENCY_PIN_CLKI = "25" *) (* FREQUENCY_PIN_CLKOP = "20" *) (* ICP_CURRENT = "12" *) (* LPF_RESISTOR = "8" *) (* MFG_ENABLE_FILTEROPAMP = "1" *) (* MFG_GMCREF_SEL = "2" *) EHXPLLL #(
		.PLLRST_ENA("DISABLED"),
		.INTFB_WAKE("DISABLED"),
		.STDBY_ENABLE("DISABLED"),
		.DPHASE_SOURCE("DISABLED"),
		.OUTDIVIDER_MUXA("DIVA"),
		.OUTDIVIDER_MUXB("DIVB"),
		.OUTDIVIDER_MUXC("DIVC"),
		.OUTDIVIDER_MUXD("DIVD"),
		.CLKI_DIV(5),
		.CLKOP_ENABLE("ENABLED"),
		.CLKOP_DIV(30),
		.CLKOP_CPHASE(15),
		.CLKOP_FPHASE(0),
		.FEEDBK_PATH("INT_OP"),
		.CLKFB_DIV(4)
	) pll_i(
		.RST(1'b0),
		.STDBY(1'b0),
		.CLKI(input_clk_25MHz),
		.CLKOP(clk_proc),
		.CLKFB(clkfb),
		.CLKINTFB(clkfb),
		.PHASESEL0(1'b0),
		.PHASESEL1(1'b0),
		.PHASEDIR(1'b1),
		.PHASESTEP(1'b1),
		.PHASELOADREG(1'b1),
		.PLLWAKESYNC(1'b0),
		.ENCLKOP(1'b0),
		.LOCK(locked)
	);
endmodule
module cla (
	a,
	b,
	cin,
	sum
);
	input wire [31:0] a;
	input wire [31:0] b;
	input wire cin;
	output wire [31:0] sum;
endmodule
module DividerUnsignedPipelined (
	clk,
	rst,
	stall,
	i_dividend,
	i_divisor,
	o_remainder,
	o_quotient
);
	input wire clk;
	input wire rst;
	input wire stall;
	input wire [31:0] i_dividend;
	input wire [31:0] i_divisor;
	output wire [31:0] o_remainder;
	output wire [31:0] o_quotient;
endmodule
module Disasm (
	insn,
	disasm
);
	parameter signed [7:0] PREFIX = "D";
	input wire [31:0] insn;
	output wire [255:0] disasm;
endmodule
module RegFile (
	rd,
	rd_data,
	rs1,
	rs1_data,
	rs2,
	rs2_data,
	clk,
	we,
	rst
);
	input wire [4:0] rd;
	input wire [31:0] rd_data;
	input wire [4:0] rs1;
	output wire [31:0] rs1_data;
	input wire [4:0] rs2;
	output wire [31:0] rs2_data;
	input wire clk;
	input wire we;
	input wire rst;
	localparam signed [31:0] NumRegs = 32;
	reg [31:0] regs [0:31];
	wire [32:1] sv2v_tmp_A2C07;
	assign sv2v_tmp_A2C07 = 32'b00000000000000000000000000000000;
	always @(*) regs[0] = sv2v_tmp_A2C07;
	assign rs1_data = regs[rs1];
	assign rs2_data = regs[rs2];
	always @(posedge clk)
		if (rst) begin : sv2v_autoblock_1
			reg signed [31:0] j;
			for (j = 0; j < NumRegs; j = j + 1)
				regs[j] <= 32'b00000000000000000000000000000000;
		end
		else if (we && (rd != 0))
			regs[rd] <= rd_data;
endmodule
module DatapathPipelined (
	clk,
	rst,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem,
	halt,
	trace_writeback_pc,
	trace_writeback_insn,
	trace_writeback_cycle_status
);
	reg _sv2v_0;
	input wire clk;
	input wire rst;
	output wire [31:0] pc_to_imem;
	input wire [31:0] insn_from_imem;
	output reg [31:0] addr_to_dmem;
	input wire [31:0] load_data_from_dmem;
	output reg [31:0] store_data_to_dmem;
	output reg [3:0] store_we_to_dmem;
	output reg halt;
	output reg [31:0] trace_writeback_pc;
	output reg [31:0] trace_writeback_insn;
	output reg [31:0] trace_writeback_cycle_status;
	localparam [6:0] OpLoad = 7'b0000011;
	localparam [6:0] OpStore = 7'b0100011;
	localparam [6:0] OpBranch = 7'b1100011;
	localparam [6:0] OpJalr = 7'b1100111;
	localparam [6:0] OpMiscMem = 7'b0001111;
	localparam [6:0] OpJal = 7'b1101111;
	localparam [6:0] OpRegImm = 7'b0010011;
	localparam [6:0] OpRegReg = 7'b0110011;
	localparam [6:0] OpEnviron = 7'b1110011;
	localparam [6:0] OpAuipc = 7'b0010111;
	localparam [6:0] OpLui = 7'b0110111;
	reg [31:0] cycles_current;
	always @(posedge clk)
		if (rst)
			cycles_current <= 0;
		else
			cycles_current <= cycles_current + 1;
	reg [31:0] f_pc_current;
	reg [31:0] f_pc_next;
	wire [31:0] f_insn;
	reg [31:0] f_cycle_status;
	reg div_stall;
	wire fence_stall;
	wire load_stall;
	always @(posedge clk)
		if (rst) begin
			f_pc_current <= 32'd0;
			f_cycle_status <= 32'd2;
		end
		else if (load_stall) begin
			f_pc_current <= f_pc_current;
			f_cycle_status <= 32'd2;
		end
		else if (div_stall) begin
			f_pc_current <= f_pc_current;
			f_cycle_status <= 32'd8;
		end
		else if (fence_stall) begin
			f_pc_current <= f_pc_current;
			f_cycle_status <= 32'd64;
		end
		else begin
			f_pc_current <= f_pc_next;
			f_cycle_status <= 32'd2;
		end
	assign pc_to_imem = f_pc_current;
	assign f_insn = insn_from_imem;
	wire [255:0] f_disasm;
	Disasm #(.PREFIX("F")) disasm_0fetch(
		.insn(f_insn),
		.disasm(f_disasm)
	);
	reg [95:0] decode_state;
	reg [4:0] branch_rd_temp;
	reg branchOp;
	reg divOp;
	always @(posedge clk)
		if (rst) begin
			decode_state <= 96'h000000000000000000000001;
			branch_rd_temp <= 5'b00000;
		end
		else if ((!load_stall && !divOp) && !div_stall) begin
			if (branchOp)
				decode_state <= 96'h000000000000000000000004;
			else begin
				decode_state <= {f_pc_current, f_insn, f_cycle_status};
				branch_rd_temp <= 5'b00000;
			end
		end
	wire [255:0] d_disasm;
	Disasm #(.PREFIX("D")) disasm_1decode(
		.insn(decode_state[63-:32]),
		.disasm(d_disasm)
	);
	wire [6:0] insn_funct7;
	wire [4:0] insn_rs2;
	wire [4:0] insn_rs1;
	wire [2:0] insn_funct3;
	wire [4:0] insn_rd;
	wire [6:0] insn_opcode;
	reg div_decOp;
	reg [3:0] div_cycle_count;
	assign {insn_funct7, insn_rs2, insn_rs1, insn_funct3, insn_rd, insn_opcode} = decode_state[63-:32];
	wire [11:0] imm12_i;
	wire [31:0] imm12_isext;
	wire [31:0] imm12_izext;
	assign imm12_i = decode_state[63:52];
	assign imm12_izext = {20'b00000000000000000000, imm12_i};
	assign imm12_isext = {{20 {imm12_i[11]}}, imm12_i};
	wire [4:0] imm_shamt = decode_state[56:52];
	reg [1:0] lb;
	wire [11:0] imm12_s;
	assign imm12_s[11:5] = insn_funct7;
	assign imm12_s[4:0] = insn_rd;
	wire [12:0] imm_b;
	assign {imm_b[12], imm_b[10:5]} = insn_funct7;
	assign {imm_b[4:1], imm_b[11]} = insn_rd;
	assign imm_b[0] = 1'b0;
	wire [20:0] imm_j;
	assign {imm_j[20], imm_j[10:1], imm_j[11], imm_j[19:12], imm_j[0]} = {decode_state[63:44], 1'b0};
	wire insn_div_dec = ((insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b100);
	wire insn_divu_dec = ((insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b101);
	wire insn_rem_dec = ((insn_opcode == OpRegReg) && (decode_state[63:57] == 7'd1)) && (decode_state[46:44] == 3'b110);
	wire [11:0] imm_l;
	assign imm_l = decode_state[63:52];
	wire [31:0] imm_i_sext = {{20 {imm12_i[11]}}, imm12_i[11:0]};
	wire [31:0] imm12_s_sext = {{20 {imm12_s[11]}}, imm12_s[11:0]};
	wire [31:0] imm_b_sext = {{19 {imm_b[12]}}, imm_b[12:0]};
	wire [31:0] imm_j_sext = {{11 {imm_j[20]}}, imm_j[20:0]};
	wire [31:0] imm20;
	assign imm20 = {decode_state[63:44], 12'b000000000000};
	reg [31:0] rs1_data_dec;
	reg [31:0] rs2_data_dec;
	reg [378:0] execute_state;
	assign load_stall = ((execute_state[6-:7] == OpLoad) && (execute_state[281-:5] != 0)) && ((insn_rs1 == execute_state[281-:5]) || ((insn_rs2 == execute_state[281-:5]) && (insn_opcode != OpStore)));
	reg [313:0] memory_state;
	assign fence_stall = ((insn_opcode == OpMiscMem) && ((execute_state[6-:7] == OpStore) || (memory_state[38-:7] == OpStore))) || ((execute_state[6-:7] == OpMiscMem) && (memory_state[38-:7] == OpStore));
	wire [31:0] rs1_data_reg;
	wire [31:0] rs2_data_reg;
	reg [136:0] writeback_state;
	always @(*) begin
		if (_sv2v_0)
			;
		if ((insn_rs1 == writeback_state[72-:5]) && (writeback_state[72-:5] != 0))
			rs1_data_dec = writeback_state[63-:32];
		else
			rs1_data_dec = rs1_data_reg;
		if ((insn_rs2 == writeback_state[72-:5]) && (writeback_state[72-:5] != 0))
			rs2_data_dec = writeback_state[63-:32];
		else
			rs2_data_dec = rs2_data_reg;
		div_decOp = (insn_div_dec ? 1 : 0);
		div_cycle_count = (insn_div_dec ? 4'h7 : 4'h0);
	end
	wire [4:0] effective_rd;
	assign effective_rd = (insn_opcode == OpBranch ? 5'b00000 : insn_rd);
	wire fenceOp;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	always @(posedge clk)
		if (rst)
			execute_state <= {96'h000000000000000000000001, load_stall, 282'h0};
		else if (branchOp)
			execute_state <= {96'h000000000000000000000004, load_stall, 282'h0};
		else if (fenceOp)
			execute_state <= {96'h000000000000000000000040, load_stall, 282'h0};
		else if (load_stall)
			execute_state <= {96'h000000000000000000000010, load_stall, 282'h0};
		else if (!div_stall)
			execute_state <= {sv2v_cast_32(decode_state[95-:32]), sv2v_cast_32(decode_state[63-:32]), sv2v_cast_32(decode_state[31-:32]), load_stall, effective_rd, insn_rs1, insn_rs2, div_cycle_count, rs1_data_dec, rs2_data_dec, imm12_izext, imm12_isext, imm20, imm_b_sext, imm_j_sext, imm12_s_sext, insn_opcode};
	wire [255:0] e_disasm;
	Disasm #(.PREFIX("E")) disasm_2execute(
		.insn(execute_state[346-:32]),
		.disasm(e_disasm)
	);
	wire insn_lui = execute_state[6-:7] == OpLui;
	wire insn_auipc = execute_state[6-:7] == OpAuipc;
	wire insn_jal = execute_state[6-:7] == OpJal;
	wire insn_jalr = execute_state[6-:7] == OpJalr;
	wire insn_beq = (execute_state[6-:7] == OpBranch) && (execute_state[329:327] == 3'b000);
	wire insn_bne = (execute_state[6-:7] == OpBranch) && (execute_state[329:327] == 3'b001);
	wire insn_blt = (execute_state[6-:7] == OpBranch) && (execute_state[329:327] == 3'b100);
	wire insn_bge = (execute_state[6-:7] == OpBranch) && (execute_state[329:327] == 3'b101);
	wire insn_bltu = (execute_state[6-:7] == OpBranch) && (execute_state[329:327] == 3'b110);
	wire insn_bgeu = (execute_state[6-:7] == OpBranch) && (execute_state[329:327] == 3'b111);
	wire insn_lb = (execute_state[6-:7] == OpLoad) && (execute_state[329:327] == 3'b000);
	wire insn_lh = (execute_state[6-:7] == OpLoad) && (execute_state[329:327] == 3'b001);
	wire insn_lw = (execute_state[6-:7] == OpLoad) && (execute_state[329:327] == 3'b010);
	wire insn_lbu = (execute_state[6-:7] == OpLoad) && (execute_state[329:327] == 3'b100);
	wire insn_lhu = (execute_state[6-:7] == OpLoad) && (execute_state[329:327] == 3'b101);
	wire insn_sb = (execute_state[6-:7] == OpStore) && (execute_state[329:327] == 3'b000);
	wire insn_sh = (execute_state[6-:7] == OpStore) && (execute_state[329:327] == 3'b001);
	wire insn_sw = (execute_state[6-:7] == OpStore) && (execute_state[329:327] == 3'b010);
	wire insn_addi = (execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b000);
	wire insn_slti = (execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b010);
	wire insn_sltiu = (execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b011);
	wire insn_xori = (execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b100);
	wire insn_ori = (execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b110);
	wire insn_andi = (execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b111);
	wire insn_slli = ((execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b001)) && (execute_state[346:340] == 7'd0);
	wire insn_srli = ((execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b101)) && (execute_state[346:340] == 7'd0);
	wire insn_srai = ((execute_state[6-:7] == OpRegImm) && (execute_state[329:327] == 3'b101)) && (execute_state[346:340] == 7'b0100000);
	wire insn_add = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b000)) && (execute_state[346:340] == 7'd0);
	wire insn_sub = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b000)) && (execute_state[346:340] == 7'b0100000);
	wire insn_sll = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b001)) && (execute_state[346:340] == 7'd0);
	wire insn_slt = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b010)) && (execute_state[346:340] == 7'd0);
	wire insn_sltu = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b011)) && (execute_state[346:340] == 7'd0);
	wire insn_xor = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b100)) && (execute_state[346:340] == 7'd0);
	wire insn_srl = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b101)) && (execute_state[346:340] == 7'd0);
	wire insn_sra = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b101)) && (execute_state[346:340] == 7'b0100000);
	wire insn_or = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b110)) && (execute_state[346:340] == 7'd0);
	wire insn_and = ((execute_state[6-:7] == OpRegReg) && (execute_state[329:327] == 3'b111)) && (execute_state[346:340] == 7'd0);
	wire insn_mul = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b000);
	wire insn_mulh = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b001);
	wire insn_mulhsu = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b010);
	wire insn_mulhu = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b011);
	wire insn_div = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b100);
	wire insn_divu = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b101);
	wire insn_rem = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b110);
	wire insn_remu = ((execute_state[6-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b111);
	wire insn_div_mem = ((memory_state[38-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b100);
	wire insn_divu_mem = ((memory_state[38-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b101);
	wire insn_rem_mem = ((memory_state[38-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b110);
	wire insn_remu_mem = ((memory_state[38-:7] == OpRegReg) && (execute_state[346:340] == 7'd1)) && (execute_state[329:327] == 3'b111);
	wire insn_ecall = (execute_state[6-:7] == OpEnviron) && (insn_from_imem[31:7] == 25'd0);
	wire insn_fence = execute_state[6-:7] == OpMiscMem;
	wire [31:0] rd_data_sum;
	reg [31:0] rs1_data_sum;
	reg [31:0] rs2_data_sum;
	reg [31:0] rs2_data;
	reg [31:0] rs1_data;
	reg we;
	reg we_reg;
	reg [4:0] rd;
	reg [4:0] rs1;
	reg [4:0] rs2;
	reg [31:0] rd_data;
	reg [31:0] rd_data_reg;
	reg illegal_insn;
	reg [31:0] cla_a;
	reg [31:0] cla_b;
	wire [31:0] cla_sum;
	reg cla_in;
	reg memOp;
	reg regOp;
	reg jalOp;
	reg [63:0] mul_result;
	wire [31:0] remainder;
	wire [31:0] quotient;
	reg [31:0] dividend;
	reg [31:0] divisor;
	reg [31:0] dividend_temp;
	reg [31:0] divisor_temp;
	reg [31:0] quotient_temp;
	reg [31:0] remainder_temp;
	reg [31:0] f_pc_next_temp;
	reg [31:0] value;
	reg [31:0] rd_data_temp;
	reg [31:0] addr_to_dmem_ex;
	reg [31:0] store_data_to_dmem_ex;
	reg [3:0] store_we_to_dmem_ex;
	reg div_sign;
	wire [4:0] imm_i_shift;
	reg halt_temp;
	reg [31:0] byte_offset;
	wire insn_on_stall;
	wire div_busy;
	always @(*) begin
		if (_sv2v_0)
			;
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
		f_pc_next_temp = 0;
		rd_data = 0;
		rd_data_temp = 0;
		cla_in = 1'b0;
		cla_a = 32'b00000000000000000000000000000000;
		cla_b = 32'b00000000000000000000000000000000;
		branchOp = 0;
		jalOp = 0;
		byte_offset = 32'b00000000000000000000000000000000;
		lb = 2'b00;
		halt_temp = 0;
		addr_to_dmem_ex = 0;
		store_data_to_dmem_ex = 0;
		store_we_to_dmem_ex = 0;
		rs1_data = execute_state[262-:32];
		rs2_data = execute_state[230-:32];
		rs1_data_sum = execute_state[262-:32];
		rs2_data_sum = execute_state[230-:32];
		divOp = 0;
		div_stall = ((execute_state[266-:4] > 0) && !insn_div_mem ? 1 : 0);
		if ((execute_state[276-:5] == memory_state[249-:5]) && (memory_state[249-:5] != 0)) begin
			rs1_data = memory_state[236-:32];
			rs1_data_sum = memory_state[236-:32];
		end
		else if ((execute_state[276-:5] == writeback_state[72-:5]) && (writeback_state[72-:5] != 0)) begin
			rs1_data = writeback_state[63-:32];
			rs1_data_sum = writeback_state[63-:32];
		end
		if ((execute_state[271-:5] == memory_state[249-:5]) && (memory_state[249-:5] != 0)) begin
			rs2_data = memory_state[236-:32];
			rs2_data_sum = memory_state[236-:32];
		end
		else if ((execute_state[271-:5] == writeback_state[72-:5]) && (writeback_state[72-:5] != 0)) begin
			rs2_data = writeback_state[63-:32];
			rs2_data_sum = writeback_state[63-:32];
		end
		case (execute_state[6-:7])
			OpLui: begin
				rd_data = execute_state[134-:32];
				we = 1'b1;
				memOp = 1'b0;
				regOp = 1'b1;
				branchOp = 1'b0;
			end
			OpAuipc: begin
				rd_data = execute_state[134-:32] + execute_state[378-:32];
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
					cla_b = execute_state[166-:32];
					cla_in = 1'b0;
					rd_data = cla_sum;
				end
				else if (insn_slti)
					rd_data = ($signed(rs1_data) < $signed(execute_state[166-:32]) ? 32'h00000001 : 32'b00000000000000000000000000000000);
				else if (insn_sltiu)
					rd_data = (rs1_data < execute_state[166-:32] ? 32'h00000001 : 32'b00000000000000000000000000000000);
				else if (insn_xori)
					rd_data = rs1_data ^ execute_state[166-:32];
				else if (insn_ori)
					rd_data = rs1_data | execute_state[166-:32];
				else if (insn_andi)
					rd_data = rs1_data & execute_state[166-:32];
				else if (insn_slli)
					rd_data = rs1_data << execute_state[171:167];
				else if (insn_srli)
					rd_data = $unsigned(rs1_data) >> execute_state[171:167];
				else if (insn_srai)
					rd_data = $signed(rs1_data) >>> execute_state[171:167];
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
				end
				else if (insn_sub) begin
					cla_a = rs1_data_sum;
					cla_b = ~rs2_data_sum + 1;
					cla_in = 1'b0;
					rd_data = cla_sum;
				end
				else if (insn_sll)
					rd_data = execute_state[262-:32] << execute_state[203:199];
				else if (insn_slt)
					rd_data = ($signed(execute_state[262-:32]) < $signed(execute_state[230-:32]) ? 32'h00000001 : 32'b00000000000000000000000000000000);
				else if (insn_sltu)
					rd_data = (execute_state[262-:32] < execute_state[230-:32] ? 32'h00000001 : 32'b00000000000000000000000000000000);
				else if (insn_xor)
					rd_data = execute_state[262-:32] ^ execute_state[230-:32];
				else if (insn_srl)
					rd_data = execute_state[262-:32] >> execute_state[203:199];
				else if (insn_sra)
					rd_data = execute_state[262-:32] >>> execute_state[203:199];
				else if (insn_or)
					rd_data = execute_state[262-:32] | execute_state[230-:32];
				else if (insn_and)
					rd_data = execute_state[262-:32] & execute_state[230-:32];
				else if (insn_mul) begin
					mul_result = execute_state[262-:32] * execute_state[230-:32];
					rd_data = mul_result[31:0];
				end
				else if (insn_mulh) begin
					mul_result = $signed(execute_state[262-:32]) * $signed(execute_state[230-:32]);
					rd_data = mul_result[63:32];
				end
				else if (insn_mulhsu) begin
					mul_result = {{32 {execute_state[262]}}, execute_state[262]} * {32'b00000000000000000000000000000000, execute_state[230-:32]};
					rd_data = mul_result[63:32];
				end
				else if (insn_mulhu) begin
					mul_result = $unsigned(execute_state[262-:32]) * $unsigned(execute_state[230-:32]);
					rd_data = mul_result[63:32];
				end
				else if (insn_div) begin
					div_sign = rs1_data[31] ^ rs2_data[31];
					dividend_temp = ($signed(rs1_data) < 0 ? ~rs1_data + 1 : rs1_data);
					divisor_temp = ($signed(rs2_data) < 0 ? ~rs2_data + 1 : rs2_data);
					divisor = divisor_temp;
					dividend = dividend_temp;
					quotient_temp = quotient;
					value = (div_sign == 0 ? quotient_temp : ~quotient_temp + 1);
					rd_data = (rs2_data == 0 ? 32'h80000000 : value);
					if (div_decOp && (execute_state[266-:4] == 8)) begin
						divisor = rs2_data_dec;
						dividend = rs1_data_dec;
					end
				end
				else if (insn_divu) begin
					dividend = execute_state[262-:32];
					divisor = execute_state[230-:32];
					rd_data = quotient;
				end
				else if (insn_rem) begin
					div_sign = execute_state[262];
					dividend_temp = ($signed(rs1_data) < 0 ? ~rs1_data + 1 : rs1_data);
					divisor_temp = ($signed(rs2_data) < 0 ? ~rs2_data + 1 : rs2_data);
					divisor = divisor_temp;
					dividend = dividend_temp;
					remainder_temp = remainder;
					value = (div_sign == 0 ? remainder_temp : ~remainder_temp + 1);
					rd_data = (rs2_data == 0 ? rs1_data : value);
					f_pc_next_temp = execute_state[378-:32];
				end
				else if (insn_remu) begin
					dividend = execute_state[262-:32];
					divisor = execute_state[230-:32];
					rd_data = remainder;
				end
				else
					illegal_insn = 1'b1;
			end
			OpBranch:
				if (insn_beq) begin
					if (rs1_data == rs2_data) begin
						f_pc_next_temp = execute_state[378-:32] + execute_state[102-:32];
						branchOp = 1'b1;
					end
				end
				else if (insn_bne) begin
					if (rs1_data != rs2_data) begin
						f_pc_next_temp = execute_state[378-:32] + execute_state[102-:32];
						branchOp = 1'b1;
					end
				end
				else if (insn_blt) begin
					if ($signed(rs1_data) < $signed(rs2_data)) begin
						f_pc_next_temp = execute_state[378-:32] + execute_state[102-:32];
						branchOp = 1'b1;
					end
				end
				else if (insn_bge) begin
					if ($signed(rs1_data) >= $signed(rs2_data)) begin
						f_pc_next_temp = execute_state[378-:32] + execute_state[102-:32];
						branchOp = 1'b1;
					end
				end
				else if (insn_bltu) begin
					if ($unsigned(rs1_data) < $unsigned(rs2_data)) begin
						f_pc_next_temp = execute_state[378-:32] + execute_state[102-:32];
						branchOp = 1'b1;
					end
				end
				else if (insn_bgeu) begin
					if ($unsigned(rs1_data) >= $unsigned(rs2_data)) begin
						f_pc_next_temp = execute_state[378-:32] + execute_state[102-:32];
						branchOp = 1'b1;
					end
				end
				else
					illegal_insn = 1'b1;
			OpJal:
				if (insn_jal) begin
					rd_data = execute_state[378-:32] + 4;
					f_pc_next_temp = execute_state[378-:32] + execute_state[70-:32];
					we = 1'b1;
					jalOp = 1;
				end
				else
					illegal_insn = 1'b1;
			OpJalr:
				if (insn_jalr) begin
					rd_data = execute_state[378-:32] + 4;
					we = 1'b1;
					f_pc_next_temp = (rs1_data + imm12_isext) & 32'h0ffffffe;
					jalOp = 1;
				end
				else
					illegal_insn = 1'b1;
			OpEnviron: begin
				if (insn_ecall) begin
					rd_data = 32'h0000005d;
					we = 1'b1;
					halt_temp = 1'b1;
				end
				illegal_insn = 1'b1;
			end
			OpMiscMem:
				if (insn_fence)
					;
				else
					illegal_insn = 1'b1;
			default: illegal_insn = 1'b1;
			OpLoad:
				if (insn_lw) begin
					addr_to_dmem_ex = (rs1_data + execute_state[166-:32]) & 32'hfffffffc;
					memOp = 1'b1;
					regOp = 1'b0;
				end
				else
					illegal_insn = 1'b1;
			OpStore:
				if (insn_sw) begin
					addr_to_dmem_ex = (rs1_data + execute_state[38-:32]) & 32'hfffffffc;
					store_data_to_dmem_ex = rs2_data;
					store_we_to_dmem_ex = 4'b1111;
					memOp = 1'b1;
					regOp = 1'b0;
				end
				else if (insn_sb) begin
					byte_offset = (rs1_data + execute_state[38-:32]) & 32'h00000003;
					lb = byte_offset[1:0];
					addr_to_dmem_ex = (rs1_data + execute_state[38-:32]) & 32'hfffffffc;
					case (lb)
						2'b00: begin
							store_data_to_dmem_ex = {24'b000000000000000000000000, rs2_data[7:0]};
							store_we_to_dmem_ex = 4'b0001;
						end
						2'b01: begin
							store_data_to_dmem_ex = {16'b0000000000000000, rs2_data[7:0], 8'b00000000};
							store_we_to_dmem_ex = 4'b0010;
						end
						2'b10: begin
							store_data_to_dmem_ex = {8'b00000000, rs2_data[7:0], 16'b0000000000000000};
							store_we_to_dmem_ex = 4'b0100;
						end
						2'b11: begin
							store_data_to_dmem_ex = {rs2_data[7:0], 24'b000000000000000000000000};
							store_we_to_dmem_ex = 4'b1000;
						end
					endcase
				end
				else if (insn_sh) begin
					byte_offset = (rs1_data + execute_state[38-:32]) & 32'h00000003;
					lb = byte_offset[1:0];
					addr_to_dmem_ex = (rs1_data + execute_state[38-:32]) & 32'hfffffffc;
					case (lb[1])
						1'b0: begin
							store_data_to_dmem_ex = {16'b0000000000000000, rs2_data[15:0]};
							store_we_to_dmem_ex = 4'b0011;
						end
						1'b1: begin
							store_data_to_dmem_ex = {rs2_data[15:0], 16'b0000000000000000};
							store_we_to_dmem_ex = 4'b1100;
						end
					endcase
				end
				else
					illegal_insn = 1'b1;
		endcase
		f_pc_next = (branchOp | jalOp ? f_pc_next_temp : f_pc_current + 4);
	end
	function automatic [4:0] sv2v_cast_5;
		input reg [4:0] inp;
		sv2v_cast_5 = inp;
	endfunction
	function automatic [6:0] sv2v_cast_7;
		input reg [6:0] inp;
		sv2v_cast_7 = inp;
	endfunction
	always @(posedge clk)
		if (rst)
			memory_state <= 314'h1;
		else begin
			if (fenceOp)
				memory_state <= 314'h40;
			else if (!div_stall)
				memory_state <= {sv2v_cast_32(execute_state[378-:32]), sv2v_cast_32(execute_state[346-:32]), sv2v_cast_5(execute_state[281-:5]), sv2v_cast_5(execute_state[271-:5]), we, memOp, regOp, rd_data, 1'd0, sv2v_cast_32(execute_state[198-:32]), sv2v_cast_32(execute_state[166-:32]), addr_to_dmem_ex, store_data_to_dmem_ex, store_we_to_dmem_ex, sv2v_cast_32(execute_state[134-:32]), halt_temp, sv2v_cast_7(execute_state[6-:7]), sv2v_cast_32(execute_state[314-:32])};
			if (div_stall)
				execute_state[266-:4] <= (execute_state[266-:4] > 0 ? execute_state[266-:4] - 1 : 0);
		end
	wire [255:0] m_disasm;
	Disasm #(.PREFIX("M")) disasm_3memory(
		.insn(memory_state[281-:32]),
		.disasm(m_disasm)
	);
	reg [31:0] rd_data_mem;
	always @(*) begin
		if (_sv2v_0)
			;
		addr_to_dmem = memory_state[139-:32];
		store_data_to_dmem = memory_state[107-:32];
		store_we_to_dmem = memory_state[75-:4];
		rd_data_mem = (memory_state[237] ? memory_state[236-:32] : load_data_from_dmem);
		if (((memory_state[38-:7] == OpStore) && (memory_state[244-:5] == writeback_state[72-:5])) && (writeback_state[72-:5] != 0))
			store_data_to_dmem = writeback_state[63-:32];
	end
	always @(posedge clk)
		if (rst)
			writeback_state <= 137'h00000000000000000000000000000000001;
		else if (!div_stall)
			writeback_state <= {sv2v_cast_32(memory_state[313-:32]), sv2v_cast_32(memory_state[281-:32]), sv2v_cast_5(memory_state[249-:5]), memory_state[239], memory_state[238], memory_state[237], memory_state[39], rd_data_mem, sv2v_cast_32(memory_state[31-:32])};
	always @(*) begin
		if (_sv2v_0)
			;
		trace_writeback_pc = writeback_state[136-:32];
		trace_writeback_insn = writeback_state[104-:32];
		trace_writeback_cycle_status = writeback_state[31-:32];
		halt = writeback_state[64];
		if (writeback_state[65] && writeback_state[67]) begin
			rd = writeback_state[72-:5];
			rd_data_reg = writeback_state[63-:32];
			we_reg = writeback_state[67];
		end
		else if (writeback_state[64]) begin
			rd = 17;
			rd_data_reg = 32'h0000005d;
			we_reg = we;
		end
		else begin
			rd = 0;
			rd_data_reg = 0;
			we_reg = 0;
		end
	end
	wire [255:0] w_disasm;
	Disasm #(.PREFIX("W")) disasm_4writeback(
		.insn(writeback_state[104-:32]),
		.disasm(w_disasm)
	);
	RegFile rf(
		.clk(clk),
		.rst(rst),
		.we(we_reg),
		.rd(rd),
		.rd_data(rd_data_reg),
		.rs1(rs1),
		.rs2(rs2),
		.rs1_data(rs1_data_reg),
		.rs2_data(rs2_data_reg)
	);
	cla cla(
		.a(cla_a),
		.b(cla_b),
		.cin(cla_in),
		.sum(cla_sum)
	);
	DividerUnsignedPipelined divider_inst(
		.clk(clk),
		.rst(rst),
		.stall(1'b0),
		.i_dividend(dividend),
		.i_divisor(divisor),
		.o_quotient(quotient),
		.o_remainder(remainder)
	);
	initial _sv2v_0 = 0;
endmodule
module MemorySingleCycle (
	rst,
	clk,
	pc_to_imem,
	insn_from_imem,
	addr_to_dmem,
	load_data_from_dmem,
	store_data_to_dmem,
	store_we_to_dmem
);
	reg _sv2v_0;
	parameter signed [31:0] NUM_WORDS = 512;
	input wire rst;
	input wire clk;
	input wire [31:0] pc_to_imem;
	output reg [31:0] insn_from_imem;
	input wire [31:0] addr_to_dmem;
	output reg [31:0] load_data_from_dmem;
	input wire [31:0] store_data_to_dmem;
	input wire [3:0] store_we_to_dmem;
	reg [31:0] mem_array [0:NUM_WORDS - 1];
	initial $readmemh("mem_initial_contents.hex", mem_array);
	always @(*)
		if (_sv2v_0)
			;
	localparam signed [31:0] AddrMsb = $clog2(NUM_WORDS) + 1;
	localparam signed [31:0] AddrLsb = 2;
	always @(negedge clk)
		if (rst)
			;
		else
			insn_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
	always @(negedge clk)
		if (rst)
			;
		else begin
			if (store_we_to_dmem[0])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
			if (store_we_to_dmem[1])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
			if (store_we_to_dmem[2])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
			if (store_we_to_dmem[3])
				mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
			load_data_from_dmem <= mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];
		end
	initial _sv2v_0 = 0;
endmodule
`default_nettype none
module txuartlite (
	i_clk,
	i_reset,
	i_wr,
	i_data,
	o_uart_tx,
	o_busy
);
	parameter [4:0] TIMING_BITS = 5'd24;
	localparam TB = TIMING_BITS;
	parameter [TB - 1:0] CLOCKS_PER_BAUD = 217;
	input wire i_clk;
	input wire i_reset;
	input wire i_wr;
	input wire [7:0] i_data;
	output reg o_uart_tx;
	output wire o_busy;
	localparam [3:0] TXUL_BIT_ZERO = 4'h0;
	localparam [3:0] TXUL_STOP = 4'h8;
	localparam [3:0] TXUL_IDLE = 4'hf;
	reg [TB - 1:0] baud_counter;
	reg [3:0] state;
	reg [7:0] lcl_data;
	reg r_busy;
	reg zero_baud_counter;
	initial r_busy = 1'b1;
	initial state = TXUL_IDLE;
	always @(posedge i_clk)
		if (i_reset) begin
			r_busy <= 1'b1;
			state <= TXUL_IDLE;
		end
		else if (!zero_baud_counter)
			r_busy <= 1'b1;
		else if (state > TXUL_STOP) begin
			state <= TXUL_IDLE;
			r_busy <= 1'b0;
			if (i_wr && !r_busy) begin
				r_busy <= 1'b1;
				state <= TXUL_BIT_ZERO;
			end
		end
		else begin
			r_busy <= 1'b1;
			if (state <= TXUL_STOP)
				state <= state + 1'b1;
			else
				state <= TXUL_IDLE;
		end
	assign o_busy = r_busy;
	initial lcl_data = 8'hff;
	always @(posedge i_clk)
		if (i_reset)
			lcl_data <= 8'hff;
		else if (i_wr && !r_busy)
			lcl_data <= i_data;
		else if (zero_baud_counter)
			lcl_data <= {1'b1, lcl_data[7:1]};
	initial o_uart_tx = 1'b1;
	always @(posedge i_clk)
		if (i_reset)
			o_uart_tx <= 1'b1;
		else if (i_wr && !r_busy)
			o_uart_tx <= 1'b0;
		else if (zero_baud_counter)
			o_uart_tx <= lcl_data[0];
	initial zero_baud_counter = 1'b1;
	initial baud_counter = 0;
	always @(posedge i_clk)
		if (i_reset) begin
			zero_baud_counter <= 1'b1;
			baud_counter <= 0;
		end
		else begin
			zero_baud_counter <= baud_counter == 1;
			if (state == TXUL_IDLE) begin
				baud_counter <= 0;
				zero_baud_counter <= 1'b1;
				if (i_wr && !r_busy) begin
					baud_counter <= CLOCKS_PER_BAUD - 1'b1;
					zero_baud_counter <= 1'b0;
				end
			end
			else if (!zero_baud_counter)
				baud_counter <= baud_counter - 1'b1;
			else if (state > TXUL_STOP) begin
				baud_counter <= 0;
				zero_baud_counter <= 1'b1;
			end
			else if (state == TXUL_STOP)
				baud_counter <= CLOCKS_PER_BAUD - 2;
			else
				baud_counter <= CLOCKS_PER_BAUD - 1'b1;
		end
endmodule
`default_nettype none
module rxuartlite (
	i_clk,
	i_reset,
	i_uart_rx,
	o_wr,
	o_data
);
	parameter TIMER_BITS = 10;
	parameter [TIMER_BITS - 1:0] CLOCKS_PER_BAUD = 217;
	localparam TB = TIMER_BITS;
	localparam [3:0] RXUL_BIT_ZERO = 4'h0;
	localparam [3:0] RXUL_BIT_ONE = 4'h1;
	localparam [3:0] RXUL_BIT_TWO = 4'h2;
	localparam [3:0] RXUL_BIT_THREE = 4'h3;
	localparam [3:0] RXUL_BIT_FOUR = 4'h4;
	localparam [3:0] RXUL_BIT_FIVE = 4'h5;
	localparam [3:0] RXUL_BIT_SIX = 4'h6;
	localparam [3:0] RXUL_BIT_SEVEN = 4'h7;
	localparam [3:0] RXUL_STOP = 4'h8;
	localparam [3:0] RXUL_WAIT = 4'h9;
	localparam [3:0] RXUL_IDLE = 4'hf;
	input wire i_clk;
	input wire i_reset;
	input wire i_uart_rx;
	output reg o_wr;
	output reg [7:0] o_data;
	wire [TB - 1:0] half_baud;
	reg [3:0] state;
	assign half_baud = {1'b0, CLOCKS_PER_BAUD[TB - 1:1]};
	reg [TB - 1:0] baud_counter;
	reg zero_baud_counter;
	reg q_uart;
	reg qq_uart;
	reg ck_uart;
	reg [TB - 1:0] chg_counter;
	reg half_baud_time;
	reg [7:0] data_reg;
	initial q_uart = 1'b1;
	initial qq_uart = 1'b1;
	initial ck_uart = 1'b1;
	always @(posedge i_clk)
		if (i_reset)
			{ck_uart, qq_uart, q_uart} <= 3'b111;
		else
			{ck_uart, qq_uart, q_uart} <= {qq_uart, q_uart, i_uart_rx};
	initial chg_counter = {TB {1'b1}};
	always @(posedge i_clk)
		if (i_reset)
			chg_counter <= {TB {1'b1}};
		else if (qq_uart != ck_uart)
			chg_counter <= 0;
		else if (chg_counter != {TB {1'b1}})
			chg_counter <= chg_counter + 1;
	initial half_baud_time = 0;
	always @(posedge i_clk)
		if (i_reset)
			half_baud_time <= 0;
		else
			half_baud_time <= !ck_uart && (chg_counter >= (half_baud - (1'b1 + 1'b1)));
	initial state = RXUL_IDLE;
	always @(posedge i_clk)
		if (i_reset)
			state <= RXUL_IDLE;
		else if (state == RXUL_IDLE) begin
			state <= RXUL_IDLE;
			if (!ck_uart && half_baud_time)
				state <= RXUL_BIT_ZERO;
		end
		else if ((state >= RXUL_WAIT) && ck_uart)
			state <= RXUL_IDLE;
		else if (zero_baud_counter) begin
			if (state <= RXUL_STOP)
				state <= state + 1;
		end
	always @(posedge i_clk)
		if (zero_baud_counter && (state != RXUL_STOP))
			data_reg <= {qq_uart, data_reg[7:1]};
	initial o_wr = 1'b0;
	initial o_data = 8'h00;
	always @(posedge i_clk)
		if (i_reset) begin
			o_wr <= 1'b0;
			o_data <= 8'h00;
		end
		else if ((zero_baud_counter && (state == RXUL_STOP)) && ck_uart) begin
			o_wr <= 1'b1;
			o_data <= data_reg;
		end
		else
			o_wr <= 1'b0;
	initial baud_counter = 0;
	always @(posedge i_clk)
		if (i_reset)
			baud_counter <= 0;
		else if (((state == RXUL_IDLE) && !ck_uart) && half_baud_time)
			baud_counter <= CLOCKS_PER_BAUD - 1'b1;
		else if (state == RXUL_WAIT)
			baud_counter <= 0;
		else if (zero_baud_counter && (state < RXUL_STOP))
			baud_counter <= CLOCKS_PER_BAUD - 1'b1;
		else if (!zero_baud_counter)
			baud_counter <= baud_counter - 1'b1;
	initial zero_baud_counter = 1'b1;
	always @(posedge i_clk)
		if (i_reset)
			zero_baud_counter <= 1'b1;
		else if (((state == RXUL_IDLE) && !ck_uart) && half_baud_time)
			zero_baud_counter <= 1'b0;
		else if (state == RXUL_WAIT)
			zero_baud_counter <= 1'b1;
		else if (zero_baud_counter && (state < RXUL_STOP))
			zero_baud_counter <= 1'b0;
		else if (baud_counter == 1)
			zero_baud_counter <= 1'b1;
endmodule
module SystemDemo (
	external_clk_25MHz,
	ftdi_txd,
	btn,
	led,
	ftdi_rxd,
	wifi_gpio0
);
	input external_clk_25MHz;
	input ftdi_txd;
	input [6:0] btn;
	output wire [7:0] led;
	output wire ftdi_rxd;
	output wire wifi_gpio0;
	localparam signed [31:0] MmapOutput = 32'hff001000;
	localparam signed [31:0] MmapInput = 32'hff002000;
	wire clk_proc;
	wire clk_locked;
	MyClockGen clock_gen(
		.input_clk_25MHz(external_clk_25MHz),
		.clk_proc(clk_proc),
		.locked(clk_locked)
	);
	wire [7:0] rx_data;
	wire rx_ready;
	wire [7:0] data2cpu_uart;
	wire [7:0] data2cpu_cpu;
	assign data2cpu_uart = (rx_ready ? rx_data : 8'h00);
	assign led = data2cpu_cpu;
	wire [7:0] tx_data;
	wire tx_ready;
	wire tx_busy;
	wire [7:0] data2uart_cpu;
	wire [7:0] data2uart_uart;
	assign tx_ready = !tx_busy;
	assign tx_data = data2uart_uart;
	rxuartlite uart_receive(
		.i_clk(external_clk_25MHz),
		.i_reset(1'b0),
		.i_uart_rx(ftdi_txd),
		.o_wr(rx_ready),
		.o_data(rx_data)
	);
	wire [31:0] mem_data_addr;
	wire [31:0] mem_data_to_write;
	wire [3:0] mem_data_we;
	DP16KD #(
		.DATA_WIDTH_A(9),
		.DATA_WIDTH_B(9),
		.REGMODE_A("NOREG"),
		.REGMODE_B("NOREG"),
		.RESETMODE("SYNC"),
		.ASYNC_RESET_RELEASE("SYNC"),
		.WRITEMODE_A("NORMAL"),
		.WRITEMODE_B("NORMAL")
	) uart2cpu(
		.ADA13(1'b0),
		.ADA12(1'b0),
		.ADA11(1'b0),
		.ADA10(1'b0),
		.ADA9(1'b0),
		.ADA8(1'b0),
		.ADA7(1'b0),
		.ADA6(1'b0),
		.ADA5(1'b0),
		.ADA4(1'b0),
		.ADA3(1'b0),
		.ADA2(1'b0),
		.ADA1(1'b0),
		.ADA0(1'b0),
		.DIA8(1'b0),
		.DIA7(data2cpu_uart[7]),
		.DIA6(data2cpu_uart[6]),
		.DIA5(data2cpu_uart[5]),
		.DIA4(data2cpu_uart[4]),
		.DIA3(data2cpu_uart[3]),
		.DIA2(data2cpu_uart[2]),
		.DIA1(data2cpu_uart[1]),
		.DIA0(data2cpu_uart[0]),
		.CEA(1'b1),
		.OCEA(1'b1),
		.CLKA(external_clk_25MHz),
		.WEA(rx_ready),
		.RSTA(1'b0),
		.ADB13(1'b0),
		.ADB12(1'b0),
		.ADB11(1'b0),
		.ADB10(1'b0),
		.ADB9(1'b0),
		.ADB8(1'b0),
		.ADB7(1'b0),
		.ADB6(1'b0),
		.ADB5(1'b0),
		.ADB4(1'b0),
		.ADB3(1'b0),
		.ADB2(1'b0),
		.ADB1(1'b0),
		.ADB0(1'b0),
		.DIB8(1'b0),
		.DIB7(mem_data_to_write[7]),
		.DIB6(mem_data_to_write[6]),
		.DIB5(mem_data_to_write[5]),
		.DIB4(mem_data_to_write[4]),
		.DIB3(mem_data_to_write[3]),
		.DIB2(mem_data_to_write[2]),
		.DIB1(mem_data_to_write[1]),
		.DIB0(mem_data_to_write[0]),
		.DOB8(),
		.DOB7(data2cpu_cpu[7]),
		.DOB6(data2cpu_cpu[6]),
		.DOB5(data2cpu_cpu[5]),
		.DOB4(data2cpu_cpu[4]),
		.DOB3(data2cpu_cpu[3]),
		.DOB2(data2cpu_cpu[2]),
		.DOB1(data2cpu_cpu[1]),
		.DOB0(data2cpu_cpu[0]),
		.CEB(1'b1),
		.OCEB(1'b1),
		.CLKB(clk_proc),
		.WEB((mem_data_addr == MmapInput) && |mem_data_we),
		.RSTB(1'b0)
	);
	txuartlite uart_transmit(
		.i_clk(external_clk_25MHz),
		.i_reset(1'b0),
		.i_wr(tx_ready),
		.i_data(tx_data),
		.o_uart_tx(ftdi_rxd),
		.o_busy(tx_busy)
	);
	DP16KD #(
		.DATA_WIDTH_A(9),
		.DATA_WIDTH_B(9),
		.REGMODE_A("NOREG"),
		.REGMODE_B("NOREG"),
		.RESETMODE("SYNC"),
		.ASYNC_RESET_RELEASE("SYNC"),
		.WRITEMODE_A("NORMAL"),
		.WRITEMODE_B("NORMAL")
	) cpu2uart(
		.ADA13(1'b0),
		.ADA12(1'b0),
		.ADA11(1'b0),
		.ADA10(1'b0),
		.ADA9(1'b0),
		.ADA8(1'b0),
		.ADA7(1'b0),
		.ADA6(1'b0),
		.ADA5(1'b0),
		.ADA4(1'b0),
		.ADA3(1'b0),
		.ADA2(1'b0),
		.ADA1(1'b0),
		.ADA0(1'b0),
		.DIA8(1'b0),
		.DIA7(data2uart_cpu[7]),
		.DIA6(data2uart_cpu[6]),
		.DIA5(data2uart_cpu[5]),
		.DIA4(data2uart_cpu[4]),
		.DIA3(data2uart_cpu[3]),
		.DIA2(data2uart_cpu[2]),
		.DIA1(data2uart_cpu[1]),
		.DIA0(data2uart_cpu[0]),
		.CEA(1'b1),
		.OCEA(1'b1),
		.CLKA(clk_proc),
		.WEA((mem_data_addr == MmapOutput) && |mem_data_we),
		.RSTA(1'b0),
		.ADB13(1'b0),
		.ADB12(1'b0),
		.ADB11(1'b0),
		.ADB10(1'b0),
		.ADB9(1'b0),
		.ADB8(1'b0),
		.ADB7(1'b0),
		.ADB6(1'b0),
		.ADB5(1'b0),
		.ADB4(1'b0),
		.ADB3(1'b0),
		.ADB2(1'b0),
		.ADB1(1'b0),
		.ADB0(1'b0),
		.DOB8(),
		.DOB7(data2uart_uart[7]),
		.DOB6(data2uart_uart[6]),
		.DOB5(data2uart_uart[5]),
		.DOB4(data2uart_uart[4]),
		.DOB3(data2uart_uart[3]),
		.DOB2(data2uart_uart[2]),
		.DOB1(data2uart_uart[1]),
		.DOB0(data2uart_uart[0]),
		.CEB(1'b1),
		.OCEB(1'b1),
		.CLKB(external_clk_25MHz),
		.WEB(1'b0),
		.RSTB(1'b0)
	);
	wire [31:0] pc_to_imem;
	wire [31:0] insn_from_imem;
	wire [31:0] mem_data_loaded_value;
	wire [31:0] trace_writeback_pc;
	wire [31:0] trace_writeback_insn;
	wire [31:0] trace_writeback_cycle_status;
	assign data2uart_cpu = mem_data_to_write[7:0];
	MemorySingleCycle #(.NUM_WORDS(1024)) memory(
		.rst(!clk_locked),
		.clk(clk_proc),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.load_data_from_dmem(mem_data_loaded_value),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem((mem_data_addr == MmapOutput ? 4'd0 : mem_data_we))
	);
	DatapathPipelined datapath(
		.clk(clk_proc),
		.rst(!clk_locked),
		.pc_to_imem(pc_to_imem),
		.insn_from_imem(insn_from_imem),
		.addr_to_dmem(mem_data_addr),
		.store_data_to_dmem(mem_data_to_write),
		.store_we_to_dmem(mem_data_we),
		.load_data_from_dmem((mem_data_addr == MmapInput ? {24'd0, data2cpu_cpu} : mem_data_loaded_value)),
		.halt(),
		.trace_writeback_pc(trace_writeback_pc),
		.trace_writeback_insn(trace_writeback_insn),
		.trace_writeback_cycle_status(trace_writeback_cycle_status)
	);
endmodule