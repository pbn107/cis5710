`timescale 1ns / 1ns

// quotient = dividend / divisor
module DividerUnsignedPipelined (
    input wire clk, rst, stall,
    input  wire  [31:0] i_dividend,
    input  wire  [31:0] i_divisor,
    output logic [31:0] o_remainder,
    output logic [31:0] o_quotient
);

  wire [31:0] div_rem[0:32];
  wire [31:0] div_quo[0:32];
  wire [31:0] div_div[0:32];

  assign div_rem[0] = 32'b0;
  assign div_quo[0] = 32'b0;
  assign div_div[0] = i_dividend;

  typedef struct packed {
    logic [31:0] dividend;
    logic [31:0] divisor;
    logic [31:0] remainder;
    logic [31:0] quotient;
  } divu_state_t;

  divu_state_t divu_state[0:6];

  generate
    genvar i;
    for (i = 0; i < 4; i = i + 1) begin
      divu_1iter u_divu_iter (
          .i_dividend (div_div[i]),
          .i_divisor  (i_divisor),
          .i_remainder(div_rem[i]),
          .i_quotient (div_quo[i]),
          .o_dividend (div_div[i+1]),
          .o_remainder(div_rem[i+1]),
          .o_quotient (div_quo[i+1])
      );
    end
  endgenerate

  always_ff @(posedge clk) begin
    if (rst) begin
      divu_state[0] <= '0;
    end else begin
        divu_state[0] <= '{
        dividend: div_div[4],
        divisor: i_divisor,
        remainder: div_rem[4],
        quotient: div_quo[4]
      };
    end
  end

  divu_1iter u_divu_1iter4 (
          .i_dividend (divu_state[0].dividend),
          .i_divisor  (divu_state[0].divisor),
          .i_remainder(divu_state[0].remainder),
          .i_quotient (divu_state[0].quotient),
          .o_dividend (div_div[5]),
          .o_remainder(div_rem[5]),
          .o_quotient (div_quo[5])
      );

  generate
    for (i = 5; i < 8; i = i + 1) begin
      divu_1iter u_divu_1iter1 (
          .i_dividend (div_div[i]),
          .i_divisor  (divu_state[0].divisor),
          .i_remainder(div_rem[i]),
          .i_quotient (div_quo[i]),
          .o_dividend (div_div[i+1]),
          .o_remainder(div_rem[i+1]),
          .o_quotient (div_quo[i+1])
      );
    end
  endgenerate

  always_ff @(posedge clk) begin
    if (rst) begin
      divu_state[1] <= '0;
    end else begin
      divu_state[1] <= '{
        dividend: div_div[8],
        divisor: divu_state[0].divisor,
        remainder: div_rem[8],
        quotient: div_quo[8]
      };
    end
  end

  divu_1iter u_divu_1iter8 (
          .i_dividend (divu_state[1].dividend),
          .i_divisor  (divu_state[1].divisor),
          .i_remainder(divu_state[1].remainder),
          .i_quotient (divu_state[1].quotient),
          .o_dividend (div_div[9]),
          .o_remainder(div_rem[9]),
          .o_quotient (div_quo[9])
      );

  generate
    for (i = 9; i < 12; i = i + 1) begin
      divu_1iter u_divu_1iter1 (
          .i_dividend (div_div[i]),
          .i_divisor  (divu_state[1].divisor),
          .i_remainder(div_rem[i]),
          .i_quotient (div_quo[i]),
          .o_dividend (div_div[i+1]),
          .o_remainder(div_rem[i+1]),
          .o_quotient (div_quo[i+1])
      );
    end
  endgenerate

  always_ff @(posedge clk) begin
    if (rst) begin
      divu_state[2] <= '0;
    end else begin
      divu_state[2] <= '{
        dividend: div_div[12],
        divisor: divu_state[1].divisor,
        remainder: div_rem[12],
        quotient: div_quo[12]
      };
    end
  end

  divu_1iter u_divu_1iter12 (
          .i_dividend (divu_state[2].dividend),
          .i_divisor  (divu_state[2].divisor),
          .i_remainder(divu_state[2].remainder),
          .i_quotient (divu_state[2].quotient),
          .o_dividend (div_div[13]),
          .o_remainder(div_rem[13]),
          .o_quotient (div_quo[13])
      );

generate
    for (i = 13; i < 16; i = i + 1) begin
      divu_1iter u_divu_1iter1 (
          .i_dividend (div_div[i]),
          .i_divisor  (divu_state[2].divisor),
          .i_remainder(div_rem[i]),
          .i_quotient (div_quo[i]),
          .o_dividend (div_div[i+1]),
          .o_remainder(div_rem[i+1]),
          .o_quotient (div_quo[i+1])
      );
    end
  endgenerate

  always_ff @(posedge clk) begin
    if (rst) begin
      divu_state[3] <= '0;
    end else begin
      divu_state[3] <= '{
        dividend: div_div[16],
        divisor: divu_state[2].divisor,
        remainder: div_rem[16],
        quotient: div_quo[16]
      };
    end
  end

divu_1iter u_divu_1iter16 (
          .i_dividend (divu_state[3].dividend),
          .i_divisor  (divu_state[3].divisor),
          .i_remainder(divu_state[3].remainder),
          .i_quotient (divu_state[3].quotient),
          .o_dividend (div_div[17]),
          .o_remainder(div_rem[17]),
          .o_quotient (div_quo[17])
      );

generate
    for (i = 17; i < 20; i = i + 1) begin
    divu_1iter u_divu_1iter1 (
        .i_dividend (div_div[i]),
        .i_divisor  (divu_state[3].divisor),
        .i_remainder(div_rem[i]),
        .i_quotient (div_quo[i]),
        .o_dividend (div_div[i+1]),
        .o_remainder(div_rem[i+1]),
        .o_quotient (div_quo[i+1])
    );
    end
endgenerate

always_ff @(posedge clk) begin
    if (rst) begin
    divu_state[4] <= '0;
    end else begin
    divu_state[4] <= '{
        dividend: div_div[20],
        divisor: divu_state[3].divisor,
        remainder: div_rem[20],
        quotient: div_quo[20]
    };
    end
end


divu_1iter u_divu_1iter20 (
      .i_dividend (divu_state[4].dividend),
      .i_divisor  (divu_state[4].divisor),
      .i_remainder(divu_state[4].remainder),
      .i_quotient (divu_state[4].quotient),
      .o_dividend (div_div[21]),
      .o_remainder(div_rem[21]),
      .o_quotient (div_quo[21])
 );

generate
    for (i = 21; i < 24; i = i + 1) begin
    divu_1iter u_divu_1iter1 (
        .i_dividend (div_div[i]),
        .i_divisor  (divu_state[4].divisor),
        .i_remainder(div_rem[i]),
        .i_quotient (div_quo[i]),
        .o_dividend (div_div[i+1]),
        .o_remainder(div_rem[i+1]),
        .o_quotient (div_quo[i+1])
    );
    end
endgenerate

always_ff @(posedge clk ) begin
    if (rst) begin
    divu_state[5] <= '0;
    end else begin
    divu_state[5] <= '{
        dividend: div_div[24],
        divisor: divu_state[4].divisor,
        remainder: div_rem[24],
        quotient: div_quo[24]
    };
    end
end

divu_1iter u_divu_1iter24 (
          .i_dividend (divu_state[5].dividend),
          .i_divisor  (divu_state[5].divisor),
          .i_remainder(divu_state[5].remainder),
          .i_quotient (divu_state[5].quotient),
          .o_dividend (div_div[25]),
          .o_remainder(div_rem[25]),
          .o_quotient (div_quo[25])
 );

generate
    for (i = 25; i < 28; i = i + 1) begin
    divu_1iter u_divu_1iter1 (
        .i_dividend (div_div[i]),
        .i_divisor  (divu_state[5].divisor),
        .i_remainder(div_rem[i]),
        .i_quotient (div_quo[i]),
        .o_dividend (div_div[i+1]),
        .o_remainder(div_rem[i+1]),
        .o_quotient (div_quo[i+1])
    );
    end
endgenerate

always_ff @(posedge clk) begin
    if (rst) begin
    divu_state[6] <= '0;
    end else begin
    divu_state[6] <= '{
        dividend: div_div[28],
        divisor: divu_state[5].divisor,
        remainder: div_rem[28],
        quotient: div_quo[28]
    };
    end
end

divu_1iter u_divu_1iter28 (
          .i_dividend (divu_state[6].dividend),
          .i_divisor  (divu_state[6].divisor),
          .i_remainder(divu_state[6].remainder),
          .i_quotient (divu_state[6].quotient),
          .o_dividend (div_div[29]),
          .o_remainder(div_rem[29]),
          .o_quotient (div_quo[29])
 );

generate
    for (i = 29; i < 32; i = i + 1) begin
    divu_1iter u_divu_1iter1 (
        .i_dividend (div_div[i]),
        .i_divisor  (divu_state[6].divisor),
        .i_remainder(div_rem[i]),
        .i_quotient (div_quo[i]),
        .o_dividend (div_div[i+1]),
        .o_remainder(div_rem[i+1]),
        .o_quotient (div_quo[i+1])
    );
    end
endgenerate

assign o_remainder = div_rem[32];
assign o_quotient = div_quo[32];

endmodule

module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);
  
  // TODO: your code here 

  wire [31:0] shifted_remainder;
  wire [31:0] new_remainder;
  wire quotient_bit;

  // Shift remainder left and bring in the MSB of dividend
  assign shifted_remainder = {i_remainder[30:0], i_dividend[31]};

  // Check if divisor fits into the remainder
  assign quotient_bit = (shifted_remainder >= i_divisor) ? 1'b1 : 1'b0;
  
  // Compute new remainder
  assign new_remainder = quotient_bit ? (shifted_remainder - i_divisor) : shifted_remainder;

  // Assign outputs
  assign o_quotient = {i_quotient[30:0], quotient_bit};  // Append quotient bit
  assign o_remainder = new_remainder;
  assign o_dividend = i_dividend << 1;  // Shift dividend left
  
endmodule

