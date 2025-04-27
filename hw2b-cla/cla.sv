`timescale 1ns / 1ps

/**
 * @param a first 1-bit input
 * @param b second 1-bit input
 * @param g whether a and b generate a carry
 * @param p whether a and b would propagate an incoming carry
 */
module gp1(input wire a, b,
           output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

/**
 * Computes aggregate generate/propagate signals over a 4-bit window.
 * @param gin incoming generate signals
 * @param pin incoming propagate signals
 * @param cin the incoming carry
 * @param gout whether these 4 bits internally would generate a carry-out (independent of cin)
 * @param pout whether these 4 bits internally would propagate an incoming carry from cin
 * @param cout the carry outs for the low-order 3 bits
 */
module gp4(input wire [3:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [2:0] cout);

   // TODO: your code here
   assign cout[0] = gin[0] | (pin[0] & cin);
   assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
   assign cout[2] = gin[2] | (pin[2] & gin[1])  | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);

   assign gout = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]);
   assign pout = (& pin);
endmodule

/** Same as gp4 but for an 8-bit window instead */
module gp8(input wire [7:0] gin, pin,
           input wire cin,
           output wire gout, pout,
           output wire [6:0] cout);

   // TODO: your code here
   wire gout_tmp, pout_tmp;
   wire [2:0] cout_lower;


   gp4 gp4_lower(
      .gin(gin[3:0]),
      .pin(pin[3:0]),
      .cin(cin),
      .gout(gout_tmp),
      .pout(pout_tmp),
      .cout(cout_lower)
   );

  assign cout[3] = gout_tmp | (pout_tmp & cin);

   gp4 gp4_upper(
      .gin(gin[7:4]),
      .pin(pin[7:4]),
      .cin(cout[3]),
      .gout(gout),
      .pout(pout),
      .cout(cout[6:4])
   );

   assign cout[2:0] = cout_lower; 


endmodule


module cla(input wire [31:0]  a, b,
   input wire         cin,
   output wire [31:0] sum);

   // TODO: your code here

   //generate gins and pins
   wire [31:0] gin, pin;
   
   genvar i;
   generate
   for (i = 0; i < 32; i++) begin: gins_and_pins_list
      assign gin[i] = a[i] & b[i];
      assign pin[i] = a[i] | b[i];
   end
   endgenerate

   wire [3:0] [6:0] couts; 
   wire [3:0] gout, pout;
   wire c8,c16,c24,c32; 

   gp8 gp8_adder_one(
         .gin(gin[7:0]),
         .pin(pin[7:0]),
         .cin(cin),
         .gout(gout[0]),
         .pout(pout[0]),
         .cout(couts[0])
   );

   gp8 gp8_adder_two(
         .gin(gin[15:8]),
         .pin(pin[15:8]),
         .cin(c8),
         .gout(gout[1]),
         .pout(pout[1]),
         .cout(couts[1])
   );

   gp8 gp8_adder_three(
         .gin(gin[23:16]),
         .pin(pin[23:16]),
         .cin(c16),
         .gout(gout[2]),
         .pout(pout[2]),
         .cout(couts[2])
   );
 
   gp8 gp8_adder_four(
         .gin(gin[31:24]),
         .pin(pin[31:24]),
         .cin(c24),
         .gout(gout[3]),
         .pout(pout[3]),
         .cout(couts[3])
   );

   assign c8  = gout[0] | (pout[0] & couts[0][6]);
   assign c16 = gout[1] | (pout[1] & couts[1][6]);
   assign c24 = gout[2] | (pout[2] & couts[2][6]);
   assign c32 = gout[3] | (pout[3] & couts[3][6]);

   assign sum[7:0]   =  a[7:0] ^ b[7:0] ^ {couts[0], cin};
   assign sum[15:8]  =  a[15:8] ^ b[15:8] ^ {couts[1], c8};  
   assign sum[23:16] =  a[23:16] ^ b[23:16] ^ {couts[2], c16};  
   assign sum[31:24] =  a[31:24] ^ b[31:24] ^ {couts[3], c24};

endmodule

