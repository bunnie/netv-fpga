//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Andrew "bunnie" Huang
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation and/or 
//    other materials provided with the distribution.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
//    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
//    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
//    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
//    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
//    POSSIBILITY OF SUCH DAMAGE.
//
//////////////////////////////////////////////////////////////////////////////
`timescale 1 ps / 1ps

module encode_terc4 (
  input            clkin,    // pixel clock input
  input            rstin,    // async. reset input (active high)
  input      [3:0] din,      // data inputs: expect registered
  output reg [9:0] dout,      // data outputs
  input            dat_gb
);

   reg [3:0] 	   din_q;    // extra stages to match pipeline delays
   reg [9:0] 	   q_m_reg;  // extra stages to match pipeline delays

   reg             dat_gb_q, dat_gb_reg;

   always @(posedge clkin or posedge rstin) begin
      if(rstin) begin
	 din_q <= 4'b0;
	 dout <= 10'b0;
	 dat_gb_q <= 1'b0;
	 dat_gb_reg <= 1'b0;
      end else begin
	 din_q <= din;
	 dat_gb_q <= dat_gb;
	 dat_gb_reg <= dat_gb_q;

	 if( dat_gb_reg ) begin
	    dout <= 10'b0100110011;
	 end else begin
	    dout <= q_m_reg;
	 end
      end // else: !if(rstin)
   end // always @ (posedge clkin or posedge rstin)
   
   always @(posedge clkin or posedge rstin) begin
      if( rstin ) begin
	 q_m_reg[9:0] <= 10'h0;
      end else begin
      case ({din_q[3], din_q[2], din_q[1], din_q[0]})
	4'b0000: q_m_reg[9:0] <= 10'b1010011100;
	4'b0001: q_m_reg[9:0] <= 10'b1001100011;
	4'b0010: q_m_reg[9:0] <= 10'b1011100100;
	4'b0011: q_m_reg[9:0] <= 10'b1011100010;
	4'b0100: q_m_reg[9:0] <= 10'b0101110001;
	4'b0101: q_m_reg[9:0] <= 10'b0100011110;
	4'b0110: q_m_reg[9:0] <= 10'b0110001110;
	4'b0111: q_m_reg[9:0] <= 10'b0100111100;
	4'b1000: q_m_reg[9:0] <= 10'b1011001100;
	4'b1001: q_m_reg[9:0] <= 10'b0100111001;
	4'b1010: q_m_reg[9:0] <= 10'b0110011100;
	4'b1011: q_m_reg[9:0] <= 10'b1011000110;
	4'b1100: q_m_reg[9:0] <= 10'b1010001110;
	4'b1101: q_m_reg[9:0] <= 10'b1001110001;
	4'b1110: q_m_reg[9:0] <= 10'b0101100011;
	4'b1111: q_m_reg[9:0] <= 10'b1011000011;
	// no default since all cases are covered in this ROM.
      endcase // case ({din_q[3], din_q[2], din_q[1], din_q[0]})
      end // else: !if( rstin )
   end // always @ (posedge clkin or posedge rstin)

endmodule // encode_terc4
