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

module decode_terc4 (
  input            clkin,    // pixel clock input
  input            rstin,    // async. reset input (active high)
  input      [9:0] din,      // data inputs: expect registered
  output reg [3:0] dout      // data outputs
);
   reg [9:0] 	   din_r;
   
   always @(posedge clkin or posedge rstin) begin
      if( rstin ) begin
	 dout[3:0] <= 4'h0;
	 din_r <= 10'b0;
      end else begin
	 din_r <= din;
	 case (din_r[9:0])
	   10'b1010011100: dout[3:0] <= 4'b0000;
	   10'b1001100011: dout[3:0] <= 4'b0001;
	   10'b1011100100: dout[3:0] <= 4'b0010;
	   10'b1011100010: dout[3:0] <= 4'b0011;
	   10'b0101110001: dout[3:0] <= 4'b0100;
	   10'b0100011110: dout[3:0] <= 4'b0101;
	   10'b0110001110: dout[3:0] <= 4'b0110;
	   10'b0100111100: dout[3:0] <= 4'b0111;
	   10'b1011001100: dout[3:0] <= 4'b1000;
	   10'b0100111001: dout[3:0] <= 4'b1001;
	   10'b0110011100: dout[3:0] <= 4'b1010;
	   10'b1011000110: dout[3:0] <= 4'b1011;
	   10'b1010001110: dout[3:0] <= 4'b1100;
	   10'b1001110001: dout[3:0] <= 4'b1101;
	   10'b0101100011: dout[3:0] <= 4'b1110;
	   10'b1011000011: dout[3:0] <= 4'b1111;
	   // no default to allow for maximum coding flexibility...
	 endcase // case (din_q)
      end // else: !if( rstin )
   end // always @ (posedge clkin or posedge rstin)

endmodule // decode_terc4

