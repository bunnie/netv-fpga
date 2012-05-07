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

`timescale 1 ns / 1 ps

module alpha_scale (
		    input wire clk,
		    input wire reset,
		    input wire [2:0] code,
		    input wire [7:0] din,
		    output wire [7:0] dout
		    );

   reg [7:0] 			     coef;
   wire [15:0] 			     multval;
   
	 // should be multiply by 1-alpha_vid value....
	 // setting  
	 // 000        1.0000000
	 // 001        0.1000000
	 // 010        0.1100000
	 // 011        0.1110000
	 // 100        0.1111000
	 // 101        0.1111100
	 // 110        0.1111110
	 // 111        0.1111111

   always @(code) begin
      case(code)
	3'b000: begin
	   coef <= 8'h80;
	end
	3'b001: begin
	   coef <= 8'h40;
	end
	3'b010: begin
	   coef <= 8'h60;
	end
	3'b011: begin
	   coef <= 8'h70;
	end
	3'b100: begin
	   coef <= 8'h78;
	end
	3'b101: begin
	   coef <= 8'h7c;
	end
	3'b110: begin
	   coef <= 8'h7e;
	end
	3'b111: begin
	   coef <= 8'h7f;
	end
      endcase // case (code)
   end // always @ (code)

   assign multval = coef * din;
   
   assign dout[7:0] = multval[14:0] >> 7;
   
endmodule // alpha_scale
