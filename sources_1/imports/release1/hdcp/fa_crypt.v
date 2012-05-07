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

module fa_crypt (
		 input wire key, // key bit
		 input wire ci,  // carry in
		 input wire hd, // hdmi data
		 input wire ld, // lcd data
		 output reg co, // carry out
		 output reg s // sum out
		 );

   always @(key or ci or hd or ld) begin
      case({key,ci,hd,ld})
	4'b0000: begin
	   co = 0;
	   s = 0;
	end
	4'b0001: begin
	   co = 0;
	   s = 1;
	end
	4'b0010: begin
	   co = 0;
	   s = 1;
	end
	4'b0011: begin
	   co = 1;
	   s = 0;
	end
	4'b0100: begin
	   co = 0;
	   s = 1;
	end
	4'b0101: begin
	   co = 1;
	   s = 0;
	end
	4'b0110: begin
	   co = 1;
	   s = 0;
	end
	4'b0111: begin
	   co = 1;
	   s = 1;
	end
	4'b1000: begin
	   co = 0;
	   s = 0;
	end
	4'b1001: begin
	   co = 1;
	   s = 1;
	end
	4'b1010: begin
	   co = 0;
	   s = 1;
	end
	4'b1011: begin
	   co = 0;
	   s = 0;
	end
	4'b1100: begin
	   co = 1;
	   s = 1;
	end
	4'b1101: begin
	   co = 1;
	   s = 0;
	end
	4'b1110: begin
	   co = 0;
	   s = 0;
	end
	4'b1111: begin
	   co = 1;
	   s = 1;
	end
      endcase // case ({key,ci,hd,ld})
   end // always @ (key or ci or hd or ld)
   
endmodule // fa_crypt

`ifdef NONEXISTENT_FOO
K = key
C = Carry In
S = Source Data
D = Output Data

K C S D  C O

0 0 0 0  0 0
0 0 0 1  0 1
0 0 1 0  0 1
0 0 1 1  1 0

0 1 0 0  0 1
0 1 0 1  1 0
0 1 1 0  1 0
0 1 1 1  1 1

1 0 0 0  0 0
1 0 0 1  1 1
1 0 1 0  0 1
1 0 1 1  0 0

1 1 0 0  1 1
1 1 0 1  1 0
1 1 1 0  0 0
1 1 1 1  1 1

Co:MUX = (K ^ S ^ D)#O6# ? Ci : (K ^ S & D)#O5#
`endif
