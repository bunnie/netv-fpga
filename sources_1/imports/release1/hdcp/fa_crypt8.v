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

module fa_crypt8 (
		  input wire [7:0] key, // key bit
		  input wire [7:0] hd, // hdmi data
		  input wire [7:0] ld, // lcd data
		  output wire [7:0] s, // sum out
		  output wire ovf
		 );

   wire [7:0] 		      carry;
   
   fa_crypt fa0 (.key(key[0]), .ci(1'b0), .hd(hd[0]), .ld(ld[0]), .co(carry[0]), .s(s[0]) );
   fa_crypt fa1 (.key(key[1]), .ci(carry[0]), .hd(hd[1]), .ld(ld[1]), .co(carry[1]), .s(s[1]) );
   fa_crypt fa2 (.key(key[2]), .ci(carry[1]), .hd(hd[2]), .ld(ld[2]), .co(carry[2]), .s(s[2]) );
   fa_crypt fa3 (.key(key[3]), .ci(carry[2]), .hd(hd[3]), .ld(ld[3]), .co(carry[3]), .s(s[3]) );
   fa_crypt fa4 (.key(key[4]), .ci(carry[3]), .hd(hd[4]), .ld(ld[4]), .co(carry[4]), .s(s[4]) );
   fa_crypt fa5 (.key(key[5]), .ci(carry[4]), .hd(hd[5]), .ld(ld[5]), .co(carry[5]), .s(s[5]) );
   fa_crypt fa6 (.key(key[6]), .ci(carry[5]), .hd(hd[6]), .ld(ld[6]), .co(carry[6]), .s(s[6]) );
   fa_crypt fa7 (.key(key[7]), .ci(carry[6]), .hd(hd[7]), .ld(ld[7]), .co(carry[7]), .s(s[7]) );

   assign ovf = carry[7];

endmodule // fa_crypt8

