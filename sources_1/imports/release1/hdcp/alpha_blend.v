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

module alpha_blend(
		   input wire clk,
		   input wire reset,
		   input wire [7:0] enc_b,
		   input wire [7:0] enc_r,
		   input wire [7:0] enc_g,
		   input wire [2:0] alpha_vid,
		   input wire alpha_en,
		   input wire [7:0] lcd_b,
		   input wire [7:0] lcd_r,
		   input wire [7:0] lcd_g,
		   input wire [23:0] cipher_stream,
		   output wire [7:0] alpha_enc_b,
		   output wire [7:0] alpha_enc_r,
		   output wire [7:0] alpha_enc_g
		   );


   wire [7:0] 			    enc_b_scaled;
   wire [7:0] 			    enc_r_scaled;
   wire [7:0] 			    enc_g_scaled;

   wire [7:0] 			    enc_b_summed;
   wire [7:0] 			    enc_r_summed;
   wire [7:0] 			    enc_g_summed;
   
   assign enc_b_scaled = (enc_b >> alpha_vid) ^ (cipher_stream[7:0] ^ (cipher_stream[7:0] >> alpha_vid));
   assign enc_g_scaled = (enc_g >> alpha_vid) ^ (cipher_stream[15:8] ^ (cipher_stream[15:8] >> alpha_vid));
   assign enc_r_scaled = (enc_r >> alpha_vid) ^ (cipher_stream[23:16] ^ (cipher_stream[23:16] >> alpha_vid));

   fa_crypt8 bCryptAdd( .key(cipher_stream[7:0]), .hd(enc_b_scaled), .ld(lcd_b), .s(enc_b_summed) );
   fa_crypt8 gCryptAdd( .key(cipher_stream[15:8]), .hd(enc_g_scaled), .ld(lcd_g), .s(enc_g_summed) );
   fa_crypt8 rCryptAdd( .key(cipher_stream[23:16]), .hd(enc_r_scaled), .ld(lcd_r), .s(enc_r_summed) );

   assign alpha_enc_b = alpha_en ? enc_b_summed : (lcd_b ^ cipher_stream[7:0]);
   assign alpha_enc_g = alpha_en ? enc_g_summed : (lcd_g ^ cipher_stream[15:8]);
   assign alpha_enc_r = alpha_en ? enc_r_summed : (lcd_r ^ cipher_stream[23:16]);
   
   // temporary hack to just pass through, this will be replaced
//   assign alpha_enc_b = cipher_stream[7:0] ^ lcd_b[7:0];
//   assign alpha_enc_g = cipher_stream[15:8] ^ lcd_g[7:0];
//   assign alpha_enc_r = cipher_stream[23:16] ^ lcd_r[7:0];
						 
endmodule // alpha_blend

