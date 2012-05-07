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
module gbgen (
	      input pclk,
	      input rstin,
	      input vsync,
	      input hsync,
	      input sync_pol,
	      input de,

	      output gb,
	      output [3:0] code
	      );
   
   reg [11:0] 		   hpixel;
   wire [11:0] 		   papos;
   reg [11:0] 		   depos;

   reg 				    hsync_v; // active when high
   reg 				    hsync_v2;
   reg 				    vsync_v;
   reg 				    vsync_v2;
   reg 				    de_d;
   
   wire 			    hsync_rising;
   wire 			    vsync_rising;

   wire 			    pa;
   
   always @(posedge pclk) begin
      if( rstin ) begin
	 hpixel <= 0;
	 depos <= 0;
      end else begin
	 if( hsync_rising ) begin
	    hpixel <= 0;
	 end else begin
	    hpixel <= hpixel + 12'b1;
	 end

	 if( de && !de_d ) begin // de is rising
	    depos <= hpixel;
	 end else begin
	    depos <= depos;
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk)

   assign papos = depos - 12'd10; // note: decimal 10, not binary 10
   //  0   1   2   3   4   5   6   7
   //  |   |   |   |   |   |   |   |
   //  ct  pa  gb  gb  v   v   v   v
   //
   //  depos = 4
   //  gbpos = 2

   assign gb = (hpixel >= (depos - 12'd2)) && (hpixel < depos);
   assign pa = (hpixel >= papos) && (hpixel < depos);

   assign code[3:1] = 3'b0; // these never change
   assign code[0] = pa;   // this is the only bit that changes on pre-amble
   
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 hsync_v <= 0;
	 vsync_v <= 0;

	 hsync_v2 <= 0;
	 vsync_v2 <= 0;
	 de_d <= 0;
      end else begin
	 hsync_v <= hsync ^ !sync_pol;
	 vsync_v <= vsync ^ !sync_pol;

	 de_d <= de;
	 hsync_v2 <= hsync_v; // just a delayed version
	 vsync_v2 <= vsync_v;
      end // else: !if( rstin )
   end // always @ (posedge pclk or posedge rstin)
   assign hsync_rising = hsync_v & !hsync_v2;
   assign vsync_rising = vsync_v & !vsync_v2;
   
endmodule // gbgen
