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
`timescale 1 ns / 1ps

module boxtiming (
		  input  wire       pclk,
		  input  wire       rstin,
		  input  wire       vsync,
		  input  wire       hsync,
		  input  wire       sync_pol, // 0 means active 0, 1 means active 1
		  input  wire       de,
		  input  wire       cv,
		  input  wire [11:0] hpos,
		  input  wire [11:0] hsize,
		  input  wire [11:0] vpos,
		  input  wire [11:0] vsize,
		  output reg       box_active
		  );

   reg [11:0] 			    hcount;
   reg [11:0] 			    vcount;
   
   reg 				    hsync_v; // active when high
   reg 				    hsync_v2;
   reg 				    vsync_v;
   reg 				    vsync_v2;
   reg 				    de_d;
   
   reg 				    active;

   wire 			    hsync_rising;
   wire 			    vsync_rising;
   wire 			    de_rising;
   wire 			    de_falling;
   
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 hsync_v <= 0;
	 vsync_v <= 0;

	 hsync_v2 <= 0;
	 vsync_v2 <= 0;

	 de_d <= 0;
      end else begin
	 de_d <= de;
	 
	 if( cv ) begin
	    hsync_v <= hsync ^ !sync_pol;
	    vsync_v <= vsync ^ !sync_pol;
	 end else begin
	    hsync_v <= hsync_v;
	    vsync_v <= vsync_v;
	 end

	 hsync_v2 <= hsync_v; // just a delayed version
	 vsync_v2 <= vsync_v;
      end // else: !if( rstin )
   end // always @ (posedge pclk or posedge rstin)
   assign hsync_rising = hsync_v & !hsync_v2;
   assign vsync_rising = vsync_v & !vsync_v2;
   assign de_rising = de & !de_d;
   assign de_falling = !de & de_d;
	 
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 hcount <= 0;
      end else begin
	 if( de_rising ) begin
	    hcount <= 12'b0000_0000_0000;
	 end else begin
	    if( de ) begin
	       hcount <= hcount + 12'b0000_0000_0001;
	    end else begin
	       hcount <= hcount;
	    end
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk or posedge rstin)

   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 vcount <= 0;
      end else begin
	 if( vsync_rising ) begin
	    vcount <= 12'b0000_0000_0000;
	 end else begin
	    if( de_falling ) begin // this may be a bug but I think it's worked around elsewhere
	       vcount <= vcount + 12'b0000_0000_0001;
	    end else begin
	       vcount <= vcount;
	    end
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk or posedge rstin)

   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 active <= 0;
      end else begin
	 if( (hcount >= hpos) && (hcount < (hpos + hsize)) && 
	     (vcount >= vpos) && (vcount < (vpos + vsize)) ) begin
	    active <= 1'b1;
	 end else begin
	    active <= 1'b0;
	 end
      end
      
      box_active <= active;
   end // always @ (posedge pclk or posedge rstin)

endmodule
   