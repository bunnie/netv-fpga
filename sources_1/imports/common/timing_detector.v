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

module timing_detector (
			input wire pclk,
			input wire rstin, // asynchronous, as it is asserted during no-clock condition
			input wire vsync, // assume positive-polarity sync on input
			input wire hsync, // assume positive-polarity sync on input
			input wire de,
			input wire refclk,
			input wire lcd_de,
			input wire lcd_vsync,

			output reg [11:0] hactive, // in pixels
			output reg [11:0] vactive, // in lines 
			output reg [11:0] htotal, // in pixels 
			output reg [23:0] vtotal, // ** in PIXELS ** must divide by htotal in software
			output reg [7:0] h_frontporch, // in pixels 
			output reg [7:0] h_backporch,  // in pixels 
			output reg [23:0] v_frontporch, // ** in PIXELS **
			output reg [23:0] v_backporch,  // ** in PIXELS **
			output reg [7:0] hsync_width, // in pixels 
			output reg [23:0] vsync_width, // ** in PIXELS **
			output reg [11:0] lcd_de_latency, // in lines
			output reg [11:0] lcd_vsync_latency, // in lines

			output reg [23:0] refclkcnt // number of refclocks in a field
			);

   reg 		  vsync_d;
   reg 		  hsync_d;
   reg 		  de_d;

   wire 	  vsync_rising;
   wire 	  hsync_rising;
   wire 	  de_rising;

   wire 	  vsync_falling;
   wire 	  hsync_falling;
   wire 	  de_falling;

   reg [11:0] 	  hcount;
   reg [7:0] 	  hsync_width_cnt;
   reg [23:0] 	  vcount;
   reg [11:0] 	  de_count;
   reg [11:0] 	  vactive_count;
   reg [7:0] 	  h_fp_count;
   reg [7:0] 	  h_bp_count;
   reg [23:0] 	  v_fp_count;
   reg [23:0] 	  v_bp_count;

   reg 		  vsync_refclk;
   reg 		  vsync__refclk;
   reg 		  vsync___refclk;
   reg 		  vsync____refclk;
   wire 	  vsync_refclk_rising;
   reg [23:0] 	  refclkcounter;

   reg 		  lcd_de_pclk;
   reg 		  lcd_de_1pclk;
   reg 		  lcd_de_2pclk;

   reg 		  lcd_vsync_pclk;
   reg 		  lcd_vsync_1pclk;
   reg 		  lcd_vsync_2pclk;

   reg 		  first_de_rising;
   reg 		  first_lcd_de_rising;
   
   //// vertical front/back porch machine
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 v_fp_count <= 0;
	 v_bp_count <= 0;
	 v_frontporch <= 0;
	 v_backporch <= 0;
	 first_de_rising <= 1;
      end else begin
	 if( vsync_falling ) begin
	    v_fp_count <= 0;
	    first_de_rising <= 1;
	 end else begin
	    if( v_fp_count == 24'hFF_FFFF ) begin
	       v_fp_count <= v_fp_count;
	    end else begin
	       v_fp_count <= v_fp_count + 24'b1; // counting in pixels
	    end

	    if( de_rising ) begin
	       first_de_rising <= 0;
	    end else begin
	       first_de_rising <= first_de_rising;
	    end
	 end

	 if( de_rising && first_de_rising ) begin
	    v_frontporch <= v_fp_count;
	 end else begin
	    v_frontporch <= v_frontporch;
	 end

	 if( de_falling ) begin
	    v_bp_count <= 0;
	 end else begin
	    if( v_bp_count == 24'hFF_FFFF ) begin
	       v_bp_count <= v_bp_count;
	    end else begin
	       v_bp_count <= v_bp_count + 24'b1; // counting in pixels
	    end
	 end

	 if( vsync_rising ) begin
	    v_backporch <= v_bp_count;
	 end else begin
	    v_backporch <= v_backporch;
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk)
   
   //// horizonal front/back porch machine
   always @(posedge pclk  or posedge rstin) begin
      if( rstin ) begin
	 h_fp_count <= 0;
	 h_bp_count <= 0;
	 h_frontporch <= 0;
	 h_backporch <= 0;
      end else begin
	 if( hsync_falling ) begin
	    h_fp_count <= 8'b0;
	 end else begin
	    if( h_fp_count == 8'b1111_1111 ) begin
	       h_fp_count <= h_fp_count; // saturate to catch de-only timings
	    end else begin
	       h_fp_count <= h_fp_count + 8'b1;
	    end
	 end

	 if( de_rising ) begin
	    h_frontporch <= h_fp_count + 8'b10; // this is a bit of a hack, why is one pixel missing?
	 end else begin
	    h_frontporch <= h_frontporch;
	 end

	 if( de_falling ) begin
	    h_bp_count <= 0;
	 end else begin
	    if( h_bp_count == 8'b1111_1111 ) begin
	       h_bp_count <= h_bp_count;
	    end else begin
	       h_bp_count <= h_bp_count + 8'b1;
	    end
	 end

	 if( hsync_rising ) begin
	    h_backporch <= h_bp_count;
	 end else begin
	    h_backporch <= h_backporch;
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk)
   
   //// vsync_width machine
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 vsync_width <= 0;
      end else begin
	 if( vsync_rising ) begin
	    vsync_width <= vsync_width;
	    // vcount is reset on vsync_rising as well
	 end else begin
	    if( vsync_falling ) begin
	       vsync_width[23:0] <= vcount[23:0]; // counting in pixels
	    end else begin
	       vsync_width <= vsync_width;
	    end
	 end
      end
   end // always @ (posedge pclk)
   
   //// hsync_width machine
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 hsync_width <= 0;
	 hsync_width_cnt <= 8'b0;
      end else begin
	 if( hsync_rising ) begin
	    hsync_width <= hsync_width;
	    hsync_width_cnt <= 8'b1;
	 end else begin
	    if( hsync_falling ) begin
	       hsync_width <= hsync_width_cnt;
	    end else begin
	       hsync_width <= hsync_width;
	    end
	    hsync_width_cnt <= hsync_width_cnt + 8'b1;
	 end
      end
   end // always @ (posedge pclk)
   
   //// vactive machine
   ////   add detectors for lcd_de_latency and lcd_vsync_latency
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 vactive <= 0;
	 vactive_count <= 0;

	 lcd_de_latency <= 0;
	 lcd_vsync_latency <= 0;

	 lcd_de_pclk <= 0;
	 lcd_de_1pclk <= 0;
	 lcd_de_2pclk <= 0;
	 
	 lcd_vsync_pclk <= 0;
	 lcd_vsync_1pclk <= 0;
	 lcd_vsync_2pclk <= 0;

	 first_lcd_de_rising <= 1;
      end else begin // if ( rstin )
	 if( vsync_rising ) begin
	    vactive <= vactive_count;
	    vactive_count <= 0;
	 end else begin
	    if( de_rising ) begin  // counting in lines
	       vactive_count <= vactive_count + 12'b1;
	    end else begin
	       vactive_count <= vactive_count;
	    end
	    vactive <= vactive;
	 end // else: !if( vsync_rising )

	 lcd_de_2pclk <= lcd_de;
	 lcd_de_1pclk <= lcd_de_2pclk;
	 lcd_de_pclk <= lcd_de_1pclk;

	 lcd_vsync_2pclk <= lcd_vsync;
	 lcd_vsync_1pclk <= lcd_vsync_2pclk;
	 lcd_vsync_pclk <= lcd_vsync_1pclk;

	 if( vsync_rising ) begin
	    first_lcd_de_rising <= 1;
	 end else begin
	    if( !lcd_de_pclk & lcd_de_1pclk ) begin // rising edge
	       first_lcd_de_rising <= 0;
	    end else begin
	       first_lcd_de_rising <= first_lcd_de_rising;
	    end
	 end

	 // look for the rising edge
	 if( !lcd_de_pclk & lcd_de_1pclk & first_lcd_de_rising ) begin
	    lcd_de_latency <= vactive_count;
	 end else begin
	    lcd_de_latency <= lcd_de_latency;
	 end

	 // look for the rising edge
	 if( !lcd_vsync_pclk & lcd_vsync_1pclk ) begin
	    lcd_vsync_latency <= vactive_count;
	 end else begin
	    lcd_vsync_latency <= lcd_vsync_latency;
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk)

   //// hactive mcahine
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 de_count <= 0;
	 hactive <= 0;
      end else begin
	 if( de_rising ) begin
	    de_count <= 12'b1; // first pixel counts
	 end else if( de ) begin
	    de_count <= de_count + 12'b1;
	 end else begin
	    de_count <= de_count;
	 end

	 if( de_falling ) begin
	    hactive <= de_count;
	 end else begin
	    hactive <= hactive;
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk)

   //// htotal, vtotal machine
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 hcount <= 0;
	 vcount <= 0;
	 
	 htotal <= 0;
	 vtotal <= 0;
      end else begin
	 if( vsync_rising ) begin
	    vtotal <= vcount;
	    vcount <= 24'b1;
	 end else begin
	    vcount <= vcount + 24'b1; /// counting in pixels
	    vtotal <= vtotal;
	 end

	 if( de_rising ) begin
	    htotal <= hcount;
	    hcount <= 12'b1;
	 end else begin
	    hcount <= hcount + 12'b1;
	    htotal <= htotal;
	 end
      end // else: !if( rstin )
   end // always @ (posedge pclk)
	 
   //// refclock machine
   //// lots of cross-domain madness that might break things.
   always @(posedge refclk or posedge rstin) begin
      if( rstin ) begin
	 refclkcnt <= 0;
	 refclkcounter <= 0;
	 
	 vsync____refclk <= 0;
	 vsync___refclk <= 0;
	 vsync__refclk <= 0;
	 vsync_refclk <= 0;
      end else begin
	 vsync_refclk <= vsync__refclk;
	 vsync__refclk <= vsync___refclk;
	 vsync___refclk <= vsync____refclk;
	 vsync____refclk <= vsync;

	 if( vsync_refclk_rising ) begin
	    refclkcnt <= refclkcounter;
	    refclkcounter <= 24'b1;
	 end else begin
	    refclkcnt <= refclkcnt;
	    refclkcounter <= refclkcounter + 24'b01;
	 end

      end // else: !if( rstin )
   end // always @ (posedge refclk or posedge rstin)
   assign vsync_refclk_rising = !vsync_refclk & vsync__refclk;

   
   //// rising/falling edge extraction machine
   always @(posedge pclk or posedge rstin) begin
      if(rstin) begin
	 vsync_d <= 0;
	 hsync_d <= 0;
	 de_d <= 0;
      end else begin
	 vsync_d <= vsync;
	 hsync_d <= hsync;
	 de_d <= de;
      end
   end // always @ (posedge pclk)

   assign vsync_rising = vsync & !vsync_d;
   assign hsync_rising = hsync & !hsync_d;
   assign de_rising = de & !de_d;
   
   assign vsync_falling = !vsync & vsync_d;
   assign hsync_falling = !hsync & hsync_d;
   assign de_falling = !de & de_d;

endmodule // timing_detector
