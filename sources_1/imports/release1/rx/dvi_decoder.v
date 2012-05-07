//////////////////////////////////////////////////////////////////////////////
//
//  Xilinx, Inc. 2010                 www.xilinx.com
//
//  XAPPxxx
//
//////////////////////////////////////////////////////////////////////////////
//
//  File name :       dvi_decoder.v
//
//  Description :     Spartan-6 DVI decoder top module
//
//
//  Author :          Bob Feng
//
//  Disclaimer: LIMITED WARRANTY AND DISCLAMER. These designs are
//              provided to you "as is". Xilinx and its licensors makeand you
//              receive no warranties or conditions, express, implied,
//              statutory or otherwise, and Xilinx specificallydisclaims any
//              implied warranties of merchantability, non-infringement,or
//              fitness for a particular purpose. Xilinx does notwarrant that
//              the functions contained in these designs will meet your
//              requirements, or that the operation of these designswill be
//              uninterrupted or error free, or that defects in theDesigns
//              will be corrected. Furthermore, Xilinx does not warrantor
//              make any representations regarding use or the results ofthe
//              use of the designs in terms of correctness, accuracy,
//              reliability, or otherwise.
//
//              LIMITATION OF LIABILITY. In no event will Xilinx or its
//              licensors be liable for any loss of data, lost profits,cost
//              or procurement of substitute goods or services, or forany
//              special, incidental, consequential, or indirect damages
//              arising from the use or operation of the designs or
//              accompanying documentation, however caused and on anytheory
//              of liability. This limitation will apply even if Xilinx
//              has been advised of the possibility of such damage. This
//              limitation shall apply not-withstanding the failure ofthe
//              essential purpose of any limited remedies herein.
//
//  Copyright © 2004 Xilinx, Inc.
//  All rights reserved
//
//////////////////////////////////////////////////////////////////////////////
// Modifications copyright (c) 2011, Andrew "bunnie" Huang
// All rights reserved as permitted by law.
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

// comment out the below line to turn off SS clocking
// turned off currently because this form causes hdmi locking to fail, suspect lack of
// phase lock to be the issue
//`define SS_CLOCKING 1

module dvi_decoder (
  input  wire tmdsclk_p,      // tmds clock
  input  wire tmdsclk_n,      // tmds clock
  input  wire blue_p,         // Blue data in
  input  wire green_p,        // Green data in
  input  wire red_p,          // Red data in
  input  wire blue_n,         // Blue data in
  input  wire green_n,        // Green data in
  input  wire red_n,          // Red data in
  input  wire exrst,          // external reset input, e.g. reset button
  input  wire pllreset,       // just reset the PLL only (after LoL)

  output wire reset,          // rx reset
  output wire pclk,           // regenerated pixel clock
  output wire pclkx2,         // double rate pixel clock
  output wire pclkx10,        // 10x pixel as IOCLK
  output wire pllclk0,        // send pllclk0 out so it can be fed into a different BUFPLL
  output wire pllclk1,        // PLL x1 output
  output wire pllclk2,        // PLL x2 output

  output wire pll_lckd,       // send pll_lckd out so it can be fed into a different BUFPLL
  output wire serdesstrobe,   // BUFPLL serdesstrobe output
  output wire tmdsclk,        // TMDS cable clock

  output reg hsync,          // hsync data
  output reg vsync,          // vsync data
  output reg de,             // data enable
  output wire basic_de,       // a DE that strobes even during data guardbands
  
  output wire blue_vld,
  output wire green_vld,
  output wire red_vld,
  output wire blue_rdy,
  output wire green_rdy,
  output wire red_rdy,

  output wire psalgnerr,

  output wire [29:0] sdout,  // note this comes 2 pixclk earlier than pix colors
  output reg [7:0] red,      // pixel data out
  output reg [7:0] green,    // pixel data out
  output reg [7:0] blue,     // pixel data out

  output reg encoding,       // high when data island is valid
  output wire hdcp_ena,      // OR of data and video encyrption internal signals
  output wire [3:0] red_di,   // red data island
  output wire [3:0] green_di, // green data island
  output wire [3:0] blue_di,  // blue data island
  output reg  data_gb,       // guardbands
  output reg  video_gb,
  output reg [3:0] ctl_code, // control code
  output reg       cv,
  output wire line_end       // fast-track signal that line ends for HDCP rekey
		    ) ; 

   wire 	    g_dgb, b_dgb, r_dgb;
   wire 	    g_vgb, b_vgb, r_vgb;
   wire             g_cv, b_cv, r_cv;

   wire [3:0] 	    r_t4;
   wire [3:0] 	    b_t4;
   wire [3:0] 	    g_t4;
   
   wire [9:0] 	    sdout_blue;
   wire [9:0] 	    sdout_green;
   wire [9:0] 	    sdout_red;

   wire [7:0] red_q1;
   wire [7:0] blu_q1;
   wire [7:0] grn_q1;

   reg [7:0] red_q2;
   reg [7:0] blu_q2;
   reg [7:0] grn_q2;

   reg 	     de_q1, de_reg;
   reg       hsync_q1, vsync_q1;
   wire      de_q2;
   wire      hsync_q2, vsync_q2;
   reg       cv_q;

   reg 	     data_gb_q, video_gb_q;
   reg [3:0] ctl_code_q;
   wire [3:0]  ctl_code_wire;
   
/*
  assign sdout = {sdout_red[9], sdout_green[9], sdout_blue[9], sdout_red[8], sdout_green[8], sdout_blue[8],
                  sdout_red[7], sdout_green[7], sdout_blue[7], sdout_red[6], sdout_green[6], sdout_blue[6],
                  sdout_red[5], sdout_green[5], sdout_blue[5], sdout_red[4], sdout_green[4], sdout_blue[4],
                  sdout_red[3], sdout_green[3], sdout_blue[3], sdout_red[2], sdout_green[2], sdout_blue[2],
                  sdout_red[1], sdout_green[1], sdout_blue[1], sdout_red[0], sdout_green[0], sdout_blue[0]} ;
*/
   parameter INIT =      8'b1 << 0;
   parameter GOING_T4 =  8'b1 << 1;
   parameter TERC4 =     8'b1 << 2;
   parameter LEAVE_T4 =  8'b1 << 3;
   parameter GOING_VID = 8'b1 << 4;
   parameter VIDEO =     8'b1 << 5;
   parameter PREAM_T4  = 8'b1 << 6;
   parameter PREAM_VID = 8'b1 << 7;
   parameter nSTATES =   8;

   reg [(nSTATES-1):0] cstate = {{(nSTATES-1){1'b0}},1'b1};
   reg [(nSTATES-1):0] nstate;

   parameter ENC_TMDS = 1'b0;
   parameter ENC_TERC4 = 1'b1;
   parameter HDCP_OFF = 1'b0;
   parameter HDCP_ON  = 1'b1;
   
//   reg 		       encoding;
   reg 		       encrypting_data;
   reg 		       encrypting_video;

   assign hdcp_ena = encrypting_data | encrypting_video;

   always @ (posedge pclk or posedge reset) begin
      if (reset)
	cstate <= INIT;
            else
	cstate <=#1 nstate;
   end

   always @ (*) begin
      case (cstate) //synthesis parallel_case full_case
	//// NOTE NOTE NOTE
	//// green channel uses same code for video and data gb
	//// so we can't consider its information in this state machine
	INIT: begin
	   if( b_vgb & r_vgb & g_vgb ) begin
//	   if( b_vgb | r_vgb | g_vgb ) begin
	      nstate = GOING_VID;
	   end else if (ctl_code_wire == 4'b0101) begin
	      // we've found a preamble for data
	      nstate =  PREAM_T4;
	   end else if (ctl_code_wire == 4'b0001) begin
	      nstate = PREAM_VID;
	   end else begin
	      nstate = INIT;
	   end
	end
	PREAM_T4: begin
	   if( b_vgb & r_vgb & g_vgb ) begin
//	   if( b_vgb | r_vgb | g_vgb ) begin
	      nstate = GOING_VID;
	   end else if (r_dgb & g_dgb) begin
	      // data guardband only happens on b/r channels
	      nstate =  GOING_T4;
	   end else if (ctl_code_wire == 4'b0101) begin
	      nstate = PREAM_T4;
	   end else begin
	      nstate = INIT;
	   end
	end
	GOING_T4: begin
	    // wait till both dgb signals drop
	   nstate = (r_dgb & g_dgb) ? GOING_T4 : TERC4;
	end
	TERC4: begin
	   if( b_cv | r_cv | g_cv ) begin
	      nstate = INIT;
	   end else if( b_vgb & r_vgb & g_vgb ) begin
//	   end else if( b_vgb | r_vgb | g_vgb ) begin
	      // if we see a video guardband and we think we're in terc4 encoding
	      // it means we missed the end of data guardband; simply re-initialize
	      // the machine so we always recover from bit error drops
	      nstate = GOING_VID;
	   end else if( r_dgb & g_dgb ) begin
	      // otherwise, gracefully leave
	      nstate = LEAVE_T4;
	   end else begin
	      nstate = TERC4;
	   end
	end // case: TERC4
	LEAVE_T4: begin
	   // wait till both dgb signals drop
	   nstate = (r_dgb & g_dgb) ? LEAVE_T4 : INIT;
	end
	PREAM_VID: begin
	   if( ctl_code_wire == 4'b0001 ) begin
	      nstate = PREAM_VID;
	   end else if( b_vgb & r_vgb & g_vgb ) begin
//	   end else if( b_vgb | r_vgb | g_vgb ) begin
	      nstate = GOING_VID;
	   end else  begin
	      nstate = INIT;
	   end
	end
	GOING_VID: begin
	   nstate = ( b_vgb & r_vgb & g_vgb ) ? GOING_VID : VIDEO;
//	   nstate = ( b_vgb | r_vgb | g_vgb ) ? GOING_VID : VIDEO;
	end
	VIDEO: begin
	   if( b_cv | r_cv | g_cv ) begin
//	   if( b_cv & r_cv & g_cv ) begin
	      nstate = INIT;
	   end else begin
	      nstate = VIDEO;
	   end
	end
      endcase // case (cstate)
   end

   always @ (posedge pclk or posedge reset) begin
      if( reset ) begin
	 encoding <=#1 ENC_TMDS;
	 encrypting_data <=#1 HDCP_OFF;
      end else begin
	 case (cstate) // synthesis parallel_case full_case
	   INIT: begin
	      encoding         <= #1 ENC_TMDS;
	      encrypting_data  <= #1 HDCP_OFF;
	      encrypting_video <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   PREAM_T4: begin
	      encoding         <= #1 ENC_TMDS;
	      encrypting_data  <= #1 HDCP_OFF;
	      encrypting_video <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   GOING_T4: begin
	      encoding         <= #1 ENC_TERC4;
	      encrypting_data  <= #1 HDCP_OFF;
	      encrypting_video <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   TERC4: begin
	      encoding         <= #1 ENC_TERC4;
	      encrypting_data  <= #1 HDCP_ON;
	      encrypting_video <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   LEAVE_T4: begin
	      encoding         <= #1 ENC_TERC4;
	      encrypting_video <= #1 HDCP_OFF;
	      encrypting_data  <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   PREAM_VID: begin
	      encoding         <= #1 ENC_TMDS;
	      encrypting_data  <= #1 HDCP_OFF;
	      encrypting_video <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   GOING_VID: begin
	      encoding         <= #1 ENC_TMDS;
	      encrypting_data  <= #1 HDCP_OFF;
	      encrypting_video <= #1 HDCP_OFF;
	      de <= #1 1'b0;
	   end
	   VIDEO: begin
	      encoding         <= #1 ENC_TMDS;
	      encrypting_data  <= #1 HDCP_OFF;
	      encrypting_video <= #1 HDCP_ON;
	      de <= #1 1'b1;
	   end
	 endcase // case (cstate)
      end // else: !if( reset )
   end // always @ (posedge pclk or posedge reset)
   
   
  assign sdout = {sdout_red[9:5], sdout_green[9:5], sdout_blue[9:5],
                  sdout_red[4:0], sdout_green[4:0], sdout_blue[4:0]};

  wire de_b, de_g, de_r;

   assign de_q2 = de_b;
   // to do: modify this against guard bands -- if dgb activates, set a flag to set terc4 coding
   // until dgb triggers again; include a force-clear if a vgb is encountered

 //wire blue_vld, green_vld, red_vld;
 //wire blue_rdy, green_rdy, red_rdy;

  wire blue_psalgnerr, green_psalgnerr, red_psalgnerr;

  //
  // Send TMDS clock to a differential buffer and then a BUFIO2
  // This is a required path in Spartan-6 feed a PLL CLKIN
  //
  wire rxclkint;
  IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE")
  ) ibuf_rxclk (.I(tmdsclk_p), .IB(tmdsclk_n), .O(rxclkint));

  wire rxclk;
`ifdef SS_CLOCKING   
   wire rxclk_pre_ss; //

  BUFIO2 #(.DIVIDE_BYPASS("TRUE"), .DIVIDE(1))
  bufio_tmdsclk (.DIVCLK(rxclk_pre_ss), .IOCLK(), .SERDESSTROBE(), .I(rxclkint));

  BUFG tmdsclk_bufg (.I(rxclk), .O(tmdsclk));

   // use spread spectrum clocking to reduce emissions...
   DCM_CLKGEN    #(
		   .DFS_OSCILLATOR_MODE("PHASE_FREQ_LOCK"), 
		   .CLKIN_PERIOD ("13.48"),
		   .SPREAD_SPECTRUM ("CENTER_LOW_SPREAD"),
 		   .CLKFX_MD_MAX("1.0"),
 		   .CLKFX_MULTIPLY	(4),
 		   .CLKFX_DIVIDE	(4) )
   dcm_fcc_spreads_me_wide (
		     .CLKIN   	(rxclk_pre_ss),
		     .FREEZEDCM (1'b0),
//		     .PROGDATA		(progdata_int),
//		     .PROGEN 			(progen_int),
//		     .PROGCLK			(clk26bufpll), 
//		     .PROGDONE		(progdone),
//		     .CLKFX180	(CLKFX180_DCM),
//		     .CLKFXDV	(CLKFXDV_DCM),
			    //		     .LOCKED  	(clkgen_locked),
		     .RST     	(rx0_reset),
		     .CLKFX   	(rxclk_ss)
			    );
   BUFG tmdsclk_bfgss (.I(rxclk_ss), .O(rxclk) ); //
`else // !`ifdef SS_CLOCKING

  BUFIO2 #(.DIVIDE_BYPASS("TRUE"), .DIVIDE(1))
  bufio_tmdsclk (.DIVCLK(rxclk), .IOCLK(), .SERDESSTROBE(), .I(rxclkint));

  BUFG tmdsclk_bufg (.I(rxclk), .O(tmdsclk));

`endif // !`ifdef SS_CLOCKING
   

  //
  // PLL is used to generate three clocks:
  // 1. pclk:    same rate as TMDS clock
  // 2. pclkx2:  double rate of pclk used for 5:10 soft gear box and ISERDES DIVCLK
  // 3. pclkx10: 10x rate of pclk used as IO clock
  //
   wire clkfbin;
   wire clkfbout;
   wire clkfbin_fb;
   
  PLL_BASE # (
//    .CLKIN_PERIOD(10.526315), // 95 MHz
    .CLKIN_PERIOD(13.481449525), // 74.176 MHz
    .CLKFBOUT_MULT(10), //set VCO to 10x of CLKIN
    .CLKOUT0_DIVIDE(1),
    .CLKOUT1_DIVIDE(10),
    .CLKOUT2_DIVIDE(5),
//    .COMPENSATION("INTERNAL")
//	      .BANDWIDTH("LOW"), // normally not here
    .COMPENSATION("SOURCE_SYNCHRONOUS")
  ) PLL_ISERDES (
    .CLKFBOUT(clkfbout),
    .CLKOUT0(pllclk0),
    .CLKOUT1(pllclk1),
    .CLKOUT2(pllclk2),
    .CLKOUT3(),
    .CLKOUT4(),
    .CLKOUT5(),
    .LOCKED(pll_lckd),
    .CLKFBIN(clkfbin_fb),
    .CLKIN(rxclk),
    .RST(exrst || pllreset)
  );

   // feedback to source-synchronize the clock
   BUFG pclkfbk (.I(clkfbout), .O(clkfbin) );
   BUFIO2FB pclkfbk_fb (.I(clkfbin), .O(clkfbin_fb) );
   
  //
  // Pixel Rate clock buffer
  //
  BUFG pclkbufg (.I(pllclk1), .O(pclk));

  //////////////////////////////////////////////////////////////////
  // 2x pclk is going to be used to drive IOSERDES2 DIVCLK
  //////////////////////////////////////////////////////////////////
  BUFG pclkx2bufg (.I(pllclk2), .O(pclkx2));

  //////////////////////////////////////////////////////////////////
  // 10x pclk is used to drive IOCLK network so a bit rate reference
  // can be used by IOSERDES2
  //////////////////////////////////////////////////////////////////
  
  wire bufpll_lock;
  BUFPLL #(.DIVIDE(5)) ioclk_buf (.PLLIN(pllclk0), .GCLK(pclkx2), .LOCKED(pll_lckd),
           .IOCLK(pclkx10), .SERDESSTROBE(serdesstrobe), .LOCK(bufpll_lock));

  assign reset = ~bufpll_lock;

   wire line_end_r;
   wire line_end_g;
   wire line_end_b;
   
   // note instance-specific decode since each channel has its own specific
   // guardband character. 
  decodeb dec_b (
    .reset        (reset),
    .pclk         (pclk),
    .pclkx2       (pclkx2),
    .pclkx10      (pclkx10),
    .serdesstrobe (serdesstrobe),
    .din_p        (blue_p),
    .din_n        (blue_n),
    .other_ch0_rdy(green_rdy),
    .other_ch1_rdy(red_rdy),
    .other_ch0_vld(green_vld),
    .other_ch1_vld(red_vld),

    .iamvld       (blue_vld),
    .iamrdy       (blue_rdy),
    .psalgnerr    (blue_psalgnerr),
    .c0           (hsync_q2),
    .c1           (vsync_q2),
    .de           (de_b),
    .sdout        (sdout_blue),
    .dout         (blu_q1),
    .dgb          (b_dgb),
    .vgb          (b_vgb),
    .ctl_vld      (b_cv),
    .line_end     (line_end_b)) ;

  decodeg dec_g (
    .reset        (reset),
    .pclk         (pclk),
    .pclkx2       (pclkx2),
    .pclkx10      (pclkx10),
    .serdesstrobe (serdesstrobe),
    .din_p        (green_p),
    .din_n        (green_n),
    .other_ch0_rdy(blue_rdy),
    .other_ch1_rdy(red_rdy),
    .other_ch0_vld(blue_vld),
    .other_ch1_vld(red_vld),

    .iamvld       (green_vld),
    .iamrdy       (green_rdy),
    .psalgnerr    (green_psalgnerr),
    .c0           (ctl_code_wire[0]),
    .c1           (ctl_code_wire[1]),
    .de           (de_g),
    .sdout        (sdout_green),
    .dout         (grn_q1),
    .dgb          (g_dgb),
    .vgb          (g_vgb),
    .ctl_vld      (g_cv),
    .line_end     (line_end_g)) ;
    
  decoder dec_r (
    .reset        (reset),
    .pclk         (pclk),
    .pclkx2       (pclkx2),
    .pclkx10      (pclkx10),
    .serdesstrobe (serdesstrobe),
    .din_p        (red_p),
    .din_n        (red_n),
    .other_ch0_rdy(blue_rdy),
    .other_ch1_rdy(green_rdy),
    .other_ch0_vld(blue_vld),
    .other_ch1_vld(green_vld),

    .iamvld       (red_vld),
    .iamrdy       (red_rdy),
    .psalgnerr    (red_psalgnerr),
    .c0           (ctl_code_wire[2]),
    .c1           (ctl_code_wire[3]),
    .de           (de_r),
    .sdout        (sdout_red),
    .dout         (red_q1),
    .dgb          (r_dgb),
    .vgb          (r_vgb),
    .ctl_vld      (r_cv),
    .line_end     (line_end_r)) ;

   assign basic_de = de_b | de_r | de_g | b_vgb | r_vgb | g_vgb;
   
   assign line_end = line_end_g | line_end_r | line_end_b;

  assign psalgnerr = red_psalgnerr | blue_psalgnerr | green_psalgnerr;

   // pipe alignment registers
   always @(posedge pclk or posedge reset) begin
      if( reset ) begin
	 red <= 8'b0;
	 red_q2 <= 8'b0;
	 blue <= 8'b0;
	 blu_q2 <= 8'b0;
	 green <= 8'b0;
	 grn_q2 <= 8'b0;

	 hsync <= 1'b0;
	 hsync_q1 <= 1'b0;
	 
	 vsync <= 1'b0;
	 vsync_q1 <= 1'b0;
	 
	 de_reg <= 1'b0;
	 de_q1 <= 1'b0;

	 data_gb_q <= 1'b0;
	 data_gb <= 1'b0;
	 video_gb_q <= 1'b0;
	 video_gb <= 1'b0;
	 ctl_code_q <= 4'b0;
	 ctl_code <= 4'b0;

	 cv_q <= 1'b0;
	 cv <= 1'b0;
      end else begin
	 red_q2 <= red_q1;
	 red <= red_q2;

	 blu_q2 <= blu_q1;
	 blue <= blu_q2;

	 grn_q2 <= grn_q1;
	 green <= grn_q2;

	 // reversed the naming convention for the following pipe stages
	 hsync <= hsync_q1;
	 hsync_q1 <= hsync_q2;

	 vsync <= vsync_q1;
	 vsync_q1 <= vsync_q2;

	 de_reg <= de_q1;
	 de_q1 <= de_q2;

	 data_gb_q <= r_dgb & g_vgb; // data guardbands only on red and green channels
	 data_gb <= data_gb_q;
	 
	 video_gb_q <= r_vgb & b_vgb & g_vgb;
	 video_gb <= video_gb_q;
	 
	 ctl_code_q <= ctl_code_wire;
	 ctl_code <= ctl_code_q;

	 cv_q <= b_cv & r_cv & g_cv;
	 cv <= cv_q;
      end // else: !if( reset )
   end // always @ (posedge pclk or posedge reset)
//   assign de = de_reg & (encoding == ENC_TMDS);
	   
   decode_terc4 dec_t4_g (
			 .rstin( reset ),
			 .clkin(pclk),
			 .din(sdout_green),
			 .dout(g_t4));
   
   decode_terc4 dec_t4_r (
			 .rstin( reset ),
			 .clkin(pclk),
			 .din(sdout_red),
			 .dout(r_t4));
   
   decode_terc4 dec_t4_b (
			 .rstin( reset ),
			 .clkin(pclk),
			 .din(sdout_blue),
			 .dout(b_t4));
   assign red_di = r_t4;
   assign green_di = g_t4;
   assign blue_di = b_t4;

endmodule
