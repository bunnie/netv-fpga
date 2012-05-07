//////////////////////////////////////////////////////////////////////////////
//
//  Xilinx, Inc. 2009                 www.xilinx.com
//
//  XAPP xyz
//
//////////////////////////////////////////////////////////////////////////////
//
//  File name :       dvi_encoder.v
//
//  Description :     dvi_encoder 
//
//  Date - revision : April 2009 - 1.0.0
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
//  Copyright © 2009 Xilinx, Inc.
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

module dvi_encoder_top (
  input  wire       pclk,           // pixel clock
  input  wire       pclkx2,         // pixel clock x2
  input  wire       pclkx10,        // pixel clock x2
  input  wire       serdesstrobe,   // OSERDES2 serdesstrobe
  input  wire       rstin,          // reset
  input  wire [7:0] blue_din,       // Blue data in
  input  wire [7:0] green_din,      // Green data in
  input  wire [7:0] red_din,        // Red data in
  input  wire       hsync,          // hsync data
  input  wire       vsync,          // vsync data
  input  wire       de,             // data enable
  output wire [3:0] TMDS,
  output wire [3:0] TMDSB,
  input  wire       vid_pa,
  input  wire       vid_gb,
  input  wire       dat_pa,
  input  wire       dat_gb,
  input  wire       dat_ena,
  input  wire [9:0] dat_din,
  input  wire [3:0] ctl_code,
  input  wire [29:0] bypass_sdata,
  input  wire       bypass_ena,
  output reg        byp_error,
  input  wire       box_active
);
    
  wire 	[9:0]	red ;
  wire 	[9:0]	green ;
  wire 	[9:0]	blue ;

   wire [9:0] 	red_t4 ;
   wire [9:0] 	green_t4 ;
   wire [9:0] 	blue_t4 ;

  wire [4:0] tmds_data0, tmds_data1, tmds_data2;
  wire [2:0] tmdsint;

  //
  // Forward TMDS Clock Using OSERDES2 block
  //
  reg [4:0] tmdsclkint = 5'b00000;
  reg toggle = 1'b0;

  always @ (posedge pclkx2 or posedge rstin) begin
    if (rstin)
      toggle <= 1'b0;
    else
      toggle <= ~toggle;
  end

  always @ (posedge pclkx2) begin
    if (toggle)
      tmdsclkint <= 5'b11111;
    else
      tmdsclkint <= 5'b00000;
  end

  wire tmdsclk;

  serdes_n_to_1 #(
    .SF           (5))
  clkout (
    .iob_data_out (tmdsclk),
    .ioclk        (pclkx10),
    .serdesstrobe (serdesstrobe),
    .gclk         (pclkx2),
    .reset        (rstin),
    .datain       (tmdsclkint));

  OBUFDS TMDS3 (.I(tmdsclk), .O(TMDS[3]), .OB(TMDSB[3])) ;// clock

  //
  // Forward TMDS Data: 3 channels
  //
  serdes_n_to_1 #(.SF(5)) oserdes0 (
             .ioclk(pclkx10),
             .serdesstrobe(serdesstrobe),
             .reset(rstin),
             .gclk(pclkx2),
             .datain(tmds_data0),
             .iob_data_out(tmdsint[0])) ;

  serdes_n_to_1 #(.SF(5)) oserdes1 (
             .ioclk(pclkx10),
             .serdesstrobe(serdesstrobe),
             .reset(rstin),
             .gclk(pclkx2),
             .datain(tmds_data1),
             .iob_data_out(tmdsint[1])) ;

  serdes_n_to_1 #(.SF(5)) oserdes2 (
             .ioclk(pclkx10),
             .serdesstrobe(serdesstrobe),
             .reset(rstin),
             .gclk(pclkx2),
             .datain(tmds_data2),
             .iob_data_out(tmdsint[2])) ;

  OBUFDS TMDS0 (.I(tmdsint[0]), .O(TMDS[0]), .OB(TMDSB[0])) ;
  OBUFDS TMDS1 (.I(tmdsint[1]), .O(TMDS[1]), .OB(TMDSB[1])) ;
  OBUFDS TMDS2 (.I(tmdsint[2]), .O(TMDS[2]), .OB(TMDSB[2])) ;

  encodeb encb (
    .clkin	(pclk),
    .rstin	(rstin),
    .din		(blue_din),
    .c0			(hsync),
    .c1			(vsync),
    .de			(de),
    .dout		(blue),
    .vid_gb             (vid_gb)) ;

  encodeg encg (
    .clkin	(pclk),
    .rstin	(rstin),
    .din		(green_din),
    .c0			(ctl_code[0]), // bit 0
    .c1			(ctl_code[1]), // bit 1
    .de			(de),
    .dout		(green),
    .vid_gb             (vid_gb)) ;
    
  encoder encr (
    .clkin	(pclk),
    .rstin	(rstin),
    .din		(red_din),
    .c0			(ctl_code[2]), // bit 2
    .c1			(ctl_code[3]), // bit 3
    .de			(de),
    .dout		(red),
    .vid_gb             (vid_gb)) ;

encode_terc4 engb_t4
  (     .clkin    (pclk),
	.rstin    (rstin),
	.din      ( {dat_din[9] | dat_gb, dat_din[8] | dat_gb, vsync, hsync} ),
	.dout     (blue_t4),
	.dat_gb   (1'b0)  // gb is considered with sync
   );
  
encode_terc4 encg_t4 
  (     .clkin    (pclk),
	.rstin    (rstin),
	.din      (dat_din[3:0]),
	.dout     (green_t4),
	.dat_gb   (dat_gb)
   );

encode_terc4 encr_t4 
  (     .clkin    (pclk),
	.rstin    (rstin),
	.din      (dat_din[7:4]),
	.dout     (red_t4),
	.dat_gb   (dat_gb)
   );
   
   // pipe alignment
   reg 		dat_ena_q, dat_ena_reg, dat_ena_r2;
   reg          dat_gb_q, dat_gb_reg, dat_gb_r2;
   
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 dat_ena_q <= 1'b0;
	 dat_ena_reg <= 1'b0;
	 dat_ena_r2 <= 1'b0;
	 dat_gb_q <= 1'b0;
	 dat_gb_reg <= 1'b0;
	 dat_gb_r2 <= 1'b0;
      end else begin
	 dat_ena_q <= dat_ena;
	 dat_ena_reg <= dat_ena_q;
	 dat_ena_r2 <= dat_ena_reg;
	 
	 dat_gb_q <= dat_gb;
	 dat_gb_reg <= dat_gb_q;
	 dat_gb_r2 <= dat_gb_reg;
      end
   end

   // insert four pipe stages to s_data override
   reg [29:0] byp_sd1;
   reg [29:0] byp_sd2;
   reg [29:0] byp_sd3;
   reg [29:0] byp_sd4;
   reg [29:0] byp_sd5;
   reg [4:0]  box_active_q;
   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 byp_sd1 <= 30'b0;
	 byp_sd2 <= 30'b0;
	 byp_sd3 <= 30'b0;
	 byp_sd4 <= 30'b0;
	 byp_sd5 <= 30'b0;
      end else begin
	 byp_sd1 <= bypass_sdata;
	 byp_sd2 <= byp_sd1;
	 byp_sd3 <= byp_sd2;
	 byp_sd4 <= byp_sd3;
	 byp_sd5 <= byp_sd4;

	 box_active_q[4] <= box_active_q[3];
	 box_active_q[3] <= box_active_q[2];
	 box_active_q[2] <= box_active_q[1];
	 box_active_q[1] <= box_active_q[0];
	 box_active_q[0] <= box_active;
      end // else: !if( rstin )
   end // always @ (posedge pclk or posedge rstin)

//   wire [29:0] s_data_x = (dat_ena_r2 | dat_gb_r2)  ? 
//	    {red_t4[9:5], green_t4[9:5], blue_t4[9:5],
//             red_t4[4:0], green_t4[4:0], blue_t4[4:0]} :
//            {red[9:5], green[9:5], blue[9:5],
//             red[4:0], green[4:0], blue[4:0]};

   // this destroys our ability to bypass sound, but fixes the problem
   // where HDMI data of Red = 0x55, Green = 0x55, blue = anything
   // causes the stream to flip into TERC4 mode
   wire [29:0] s_data_x = {red[9:5], green[9:5], blue[9:5],
			   red[4:0], green[4:0], blue[4:0]};
   
   // was bypass_ena in here...
   wire [29:0] s_data = !box_active_q[4] ? byp_sd5 : s_data_x;

   always @(posedge pclk or posedge rstin) begin
      if( rstin ) begin
	 byp_error <= 1'b0;
      end else begin
	 byp_error <= byp_sd5 != s_data_x;
      end
   end
   
  convert_30to15_fifo pixel2x (
    .rst     (rstin),
    .clk     (pclk),
    .clkx2   (pclkx2),
    .datain  (s_data),
    .dataout ({tmds_data2, tmds_data1, tmds_data0}));

endmodule
