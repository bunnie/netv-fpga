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
// Notes
//  1280 x 720p60  = 74.25 MHz pclk; 45 kHz horiz rate, 750 vertical
//  1920 x 1080p24 = 74.25 MHz pclk; 27k Hz horiz rate, 1125 vertical
//////////////////////////////////////////////////////////////////////////////

`define ALPHA_ON // turn on encrypted alpha blending

// removes I2C control elements when this is commented
`define PRODUCTION   // comment out for simulation, leave in for production

// removes Rx device when this is commented, self-only mode is possible
// also, uncomment the CLOCK_DEDICATED_ROUTE line inside the .ucf file
// and also comment out the TIG paths as noted
`define PASSTHROUGH 1

// enable SS clocking to the LCD interface to reduce noise (valid in both pass-thru and self-timed mode)
`define SS_CLOCKING_TOP 1

// uncomment the below to enable pre version-8 timing detector behavior, i.e. detect timing
// of the LCD stream during self-timed mode, instead of the Rx stream
//`define TIMING_POSTX 1

`timescale 1 ns / 1 ps

module hdmi_overlay (
  input wire        rstbtn_n,    
  input wire        clk26,      //26 mhz oscillator
  input wire [3:0]  RX0_TMDS,
  input wire [3:0]  RX0_TMDSB,

  output wire [3:0] TX0_TMDS,
  output wire [3:0] TX0_TMDSB,

  input wire [7:2]  LCD_R,
  input wire [7:2]  LCD_G,
  input wire [7:2]  LCD_B,
  output wire       LCDO_DCLK, // changed to output...
  input wire        LCD_DE,
  input wire        LCD_HSYNC,
  input wire        LCD_VSYNC,

  output wire       VSYNC_STB,

  output reg        LED0,
  output wire       LED1,
  inout  wire       SDA,
  input  wire       SCL,
  inout  wire       DDC_SDA,
  output wire       DDC_SDA_PU,
  output wire       DDC_SDA_PD,
  input  wire       DDC_SCL,
  input  wire       HPD_N,
  output wire       HPD_NOTIFY,
  output wire       HPD_OVERRIDE,
  output wire       SOURCE_NOTIFY,

  input wire        CEC,  // just a placeholder for CEC for now
  input wire        CHUMBY_BEND, // to prevent this from being tied-off
		     
  input wire        LOWVOLT_N,
  output reg        LOWVOLT_NOTIFY, // tell the CPU that we have a low voltage condition GPIO 93
  output reg        HDCP_AKSV       // tell the CPU that AKSV was writ, so generate Km...GPIO 92
);

//   wire 	    LCD_DCLK_intbuf;
   
   wire box_active; 
   wire box_active_raw;
   wire chroma_match;
         
   wire [7:0] blend_b;
   wire [7:0] blend_r;
   wire [7:0] blend_g;

   wire [7:0] chroma_r;
   wire [7:0] chroma_g;
   wire [7:0] chroma_b;
   
   wire [23:0] cipher_stream;

   wire        clk26buf;

   // extend sync pulses and detect edge
   reg 	      vsync_v;
   reg 	      hsync_v;
   reg 	      hsync_v2;
   reg 	      vsync_v2;
   wire       hsync_rising;
   wire       vsync_rising;

   // autodetect sync polarity
   reg 	hdmi_vsync_pol;
   reg 	hdmi_hsync_pol;

   // compute HPD relay to host (delay one vsync)
   reg 	       hpd_saw_vsync;

   // try to make it so that hdcp isn't used if it isn't ready
   wire        hdcp_comp_ready;
   wire        hdcp_is_ready;

   // timing derivation interface
   wire [11:0] t_hactive; // in pixels
   wire [11:0] t_vactive; // in lines 
   wire [11:0] t_htotal; // in pixels 
   wire [23:0] t_vtotal; // ** in PIXELS ** must divide by htotal in software
   wire [7:0]  t_h_fp; // in pixels 
   wire [7:0]  t_h_bp;  // in pixels 
   wire [23:0] t_v_fp; // ** in PIXELS **
   wire [23:0] t_v_bp;  // ** in PIXELS **
   wire [7:0]  t_hsync_width; // in pixels 
   wire [23:0] t_vsync_width; // ** in PIXELS **
   wire [11:0] t_lcd_de_latency; // in lines
   wire [11:0] t_vsync_latency; // in lines
   wire [23:0] t_refclkcnt; // number of refclocks in a field

   // break self-mode toggle to a wire so we force it in the self-mode configuration
   reg 	       self_mode;

   // note if HDCP was requested on a frame
   reg 	       hdcp_requested;

   // de type selection
   wire        de_sync;
   wire        use_basic_de;

   // device DNA -- state machine at bottom
   reg [55:0] 		   dna_data;

   // lock state of PLLs and channels
   wire ss_locked;
   wire tx0_plllckd;
   wire rx0_plllckd;
   wire rx0_psalgnerr;      // channel phase alignment error
  wire rx0_blue_vld;
  wire rx0_green_vld;
  wire rx0_red_vld;
  wire rx0_blue_rdy;
  wire rx0_green_rdy;
  wire rx0_red_rdy;
   wire m720p_locked;
   
   ////////// I2C host interface ////////
   wire SDA_pd;
   wire [7:0] reg_addr;
   wire       wr_stb;
   wire [7:0] reg_data_in;
   wire [7:0] reg_a2;
   wire       SDA_int;
   wire [7:0] snoop_ctl;
   wire [7:0] snoop_rbk_adr;
   wire [7:0] snoop_rbk_dat;

   wire [7:0] hdcp_snoop_data;
   wire [7:0] hdcp_snoop_addr;
   wire [7:0] edid_snoop_data;
   wire [7:0] edid_snoop_addr;
   
   wire [7:0] comp_ctl;
   wire [15:0] window_w;
   wire [14:0] window_h;
   wire        window_update;
   wire [15:0] window_x;
   wire [15:0] window_y;
   reg [11:0]  window_x_buf;
   reg [11:0]  window_y_buf;
   reg [11:0]  window_w_buf;
   reg [11:0]  window_h_buf;
   
   wire [7:0] ext1_ctl;

   wire [55:0] Km;
   wire [63:0] An;
   wire        Aksv14_write;
   reg         Km_rdy0;
   reg         Km_rdy1;
   reg 	       Km_rdy2;
   wire        Km_ready;
   wire [7:0]  edid_daddr;

   wire        rst_skewmach;

   wire [3:0]  line_full_level;
   wire [3:0]  line_empty_level;
   wire [3:0]  write_init_level;
   wire [3:0]  read_init_level;

   wire [23:0] target_lead_pixels;
   wire        reset_lock_machine;
   wire        genlock_locked;
   wire [15:0] lock_tolerance;
   wire        smartlock_on;
   
   wire        modeline_write;
   wire [7:0]  modeline_adr;
   wire [7:0]  modeline_dat;

   // alpha blending
   wire [2:0]  alpha_vid;
   wire [2:0]  alpha_lcd;
   wire        alpha_en;
      
   wire        rx_all_valid;
   assign rx_all_valid = (rx0_blue_vld & rx0_green_vld & rx0_red_vld);

   reg [7:0]  reg_data_holding;
   reg 	      wr_stb_d;
   always @(posedge clk26buf) begin
      wr_stb_d <= wr_stb;
      if (wr_stb & !wr_stb_d) begin // only act on the rising pulse of wr_stb
	 reg_data_holding <= reg_data_in;
      end else begin
	 reg_data_holding <= reg_data_holding;
      end
   end
   
`ifdef PRODUCTION   
   i2c_slave host_i2c(
		      .SCL(SCL),
		      .SDA(SDA_int),
		      .SDA_pd(SDA_pd),

		      .clk(clk26buf),
		      .glbl_reset(~rstbtn_n),

		      .i2c_device_addr(8'h3C),
//		      .reg_addr(reg_addr),
//		      .reg_data_in(reg_data_in),
//		      .reg_data_in(reg_data_holding),
//		      .wr_stb(wr_stb),
		      .reg_0(snoop_ctl),
		      .reg_1(snoop_rbk_adr),
		      .reg_2(reg_data_holding),
		      .reg_3(comp_ctl),
//`define REGWINDOWS
`ifdef REGWINDOWS
		      .reg_4(window_w[7:0]),
		      .reg_5(window_w[15:8]),
		      .reg_6(window_h[7:0]),
		      .reg_7({window_update,window_h[14:8]}),
		      .reg_8(window_x[7:0]),
		      .reg_9(window_x[15:8]),
		      .reg_a(window_y[7:0]),
		      .reg_b(window_y[15:8]),
`endif

		      .reg_8(8'h34),  // just a placeholder for now
// debug HDCP		      
//		      .reg_9({6'b0,HDCP_cstate[17:16]}),
//		      .reg_a(HDCP_cstate[15:8]),
//		      .reg_b(HDCP_cstate[7:0]),
// debug HDCP
		      .reg_9(8'h0D),  // placeholders
		      .reg_a(8'hBA),
		      .reg_b(8'hBE),
		      
		      .reg_c(ext1_ctl),
		      
		      // hard coded now
//		      .reg_d({line_full_level[3:0],line_empty_level[3:0]}),
//		      .reg_e({write_init_level[3:0], read_init_level[3:0]}),

		      // hard-wired to 240, 0, 240
//		      .reg_d(chroma_r),
//		      .reg_e(chroma_g),
//		      .reg_f(chroma_b),
		      
		      .reg_10({hdcp_requested,hdmi_vsync_pol,hdmi_hsync_pol,LOWVOLT_NOTIFY,1'b0,CEC,
			       genlock_locked, CHUMBY_BEND}),


		      .reg_11(lock_tolerance[7:0]),
		      .reg_12(lock_tolerance[15:8]),
		      
		      .reg_13({modeline_write,modeline_adr[6:0]}),
		      .reg_14(modeline_dat),

		      .reg_15(target_lead_pixels[7:0]),
		      .reg_16(target_lead_pixels[15:8]),
		      .reg_17(target_lead_pixels[23:16]),

//		      .reg_18(edid_daddr[7:0]),
		      .reg_18({rx_all_valid,
			       rx0_blue_rdy, rx0_green_rdy, rx0_red_rdy,
			       rx0_psalgnerr,
			       m720p_locked, tx0_plllckd, rx0_plllckd}),
			       
		      .reg_19(Km[7:0]),
		      .reg_1a(Km[15:8]),
		      .reg_1b(Km[23:16]),
		      .reg_1c(Km[31:24]),
		      .reg_1d(Km[39:32]),
		      .reg_1e(Km[47:40]),
		      .reg_1f(Km[55:48]),

		      //// read-only registers after this point
		      .reg_20(t_hactive[7:0]),
		      .reg_21({4'b0,t_hactive[11:8]}),
		      .reg_22(t_vactive[7:0]),
		      .reg_23({4'b0,t_vactive[11:8]}),
		      .reg_24(t_htotal[7:0]),
		      .reg_25({4'b0,t_htotal[11:8]}),
		      .reg_26(t_vtotal[7:0]),
		      .reg_27(t_vtotal[15:8]),
		      .reg_28(t_vtotal[23:16]),
		      .reg_29(t_h_fp[7:0]),
		      .reg_2a(t_h_bp[7:0]),
		      .reg_2b(t_v_fp[7:0]),
		      .reg_2c(t_v_fp[15:8]),
		      .reg_2d(t_v_fp[23:16]),
		      .reg_2e(t_v_bp[7:0]),
		      .reg_2f(t_v_bp[15:8]),
		      .reg_30(t_v_bp[23:16]),
		      .reg_31(t_hsync_width[7:0]),
		      .reg_32(t_vsync_width[7:0]),
		      .reg_33(t_vsync_width[15:8]),
		      .reg_34(t_vsync_width[23:16]),
		      .reg_35(t_refclkcnt[7:0]),
		      .reg_36(t_refclkcnt[15:8]),
		      .reg_37(t_refclkcnt[23:16]),
//		      .reg_38(t_lcd_de_latency[7:0]),
//		      .reg_39({4'b0,t_lcd_de_latency[11:8]}),
//		      .reg_3a(t_vsync_latency[7:0]),
//		      .reg_3b({4'b0,t_vsync_latency[11:8]}),

		      .reg_38(dna_data[7:0]),
		      .reg_39(dna_data[15:8]),
		      .reg_3a(dna_data[23:16]),
		      .reg_3b(dna_data[31:24]),
		      .reg_3c(dna_data[39:32]),
		      .reg_3d(dna_data[47:40]),
		      .reg_3e(dna_data[55:48]),

		      .reg_3f(8'h11)    // version number
		      );
   /////// version 4 changes
   // - added input registers to LCD path to clean up timing
   // - added a pipeline stage to the core video processing pipe
   // - adjusted the position of chroma decision versus chroma to remove right pink stripe
   // - fixed chroma to 240, 0, 240 to reduce computational complexity
   // - fixed timing files to get better coverage of the FPGA
   // - inverted clock to device DNA state machine to fix hold time race condition
   // - self-timed mode is now native, no need to switch FPGA configs
   // - added PLL and alignment/valid feedback registers to detect when source is present
   // - touch-up clock tree to improve clarity of timing definition & rule propagation
   // - full switch-over to PlanAhead tool for compilation

   /////// version 5 changes (log created 8/11/2011)
   // - changed blue LED from flashing to breathing

   /////// version 6 changes (log created 8/12/2011)
   // - added off state to LED which is automatically triggered when output not plugged in
   
   /////// version 7 changes (log created 8/12/2011)
   // - added SOURCE_NOTIFY reporting trigger
   // - removed HPD debounce circuit, so HPD reports even when no source is present

   /////// version 8 changes (log cerated 8/21/2011)
   // - changed timing detector to always report the timing of the Rx stream, even in self-timed mode

   /////// version 9 changes (log created 8/22/2011)
   // - removed setbox functionality. Box is always "full screen". 
   // Registers 4-B are now deprecated (they are NOPs if written in this implementation, may
   // change to be active later).

   /////// version A changes (log created 8/22/2011)
   // - HDCP cipher now resets properly when Ksv is double-initialized
   // - EDID snooper for HDCP now limits read bounds to 5 to compensate
   //   for tivo series 3 weirdness.

   /////// version B changes (log created 8/27/2011)
   // - timing closure only

   /////// version C changes (log created 8/27/2011)
   // - fix chroma issue in overlay mode, was checking too many bits, causing jagged edges on photos
   
   /////// version D changes (log created 9/5/2011)
   // - add workaround for Apple TV 2 EESS bug. ATV2 asserts EESS by far too early. Added a trap to catch
   //   EESS when it is asserted too early relative to vsync

   /////// version E changes (log created 9/29/2011)
   // - fix RGB color depth issue; turns out that extending the LSB's isn't the right way to do it.
   //   now, we truncate the unused bits to zero

   /////// version F changes (log create 12/13/2011)
   // - add encrypted alpha blending - partially working
   // - optimize reset condition of i2c engine to reduce logic area of i2c; if this works, in a
   //   future version we should strip out more superfluous / problematic reset logic
   // - fix typo in DNA cstate synthesis case directives

   /////// version 10 changes (log created 3/7/2012)
   // - fix issue with EDID squashing where first byte would go unsquashed on repeated start conditions
   //   (first identified with motorola DCH3200)
   // - fix issue with alpha blending being off (was picking wrong path to bypass)
   
   assign SOURCE_NOTIFY = rx0_plllckd;		     
   
   // hard wire chroma because it never changes, optimize out some gates
   assign chroma_r = 8'd240;
   assign chroma_g = 8'd0;
   assign chroma_b = 8'd240;

   assign modeline_adr[7] = snoop_ctl[6]; // hack to add another bank into the modeline RAM
//   assign use_basic_de = ext1_ctl[0];

   // these are hard-coded now, used to be registers
   assign line_full_level[3:0] = 4'h2;
   assign line_empty_level[3:0] = 4'h2;
   assign write_init_level[3:0] = 4'h1;
   assign read_init_level[3:0] = 4'h2;

   assign alpha_vid = ext1_ctl[7:5];
   assign alpha_lcd = ~alpha_vid;
   // 000 001 010 011 100 101 110 111
   // 111 110 101 100 011 010 001 000
   assign alpha_en = ext1_ctl[4];
   
`else // !`ifdef PRODUCTION
   assign comp_ctl = 8'h09; // setup for testing stand-alone mode
   assign snoop_ctl = 8'h0;
   assign snoop_ctl = 8'h1;
`endif // !`ifdef PRODUCTION

//   IBUF IBUF_sda (.I(SDA), .O(SDA_int));
   IOBUF #(.DRIVE(12), .SLEW("SLOW")) IOBUF_sda (.IO(SDA), .I(1'b0), .T(!SDA_pd), .O(SDA_int));

   //// I2C internal control wiring ////
   /////////////////
   /// register 0: control snoop state (SNOOP_CTL r/w)
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //         | MODEBNK |         |         | HPD_FRC | SQUASH  |  RD_STB |  RD_HDCP
   //
   //  bit 0 - RD_HDCP. When 1, select readback of the HDCP set; when 0, select EDID set
   //  bit 1 - RD_STB. When high, update the contents of SNOOP_DAT with data at SNOOP_ADR
   //  bit 2 - enable EDID squashing
   //  bit 3 - when high, force HPD to show that nothing is plugged in; low, act as normal
   //  bit 6 - sets the bank to write with register 0x13 (yah yah this is a hack)
   //
   /////////////////
   /// register 1: snoop readback address (SNOOP_ADR r/w)
   //  bits 7:0 are the address to read back  from the snoop unit
   //
   /////////////////
   /// register 2: snoop readback data (SNOOP_DAT ro)
   //  bits 7:0 are the data corresponding to the last loaded snoop address as
   //     selected by RD_HDCP bits in SNOOP_CTL when RD_STB was last toggled
   //
   //  REVISION -- now dynamically relays the value specified by snoop_adr without having
   //     to toggle RD_STB. RD_STB has no effect currently.
   /////////////////
   //  register 3: Compositing control (COMP_CTL r/w)
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  KM_SEM | RST_GNLK| SMRTLCK | RST_PLL |  SELF   | COMP_ON |  KM_RDY |  HDCP_ON
   //
   //  bit 0 - enable HDCP encryption of the composited stream. Enables HDCP encryption
   //          only if the HDCP cipher is used. If the input stream has no HDCP on it
   //          then this bit has no meaning.
   //  bit 1 - indicates that Km is ready and loaded (currently ignored)
   //  bit 2 - enable compositing of incoming data from LCD port to HDMI stream
   //  bit 3 - when set, ignore incoming data and generate sync based on LCD port signals
   //  bit 4 - when set, resets PLLs only on the paths designated by bit3 ("self")
   //  bit 5 - when set, enable "smart locking", i.e., genlock turns off once we're locked
   //  bit 6 - reset the genlock control machine
   //  bit 7 - Km semaphore -- used to indicate to the kernel whether an existing process
   //          is controlling the Km derivation process. This is to resolve the issue where
   //          the HPD will generate two events to cover Km generation in the case that
   //          the final protocol requires a "restart" to get the correct Km value to stick
   //
   /////////////////

`ifdef REGWINDOWS
   ///////////// REGISTER 4 - B are now DEPRECATED.
   //  register 4: window width
   //  bits 7:0 are LSB of window width
   //
   /////////////////
   //  register 5: window width
   //  bits 7:0 are MSB of window width
   //
   /////////////////
   //  register 6: window height
   //  bits 7:0 are LSB of window height
   //
   /////////////////
   //  register 7: window height
   //  bits 6:0 are MSB of window height
   //  bit 7 is the "update" bit which informs the system to take the new values on the next
   //    vsync period
   //
   /////////////////
   //  register 8: window X position
   //  bits 7:0 are LSB of window X position
   //
   /////////////////
   //  register 9: window X position
   //  bits 7:0 are MSB of window X position
   //
   /////////////////
   //  register A: window Y position
   //  bits 7:0 are LSB of window Y position
   //
   /////////////////
   //  register B: window Y position
   //  bits 7:0 are MSB of window Y position
   //
   /////////////////////////// END DEPRACATION BLOCK
`endif

   /////////////////
   //  register 8-B: read-only debug registers, meaning reserved
   //
   
   /////////////////
   //  register C: extended control set (EXT1_CTL r/w)
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  ALPH2  | ALPH1   | ALPH0   |  ALPHEN |         |         | CHROMA  |
   //
   //  bit 7-5: alpha value, 3 bits.
   //  bit 4: alpha blending enable
   //  bit 1: when set, turn on chroma keying; otherwise, always blend in
   //
   /////////////////
   //  NOTE: these are now hard-wired to 240, 0, 240. These registers will likely be deprecated soon.
   //  register D: chroma R value, 8 bits
   //  register E: chroma G value, 8 bits
   //  register F: chroma B value, 8 bits
   //
   /////////////////
   //  register 0x10 is read-only:
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  HDCPDET| VSYNCPOL| HSYNCPOL| LOWVOLT |         |  CEC    | LOCKED  |  BEND
   //  bit 0: chumby_bend pin state (tied off to insure configuratios as an input)
   //  bit 1: indicates that the genlock machine has locked LCD to HDMI streams
   //  bit 2: CEC pin state (tied off to insure configuratios as an input)
   //  bit 4: when high indicates that a low voltage condition was detected; only active during condition
   //         there is also an interrupt to the CPU that fires
   //  bit 5: indicates the polarity of the HSYNC detected on the HDMI stream, 1 = active high
   //  bit 6: indicates the polarity of the VSYNC detected on the HDMI stream, 1 = active high
   //  bit 7: when high, indicates that an HDCP stream is being encrypted. Not active during
   //         horiz and vert sync periods.
   //
   /////////////////
   //  register 0x11-12:
   //    lock tolerance, in pixels, in little-endian byte order
   //    This defines the tolerance of the "lock" threshold. This value matters mostly when
   //    "smart locking" is turned on, i.e., when we want to disable genlock once we're within
   //    our locking tolerance window.
   //
   /////////////////
   //  register 0x13 is address of modeline RAM to write
   //  7 is a write strobe (write when high)
   //  6:0 is the actual modeline address
   //
   /////////////////
   //  register 0x14 is the data to write into the modeline RAM
   //  7:0 is the data
   //
   /////////////////
   //  registers 0x15-0x17:
   //    lock target count, in pixels, in little-endian byte order.
   //    Lock target count is the amount of time that the LCD interface should lead the
   //    HDMI inteface for producing data. The amount of time should be large enough to
   //    absorb timing variations in the interrupt latency handling of the PXA168, but
   //    in all cases smaller than 8 lines of video (that's the size of the line buffers).
   //
   //    The total time should be expressed in units of pixels, not lines.
   //
   /////////////////
   //  register 0x18 is read-only:
   //  bit 7  |  bit 6  |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |  bit 0
   //  RX_VLD |  B_RDY  |  G_RDY  |  R_RDY  | ALGNERR |  SYNLCK | TXLCK   |  RXLCK
   //  bit 0: rx PLL is locked, indicates that a clock is present and cable is plugged in
   //  bit 1: tx PLL is locked. This should generally always be the case.
   //  bit 2: synthesizer DCM is locked. This should generally always be the case.
   //  bit 3: rx alignment error
   //  bit 4: red channel has received a valid pixel
   //  bit 5: green chanel has received a valid pixel
   //  bit 6: blue channel has received a valid pixel
   //  bit 7: all RX chanels are phase aligned and producing valid data
   //   
   /////////////////
   //  registers 0x19-0x1f: Km
   //  56-bit value of Km, entered in little-endian order.
   //
   /////////////////
   //  All registers after this point are read-only, and byte-order is little-endian
   /////////////////
   //  register 0x20-21: horizontal active in pixels
   //  register 0x22-23: vertical active in lines
   //  register 0x24-25: horizontal total width in pixels
   //  register 0x26-28: vertical total height in pixels (not lines)
   //  register 0x29: horizontal front porch in pixels
   //  register 0x2a: horizontal back porch in pixels
   //  register 0x2b-2d: vertical front porch in pixels (not lines)
   //  register 0x2e-30: vertical back porch in pixels (not lines)
   //  register 0x31: horizontal sync width in pixels
   //  register 0x32-0x34: vertical sync width in pixels (not lines)
   //  register 0x35-0x37: reference clock count cycles
   //
   //  register 0x38-0x3e: device ID (7 bytes)
   //
   //  register 0x3f: version number
   

   //////////// DEPRACATED REGISTERS ////////////// (but not confident they are dead yet)
   //  register D: line empty/full levels
   //  This sets the level at which we declare the line buffers to be "empty" or "full"
   //  This is on a the granularity of a full video line, not at the pixel level. The
   //  pixel-level full/empty is hard-coded by the FIFO primitive.
   //  bit 7 is ignored
   //  bits [6:4] are the full level target: nominally 2
   //  bit 3 is ignored
   //  bits [2:0] are the empty level target: nominally 2
   //
   /////////////////
   //  register E: write and read initial levels
   //  This sets the initial state of the write/read pointers, reset on every vsync.
   //  Note that the actual bit vector that keeps track of active lines can be much longer
   //  than 4 bits, but we only get to diddle with the bottom four bits in this implementation.
   //  bits [7:4] are for the write: nominally 1
   //  bits [3:0] are for the read: nominally 2
   //
   /////////////////
   //  register F: reserved
   //

`ifdef PRODUCTION   
`define EDID_SNOOP 1  // turn on EDID snooping
   
   assign reg_addr = 8'h02;
   assign wr_stb = snoop_ctl[1];
`ifdef EDID_SNOOP
   assign reg_data_in[7:0] = snoop_ctl[0] ? hdcp_snoop_data[7:0] : edid_snoop_data[7:0];
   assign edid_snoop_addr = snoop_rbk_adr;
`else
   assign reg_data_in[7:0] = hdcp_snoop_data[7:0];
`endif
   assign hdcp_snoop_addr = snoop_rbk_adr;
   
   /////// DDC snooping interface //////
   wire        DDC_SDA_int;
   wire        DDC_SDA_pu_int;
   wire        DDC_SDA_pd_int;
   i2c_snoop ddc_hdcp_snoop (
		  .SCL(!DDC_SCL),      // inverters on inputs...
		  .SDA(!DDC_SDA_int), 
		  .clk(clk26buf), 
		  .reset(~rstbtn_n),
			     
		  .i2c_snoop_addr(8'h74),  // HDCP address *only* is snooped by this one
			     
		  .reg_addr(hdcp_snoop_addr), 
		  .reg_dout(hdcp_snoop_data),

		  .An(An),
		  .Aksv14_write(Aksv14_write)
		  );
`ifdef EDID_SNOOP
   /////// EDID snooping interface //////
   i2c_snoop_edid ddc_edid_snoop (
		  .SCL(!DDC_SCL), 
		  .SDA(!DDC_SDA_int), 
		  .clk(clk26buf), 
		  .reset(~rstbtn_n),
			     
		  .i2c_snoop_addr(8'h74),  // all *but* this address is snooped by this one
			     
		  .reg_addr(edid_snoop_addr), 
		  .reg_dout(edid_snoop_data)
		  );

   i2c_squash_edid ddc_edid_squash (
		  .SCL(!DDC_SCL), 
		  .SDA(!DDC_SDA_int), 
		  .clk(clk26buf), 
		  .reset(~rstbtn_n),

		  .SDA_pu(DDC_SDA_pu_int),
		  .SDA_pd(DDC_SDA_pd_int),
			     
		  .i2c_snoop_addr(8'ha0),  // only reads from this addres can be squashed
				    
		  .modeline_write(modeline_write),
		  .modeline_dat(modeline_dat),
		  .modeline_adr(modeline_adr)
		  );
//   IOBUF #(.DRIVE(24), .SLEW("SLOW")) IOBUF_DDC_sda (.IO(DDC_SDA), .I(DDC_SDA_pu), 
//						     .T(!(DDC_SDA_pd | DDC_SDA_pu) | !snoop_ctl[2]),
//						     .O(DDC_SDA_int));

   // and with inverse of SDA_PD to ensure we never get overlap of pullup/pulldown and thus DISASTER
   assign DDC_SDA_PU = (DDC_SDA_pu_int & snoop_ctl[2]) & !DDC_SDA_pd_int;
   assign DDC_SDA_PD = DDC_SDA_pd_int & snoop_ctl[2];

   IBUF IBUF_DDC_sda (.I(DDC_SDA), .O(DDC_SDA_int));
   
`endif
`endif
      
  ////////////////////////////////////////////////////
  // utility clock buffer 
  ////////////////////////////////////////////////////
//  BUFIO2 #(.DIVIDE_BYPASS("FALSE"), .DIVIDE(5))
//  sysclk_div (.DIVCLK(clk26m), .IOCLK(), .SERDESSTROBE(), .I(clk26));
   wire        clk26_ibuf;
   IBUFG clk26buf_ibuf(.I(clk26), .O(clk26ibuf));
   BUFG clk26buf_buf (.I(clk26ibuf), .O(clk26buf));
//   BUFG clk26buf_bufx(.I(clk26ibuf), .O(clk26bufpll));
   
   wire       tx0_pclk;  // tx clock got promoted above input port because 
   // input port is compiled out in some cases

  /////////////////////////
  //
  // Input Port 0
  //
  /////////////////////////
  wire rx0_pclk, rx0_pclkx2, rx0_pclkx10, rx0_pllclk0;
  wire rx0_reset;
  wire rx0_serdesstrobe;
  wire rx0_hsync;          // hsync data
  wire rx0_vsync;          // vsync data
  wire rx0_de;             // data enable
   wire rx0_basic_de;
  wire [7:0] rx0_red;      // pixel data out
  wire [7:0] rx0_green;    // pixel data out
  wire [7:0] rx0_blue;     // pixel data out
  wire [29:0] rx0_sdata;
  wire rx0_cv;

   wire rx0_encoding;
   wire rx0_hdcp_ena;
   wire [3:0] rx0_red_di;
   wire [3:0] rx0_blue_di;
   wire [3:0] rx0_green_di;
   wire       rx0_data_gb;
   wire       rx0_video_gb;
   wire [3:0] rx0_ctl_code;

   wire       rx0_line_end;

`ifdef PASSTHROUGH
  dvi_decoder dvi_rx0 (
    //These are input ports
    .tmdsclk_p   (RX0_TMDS[3]),
    .tmdsclk_n   (RX0_TMDSB[3]),
    .blue_p      (RX0_TMDS[0]),
    .green_p     (RX0_TMDS[1]),
    .red_p       (RX0_TMDS[2]),
    .blue_n      (RX0_TMDSB[0]),
    .green_n     (RX0_TMDSB[1]),
    .red_n       (RX0_TMDSB[2]),
    .exrst       (~rstbtn_n || HPD_N), 
    .pllreset    ((comp_ctl[4] && !self_mode)),

    //These are output ports
    .reset       (rx0_reset),
    .pclk        (rx0_pclk),
    .pclkx2      (rx0_pclkx2),
    .pclkx10     (rx0_pclkx10),
    .pllclk0     (rx0_pllclk0), // PLL x10 output
    .pllclk1     (rx0_pllclk1), // PLL x1 output
    .pllclk2     (rx0_pllclk2), // PLL x2 output
    .pll_lckd    (rx0_plllckd),
    .tmdsclk     (rx0_tmdsclk),
    .serdesstrobe(rx0_serdesstrobe),
    .hsync       (rx0_hsync),
    .vsync       (rx0_vsync),
    .de          (rx0_de),
    .basic_de    (rx0_basic_de),

    .blue_vld    (rx0_blue_vld),
    .green_vld   (rx0_green_vld),
    .red_vld     (rx0_red_vld),
    .blue_rdy    (rx0_blue_rdy),
    .green_rdy   (rx0_green_rdy),
    .red_rdy     (rx0_red_rdy),

    .psalgnerr   (rx0_psalgnerr),

    .sdout       (rx0_sdata),
    .red         (rx0_red),
    .green       (rx0_green),
    .blue        (rx0_blue),

    .encoding    (rx0_encoding),
    .hdcp_ena    (rx0_hdcp_ena),
    .red_di      (rx0_red_di),
    .green_di    (rx0_green_di),
    .blue_di     (rx0_blue_di),
    .data_gb     (rx0_data_gb),
    .video_gb    (rx0_video_gb),
    .ctl_code    (rx0_ctl_code),
    .cv          (rx0_cv),
    .line_end    (rx0_line_end)
		       ); 

   //
   // This BUFG mirrors the rx0 clock
   // This way we have a matched skew between the RX pclk clocks and the TX pclk
   //
//   BUFG tx0_bufg_pclk (.I(self_mode ? tx0_pclk_self : rx0_pllclk1), .O(tx0_pclk)); // clock comes from rx is passthru mode
   wire tx0_pclk_ss;
   wire tx0_pclk_ss_unbuf;

//   assign self_mode = comp_ctl[3];
   reg 	self_mode_pre;
   always @(posedge tx0_pclk_self) begin
      self_mode_pre <= comp_ctl[3];
      self_mode <= self_mode_pre;
   end

   wire tx0_pclk_todcm;
   BUFGMUX tx0_bufg_mux    (.I0(rx0_pllclk1), .I1(tx0_pclk_self), .S(self_mode_pre), .O(tx0_pclk));
   BUFGMUX tx0_bufg_muxdcm (.I0(rx0_pllclk1), .I1(tx0_pclk_self), .S(self_mode_pre), .O(tx0_pclk_todcm));
   
//   clkgendcm_720p60hz selfclockgen (.CLK_IN1(clk26buf), .CLK_OUT1(tx0_pclk_ss), .RESET((comp_ctl[4] && self_mode)), .LOCKED(m720p_locked) );
   clkgendcm_720p60hz selfclockgen (.CLK_IN1(clk26buf), .CLK_OUT1(tx0_pclk_self), .RESET((comp_ctl[4] && self_mode)), .LOCKED(m720p_locked) );

/*   
   // use spread spectrum clocking to reduce emissions...
   DCM_CLKGEN    #(
		   .DFS_OSCILLATOR_MODE("PHASE_FREQ_LOCK"), 
		   .CLKIN_PERIOD ("13.48"),
		   .SPREAD_SPECTRUM ("CENTER_LOW_SPREAD"),
// 		   .CLKFX_MD_MAX("1.0"),
 		   .CLKFX_MULTIPLY	(4),
 		   .CLKFX_DIVIDE	(4) )
   dcm_fcc_spreads_me_wide (
		     .CLKIN   	(tx0_pclk_ss),
		     .FREEZEDCM (1'b0),
		     .PROGDATA		(1'b0),
		     .PROGEN 		(1'b0),
		     .PROGCLK		(1'b0), 
//		     .PROGDONE		(1'b0),
//		     .CLKFX180	(CLKFX180_DCM),
//		     .CLKFXDV	(CLKFXDV_DCM),
//		     .LOCKED  	(clkgen_locked),
		     .RST     	(self_mode || comp_ctl[4]),
		     .CLKFX   	(tx0_pclk_ss_unbuf),
		     .LOCKED(ss_locked)
			    );
   BUFG tmdsclk_bfgss (.I(tx0_pclk_ss_unbuf), .O(tx0_pclk_self) );
 */

`else // !`ifdef PASSTHROUGH
   assign self_mode = 1'b1;
   
   // generate internal 720p clock for self-timed mode operation
   wire m720p_clk;

`ifdef SS_CLOCKING_TOP
   wire m720p_locked; // not currently used

   wire progdata_int;
   wire progen_int;
   wire progdone;
   wire clkgen_locked;
   wire tx0_pclk_unbuf;
   
   wire tx0_pclk_ss;
   wire tx0_pclk_ss_unbuf;

   clk_720p_60hz clk720p (.CLK_IN1(clk26bufpll), .CLK_OUT1(tx0_pclk_ss), .RESET((comp_ctl[4] && self_mode)), 
			  .LOCKED(m720p_locked) );

   // use spread spectrum clocking to reduce emissions...
   DCM_CLKGEN    #(
		   .DFS_OSCILLATOR_MODE("PHASE_FREQ_LOCK"), 
		   .CLKIN_PERIOD ("13.48"),
		   .SPREAD_SPECTRUM ("CENTER_HIGH_SPREAD"),
// 		   .CLKFX_MD_MAX("1.0"),
 		   .CLKFX_MULTIPLY	(4),
 		   .CLKFX_DIVIDE	(4) )
   dcm_fcc_spreads_me_wide (
		     .CLKIN   	(tx0_pclk_ss),
		     .FREEZEDCM (1'b0),
		     .PROGDATA		(1'b0),
		     .PROGEN 		(1'b0),
		     .PROGCLK		(1'b0), 
//		     .PROGDONE		(1'b0),
//		     .CLKFX180	(CLKFX180_DCM),
//		     .CLKFXDV	(CLKFXDV_DCM),
//		     .LOCKED  	(clkgen_locked),
		     .RST     	(comp_ctl[4]),
		     .CLKFX   	(tx0_pclk_ss_unbuf),
		     .LOCKED(ss_locked)
			    );
   BUFG tmdsclk_bfgss (.I(tx0_pclk_ss_unbuf), .O(tx0_pclk) ); //
   
	
`else // !`ifdef SS_CLOCKING_TOP

   assign ss_locked = 1'b1;
   
   clk_720p_60hz clk720p (.CLK_IN1(clk26bufpll), .CLK_OUT1(tx0_pclk), .RESET((comp_ctl[4] && self_mode)), 
			  .LOCKED(m720p_locked) );
   
   //  BUFG tx0_bufg_pclk (.I(self_mode ? LCD_DCLK_intbuf : rx0_pllclk1), .O(tx0_pclk));
   //   BUFG tx0_bufg_pclk (.I(m720p_clk), .O(tx0_pclk)); // clock comes from freqsynth if internal
`endif // !`ifdef SS_CLOCKING_TOP

   ///// dummy tie-offs so we don't need a separate .UCF for the self-timed case
   wire [3:0] dummy_tmds;
   
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy0 (.I(RX0_TMDS[0]), .IB(RX0_TMDSB[0]), .O(dummy_tmds[0]));
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy1 (.I(RX0_TMDS[1]), .IB(RX0_TMDSB[1]), .O(dummy_tmds[1]));
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy2 (.I(RX0_TMDS[2]), .IB(RX0_TMDSB[2]), .O(dummy_tmds[2]));
   IBUFDS  #(.IOSTANDARD("TMDS_33"), .DIFF_TERM("FALSE") 
	     ) ibuf_dummy3 (.I(RX0_TMDS[3]), .IB(RX0_TMDSB[3]), .O(dummy_tmds[3]));

`endif // !`ifdef PASSTHROUGH
   
   // a simple pipeline register for LCD input retiming
   reg 	lcdr_de;
   reg 	lcdr_hsync;
   reg 	lcdr_vsync;
   reg [5:0] lcdr_b;
   reg [5:0] lcdr_g;
   reg [5:0] lcdr_r;
   always @(posedge tx0_pclk) begin
      lcdr_de <= LCD_DE;
      lcdr_hsync <= LCD_HSYNC;
      lcdr_vsync <= LCD_VSYNC;
      lcdr_b <= {LCD_B[7:3], 1'b0};
      lcdr_g <= LCD_G[7:2];
      lcdr_r <= {LCD_R[7:3], 1'b0};
   end

   // this should be safe to have outside the ifdef zone
   wire       chroma_match_self;
   assign chroma_match_self = (({lcdr_b[5:1],3'b0} == chroma_b[7:0]) &&
			      ( {lcdr_r[5:1],3'b0} == chroma_r[7:0]) &&
			      ( {lcdr_g[5:0],2'b0} == chroma_g[7:0])) && ext1_ctl[1];
   
  /////////////////
  //
  // Output Port 0
  //
  /////////////////
  reg          tx0_de;
  reg          tx0_basic_de;
  wire         tx0_pclkx2;
  wire         tx0_pclkx10;
  wire         tx0_serdesstrobe;
  wire         tx0_reset;
  reg  [7:0]   tx0_blue;
  reg  [7:0]   tx0_green;
  reg  [7:0]   tx0_red;
  reg          tx0_hsync;
  reg          tx0_vsync;
  wire         tx0_pll_reset;

   reg         tx0_vid_pa;
   reg         tx0_vid_gb;
   reg         tx0_dat_pa;
   reg         tx0_dat_gb;
   reg         tx0_dat_ena;
   reg [3:0]   tx0_ctl_code;
   reg [9:0]   tx0_dat_din;
   reg  [29:0] tx0_sdata;
   reg         tx0_cv;

   reg 	       tx1_de;
   reg 	       tx1_basic_de;
   reg [7:0]   tx1_blue;
   reg [7:0]   tx1_green;
   reg [7:0]   tx1_red;
   reg 	       tx1_hsync;
   reg 	       tx1_vsync;
   reg         tx1_vid_pa;
   reg         tx1_vid_gb;
   reg         tx1_dat_pa;
   reg         tx1_dat_gb;
   reg         tx1_dat_ena;
   reg [3:0]   tx1_ctl_code;
   reg [9:0]   tx1_dat_din;
   reg  [29:0] tx1_sdata;
   reg         tx1_cv;

   reg 	       tx2_de;
   reg 	       tx2_basic_de;
   reg [7:0]   tx2_blue;
   reg [7:0]   tx2_green;
   reg [7:0]   tx2_red;
   reg 	       tx2_hsync;
   reg 	       tx2_vsync;
   reg         tx2_vid_pa;
   reg         tx2_vid_gb;
   reg         tx2_dat_pa;
   reg         tx2_dat_gb;
   reg         tx2_dat_ena;
   reg [3:0]   tx2_ctl_code;
   reg [9:0]   tx2_dat_din;
   reg  [29:0] tx2_sdata;
   reg         tx2_cv;
   
   wire        computed_gb;
   wire        computed_ctl_code;

   always @ (posedge tx0_pclk) begin
      ///////// earlier pipe stage
      tx0_de <= rx0_de;
      tx0_basic_de <= rx0_basic_de;
      tx0_blue <= rx0_blue;
      tx0_green <= rx0_green;
      tx0_red <= rx0_red;
      
      tx0_hsync <= rx0_hsync;
      tx0_vsync <= rx0_vsync;
      tx0_vid_gb <= rx0_video_gb;
      tx0_vid_pa <= (rx0_ctl_code[3:0] == 4'b0001);
      tx0_dat_gb <= rx0_data_gb;
      tx0_dat_pa <= (rx0_ctl_code[3:0] == 4'b0101);
      tx0_dat_ena <= rx0_encoding;
      
      tx0_ctl_code <= rx0_ctl_code;
      tx0_dat_din[9:0] <= {rx0_blue_di[1:0],rx0_red_di[3:0],rx0_green_di[3:0]};
      tx0_sdata <= rx0_sdata;
      tx0_cv <= rx0_cv;
      
      ///////// later pipe stage
      if( self_mode ) begin
	 // self-timed
	 tx1_de <= lcdr_de; // de is always active high
	 tx1_blue <= chroma_match_self ? 8'b0 : {lcdr_b[5:1],3'b0};
	 tx1_green <= chroma_match_self ? 8'b0 : {lcdr_g[5:0],2'b0};
	 tx1_red <= chroma_match_self ? 8'b0 : {lcdr_r[5:1],3'b0};
	 tx1_hsync <= lcdr_hsync; // sync signals from CPU are always active high
	 tx1_vsync <= lcdr_vsync;
	 
	 //	    tx1_vid_gb <= computed_gb;
	 tx1_vid_gb <= 0;
	 tx1_vid_pa <= 0; // this is a deprecated interface
	 tx1_dat_gb <= 0;
	 tx1_dat_pa <= 0;
	 tx1_dat_ena <= 0;
	 
	 tx1_dat_din[9:0] <= 0;
	 tx1_sdata <= 0;
	 tx1_cv <= 1'b0; // not used
	 //	    tx1_ctl_code <= computed_ctl_code;  // this is the real pa (encoded as ctl code)
	 tx1_ctl_code <= 4'b0;
      end else begin
	 // passing through
	 tx1_de <= tx0_de;
	 tx1_basic_de <= tx0_basic_de;
	 tx1_blue <= tx0_blue; 
	 tx1_green <= tx0_green;
	 tx1_red <= tx0_red;
	 tx1_hsync <= tx0_hsync;
	 tx1_vsync <= tx0_vsync;
	    
	 tx1_vid_gb <= tx0_vid_gb;
	 tx1_vid_pa <= tx0_vid_pa;
	 tx1_dat_gb <= tx0_dat_gb;
	 tx1_dat_pa <= tx0_dat_pa;
	 tx1_dat_ena <= tx0_dat_ena;
	    
	 tx1_ctl_code <= tx0_ctl_code;
	 tx1_dat_din[9:0] <= tx0_dat_din[9:0];
	 tx1_sdata <= tx0_sdata;
	 tx1_cv <= tx0_cv;
      end // else: !if( self_mode )
      
      tx2_de <= tx1_de;
      tx2_hsync <= tx1_hsync;
      tx2_vsync <= tx1_vsync;
      tx2_vid_pa <= tx1_vid_pa;
      tx2_vid_gb <= tx1_vid_gb;
      tx2_dat_pa <= tx1_dat_pa;
      tx2_dat_gb <= tx1_dat_gb;
      tx2_dat_ena <= tx1_dat_ena;
      tx2_ctl_code <= tx1_ctl_code;
      tx2_dat_din <= tx1_dat_din;
      tx2_sdata <= tx1_sdata;
      tx2_cv <= tx1_cv;
      
   end // always @ (posedge tx0_pclk)

//   assign de_sync = use_basic_de ? tx0_basic_de : tx0_de;
   assign de_sync = tx0_de;
   
   // assuming this doesn't need to be pipelined...   
   assign tx0_pll_reset    =  (rx0_reset && !self_mode) || (comp_ctl[4] && self_mode);

//   reg tx0_reset_wire;
//   always @(posedge tx0_pclk) begin
//      tx0_reset_wire <= rx0_reset || (comp_ctl[4] && self_mode);
//   end
   
//   BUFG tx0_reset_bufg( .I(tx0_reset_wire), .O(tx0_rstin));
   
   assign tx0_rstin = (rx0_reset && !self_mode) || (comp_ctl[4] && self_mode);

   gbgen gbgen (
		.pclk(tx0_pclk),
		.rstin(tx0_rstin),
		.vsync(lcdr_vsync),
		.hsync(lcdr_hsync),
		.sync_pol(1'b1), // sync from CPU is always active high
		.de(lcdr_de),

		.gb(computed_gb),
		.code(computed_ctl_code)
		);

  //////////////////////////////////////////////////////////////////
  // Instantiate a dedicate PLL for output port
  //////////////////////////////////////////////////////////////////
  wire tx0_clkfbout, tx0_clkfbin;
  wire tx0_pllclk0, tx0_pllclk2;

  PLL_BASE # (
//    .CLKIN_PERIOD(10.526315), // 95 MHz
//    .CLKIN_PERIOD(35.34), // 28.29 MHz 480p/60
    .CLKIN_PERIOD(13.481449525), // 74.176 MHz
    .CLKFBOUT_MULT(10), //set VCO to 10x of CLKIN
    .CLKOUT0_DIVIDE(1),
    .CLKOUT1_DIVIDE(10),
    .CLKOUT2_DIVIDE(5),
//	      .BANDWIDTH("LOW"), // normally not here
    .COMPENSATION("SOURCE_SYNCHRONOUS")
  ) PLL_OSERDES_0 (
    .CLKFBOUT(tx0_clkfbout),
    .CLKOUT0(tx0_pllclk0),
    .CLKOUT1(),
    .CLKOUT2(tx0_pllclk2),
    .CLKOUT3(),
    .CLKOUT4(),
    .CLKOUT5(),
    .LOCKED(tx0_plllckd),
    .CLKFBIN(tx0_clkfbin),
    .CLKIN(tx0_pclk_todcm),
    .RST(tx0_pll_reset || comp_ctl[4] || (!m720p_locked & self_mode))  
  );

   wire lcd_intbuf;
   //////////////////// buffer our Rx clock back to the CPU so we are meso-synchronous
   ODDR2 lcd_refclk_buf (.D0(1'b1), .D1(1'b0), .C0(tx0_pclk), .C1(!tx0_pclk), .Q(LCDO_DCLK), .CE(1), .R(0), .S(0) );
//   wire LCD_fbkclk;
//   IBUF lcd_fbk_ibuf(.I(LCDO_DCLK), .O(LCD_fbkclk));
//   BUFG lcd_fbk_buf(.I(LCD_fbkclk), .O(lcd_intbuf));
   
  //
  // This BUFG is needed in order to deskew between PLL clkin and clkout
  // So the tx0 pclkx2 and pclkx10 will have the same phase as the pclk input
  //
  BUFG tx0_clkfb_buf (.I(tx0_clkfbout), .O(tx0_clkfbin));

  //
  // regenerate pclkx2 for TX
  //
  BUFG tx0_pclkx2_buf (.I(tx0_pllclk2), .O(tx0_pclkx2));

  //
  // regenerate pclkx10 for TX
  //
  wire tx0_bufpll_lock;
  BUFPLL #(.DIVIDE(5)) tx0_ioclk_buf (.PLLIN(tx0_pllclk0), .GCLK(tx0_pclkx2), .LOCKED(tx0_plllckd),
           .IOCLK(tx0_pclkx10), .SERDESSTROBE(tx0_serdesstrobe), .LOCK(tx0_bufpll_lock));

   // reset off of master PLL lock, not BUFPLL lock
   assign tx0_reset = (~tx0_plllckd) || (comp_ctl[4] & self_mode);

   wire byp_error;

   reg [7:0] blend_b1;
   reg [7:0] blend_g1;
   reg [7:0] blend_r1;

`ifdef ALPHA_ON
   wire [7:0] alpha_enc_b;
   wire [7:0] alpha_enc_r;
   wire [7:0] alpha_enc_g;

   alpha_blend blender(.clk(tx0_pclk), .reset(tx0_reset),
	       .enc_b(tx1_blue),
	       .enc_r(tx1_red),
	       .enc_g(tx1_green),
	       .alpha_vid(alpha_vid),
	       .alpha_en(alpha_en),
	       .lcd_b(blend_b1), // assume: pre-scaled by alpha_lcd
	       .lcd_r(blend_r1),
	       .lcd_g(blend_g1),
	       .cipher_stream(cipher_stream[23:0]),
	       .alpha_enc_b(alpha_enc_b),
	       .alpha_enc_r(alpha_enc_r),
	       .alpha_enc_g(alpha_enc_g));

   wire [7:0] blend_b_scaled;
   wire [7:0] blend_r_scaled;
   wire [7:0] blend_g_scaled;
   alpha_scale scaler_b(.clk(tx0_pclk),
		      .reset(tx0_reset),
		      .code(alpha_vid), // the inversion of the code is done in the module LUT
		      .din(blend_b),
		      .dout(blend_b_scaled));
   
   alpha_scale scaler_r(.clk(tx0_pclk),
		      .reset(tx0_reset),
		      .code(alpha_vid), // the inversion of the code is done in the module LUT
		      .din(blend_r),
		      .dout(blend_r_scaled));
   
   alpha_scale scaler_g(.clk(tx0_pclk),
		      .reset(tx0_reset),
		      .code(alpha_vid), // the inversion of the code is done in the module LUT
		      .din(blend_g),
		      .dout(blend_g_scaled));
`endif	       

   assign hdcp_comp_ready = comp_ctl[0] && hdcp_is_ready;
   
   always @ (posedge tx0_pclk) begin
`ifdef ALPHA_ON
      blend_b1 <= alpha_en ? blend_b_scaled : blend_b;
      blend_r1 <= alpha_en ? blend_r_scaled : blend_r;
      blend_g1 <= alpha_en ? blend_g_scaled : blend_g;
      
      tx2_blue <= !((box_active || alpha_en) & comp_ctl[2]) ? 
	/* 1 */	  tx1_blue :
	/* 0 */	     hdcp_comp_ready ? 
	   /* 1 */      alpha_enc_b :
	   /* 0 */      (alpha_en ? 
              /* 1 */      blend_b1 + (tx1_blue >> alpha_vid) : 
              /* 0 */      blend_b1[7:0]);
      
      tx2_green <= !((box_active || alpha_en) & comp_ctl[2]) ? tx1_green :
		   hdcp_comp_ready ? alpha_enc_g : 
		   (alpha_en ? blend_g1 + (tx1_green >> alpha_vid) : blend_g1[7:0]);
      
      tx2_red <= !((box_active || alpha_en) & comp_ctl[2]) ? tx1_red :
		 hdcp_comp_ready ? alpha_enc_r :
		 (alpha_en ? blend_r1 + (tx1_red >> alpha_vid) : blend_r1[7:0]);
`else
      blend_b1 <= blend_b;
      blend_r1 <= blend_r;
      blend_g1 <= blend_g;
      
      tx2_blue <= !(box_active & comp_ctl[2]) ? 
	/* 1 */	  tx1_blue :
	/* 0 */	      hdcp_comp_ready ? 
	      /* 1 */    blend_b1[7:0] ^ cipher_stream[7:0] :   // encrypted path
	      /* 0 */    blend_b1[7:0];                         // unecrypted path
   
      tx2_green <= !(box_active & comp_ctl[2]) ? tx1_green :
		   hdcp_comp_ready ? blend_g1[7:0] ^ cipher_stream[15:8] : blend_g1[7:0];
      
      tx2_red <= !(box_active & comp_ctl[2]) ? tx1_red :
		 hdcp_comp_ready ? blend_r1[7:0] ^ cipher_stream[23:16] : blend_r1[7:0];
`endif	 
   end // always @ (posedge tx0_pclk)
   
   
  dvi_encoder_top dvi_tx0 (
    .pclk        (tx0_pclk),
    .pclkx2      (tx0_pclkx2),
    .pclkx10     (tx0_pclkx10),
    .serdesstrobe(tx0_serdesstrobe),
    .rstin       (tx0_reset),
    .blue_din    (tx2_blue),
    .green_din   (tx2_green),
    .red_din     (tx2_red),
    .hsync       (tx2_hsync),
    .vsync       (tx2_vsync),
    .de          (tx2_de),
    .TMDS        (TX0_TMDS),
    .TMDSB       (TX0_TMDSB),
    .vid_pa      (tx2_vid_pa),
    .vid_gb      (tx2_vid_gb),
    .dat_pa      (tx2_dat_pa),
    .dat_gb      (tx2_dat_gb),
    .dat_ena     (tx2_dat_ena),
    .dat_din     (tx2_dat_din),
    .ctl_code    (tx2_ctl_code),
    .bypass_sdata(tx2_sdata),
    .bypass_ena  (1'b1),
    .byp_error   (byp_error),
    .box_active  ((box_active && comp_ctl[2]) || self_mode)
  );

   assign hsync_rising = hsync_v & !hsync_v2;
   assign vsync_rising = vsync_v & !vsync_v2;
   // extend hsync's validity when control periods are inactive
   always @(posedge tx0_pclk or posedge tx0_reset) begin
      if( tx0_reset == 1'b1 ) begin
	 vsync_v <= 0;
	 hsync_v <= 0;
	 vsync_v2 <= 0;
	 hsync_v2 <= 0;
      end else begin
	 vsync_v2 <= vsync_v;
	 hsync_v2 <= hsync_v;
   	 if(tx0_cv) begin
	    vsync_v <= tx0_vsync ^ !hdmi_vsync_pol;
	    hsync_v <= tx0_hsync ^ !hdmi_hsync_pol;
	 end else begin
	    vsync_v <= vsync_v;
	    hsync_v <= hsync_v;
	 end
      end // else: !if( tx0_reset == 1'b1 )
   end // always @ (posedge tx0_pclk or posedge tx0_reset)

`ifdef REGWINDOWS
   always @(posedge tx0_pclk or posedge tx0_rstin) begin
      if(tx0_rstin) begin
	 window_x_buf <= 12'b0;
	 window_y_buf <= 12'b0;
	 window_h_buf <= 12'b0;
	 window_w_buf <= 12'b0;
      end else begin
	 if( vsync_rising && window_update ) begin
	    window_x_buf[11:0] <= window_x[11:0];
	    window_y_buf[11:0] <= window_y[11:0];
	    window_w_buf[11:0] <= window_w[11:0];
	    window_h_buf[11:0] <= window_h[11:0];
	 end else begin
	    window_x_buf[11:0] <= window_x_buf[11:0];
	    window_y_buf[11:0] <= window_y_buf[11:0];
	    window_w_buf[11:0] <= window_w_buf[11:0];
	    window_h_buf[11:0] <= window_h_buf[11:0];
	 end // else: !if( vsync_rising && window_update )
      end // else: !if(tx0_rstin)
   end // always @ (posedge tx0_pclk or posedge tx0_rstin)
   
   // instantiate the timing module that generates the mask box for enabling the overlay
   boxtiming boxtimer (
		       .pclk(tx0_pclk),
		       .rstin(tx0_rstin),
//		       .vsync(tx0_vsync ^ !hdmi_vsync_pol),
//		       .hsync(tx0_hsync ^ !hdmi_hsync_pol),
		       .vsync(vsync_v),
		       .hsync(hsync_v),
		       .sync_pol(1'b1), // with auto-detect, polarity is always "righted"
		       .de(de_sync),
		       .cv(tx0_cv),

		       .hpos(window_x_buf[11:0]),
		       .hsize(window_w_buf[11:0]),
		       .vpos(window_y_buf[11:0]),
		       .vsize(window_h_buf[11:0]),

		       .box_active(box_active_raw)
		       );
`else // !`ifdef REGWINDOWS
   assign box_active_raw = de_sync & !(vsync_v || hsync_v);
`endif // !`ifdef REGWINDOWS
   
   // chroma should only check high 6 bits because the lower bits don't exist
   assign chroma_match = (blend_b[7:2] == chroma_b[7:2]) &&
			 (blend_r[7:2] == chroma_r[7:2]) &&
			 (blend_g[7:2] == chroma_g[7:2]);
   
   assign box_active = box_active_raw && (!chroma_match || !ext1_ctl[1]);

   // hpd debounce and delay
   always @(posedge tx0_pclk or posedge tx0_reset) begin
      if( tx0_reset == 1'b1 ) begin
	 hpd_saw_vsync <= 1'b0;
      end else begin
	 if( HPD_N ) begin
	    hpd_saw_vsync <= 1'b0;
	 end else begin
	    if( self_mode ) begin
	       // in self-timed mode, there's no external vsync to sync to, so just pass this through
	       hpd_saw_vsync <= 1'b1;
	    end else begin
	       // in genlock mode, synch to a vsync edge as a debounce mechanism
	       if( vsync_rising ) begin
		  hpd_saw_vsync <= 1'b1;
	       end else begin
		  hpd_saw_vsync <= hpd_saw_vsync;
	       end
	    end
	 end
      end
   end // always @ (posedge tx0_pclk or posedge tx0_reset)
//   assign HPD_NOTIFY = hpd_saw_vsync;
   assign HPD_NOTIFY = !HPD_N; // fuck debouncing, the CPU can take care of it. 800 MHz of CPU power and all it has to do is...

   assign HPD_OVERRIDE = snoop_ctl[3]; // force hpd to look like nothing is plugged in hen asserted

   /////////// timing extraction machine
`ifdef TIMING_POSTX
   // run the timing detector after the Tx mux, so that during self-timed mode we get
   // the timing of what we're putting to the screen as feedback...
   timing_detector timing_det (
			       .pclk(tx0_pclk),
			       .rstin(tx0_rstin),
			       .vsync(self_mode ? tx1_vsync : vsync_v), // vsync_v is polarity-corrrected
			       .hsync(self_mode ? tx1_vsync : hsync_v), // hsync_v is polarity-corrected
			       .de(self_mode ? tx1_de : de_sync),
			       .refclk(clk26buf), // actually 26 MHz due to PXA168 multipliers

			       .lcd_de(lcdr_de),
			       .lcd_vsync(lcdr_vsync),

			       .hactive(t_hactive), // pixels
			       .vactive(t_vactive), // pixels
			       .htotal(t_htotal), // pixels
			       .vtotal(t_vtotal), // PIXELS
			       .h_frontporch(t_h_fp), // pixels
			       .h_backporch(t_h_bp), // pixels
			       .v_frontporch(t_v_fp), // PIXELS
			       .v_backporch(t_v_bp), // PIXELS
			       .hsync_width(t_hsync_width), // pixels
			       .vsync_width(t_vsync_width), // PIXELS
			       .lcd_de_latency(t_lcd_de_latency),
			       .lcd_vsync_latency(t_vsync_latency),

			       .refclkcnt(t_refclkcnt)
			       );
`else // !`ifdef TIMING_POSTX
   // run the timing detector after the Rx mux, so that during self-timed mode we
   // get the timing of whatever's coming in on the input port.
   timing_detector timing_det (
			       .pclk(rx0_pllclk1), // off of the rx clock
			       .rstin(tx0_rstin),
			       .vsync(vsync_v), // vsync_v is polarity-corrrected
			       .hsync(hsync_v), // hsync_v is polarity-corrected
			       .de(de_sync),
			       .refclk(clk26buf), // actually 26 MHz due to PXA168 multipliers

			       .lcd_de(lcdr_de),
			       .lcd_vsync(lcdr_vsync),

			       .hactive(t_hactive), // pixels
			       .vactive(t_vactive), // pixels
			       .htotal(t_htotal), // pixels
			       .vtotal(t_vtotal), // PIXELS
			       .h_frontporch(t_h_fp), // pixels
			       .h_backporch(t_h_bp), // pixels
			       .v_frontporch(t_v_fp), // PIXELS
			       .v_backporch(t_v_bp), // PIXELS
			       .hsync_width(t_hsync_width), // pixels
			       .vsync_width(t_vsync_width), // PIXELS
			       .lcd_de_latency(t_lcd_de_latency),
			       .lcd_vsync_latency(t_vsync_latency),

			       .refclkcnt(t_refclkcnt)
			       );

`endif // !`ifdef TIMING_POSTX
      
  /////////////////
  //
  // LCD input & FIFO
  //
  /////////////////

   // This module will contain the input from the LCD
   // and a FIFO that can hold a few lines of video data
   wire dummy;
   wire genlock;
   assign VSYNC_STB = genlock & !self_mode; // only genlock when self-mode is off
   
   lcd_input lcd_input_top (
			    .rstin(self_mode ? comp_ctl[4] : rx0_reset),
//			    .lcd_dclk(LCD_DCLK_intbuf),
			    .lcd_dclk(tx0_pclk),
			    .lcd_de(lcdr_de),
			    .lcd_hsync(lcdr_hsync),
			    .lcd_vsync(lcdr_vsync),
			    .lcd_sync_pol(1'b1),
			    .lcd_b(lcdr_b),
			    .lcd_g(lcdr_g),
			    .lcd_r(lcdr_r),

			    .hdmi_pclk(tx0_pclk),
			    .hdmi_de(de_sync),
//nuke			    .hdmi_vsync(tx0_vsync),
//			    .hdmi_hsync(tx0_hsync),
			    .hdmi_vsync(tx0_vsync ^ !hdmi_vsync_pol),
			    .hdmi_hsync(tx0_hsync ^ !hdmi_hsync_pol),
			    .hdmi_sync_pol(1'b1),
			    .hdmi_cv(tx0_cv),

			    .hdmi_b(blend_b),
			    .hdmi_r(blend_r),
			    .hdmi_g(blend_g),
			    .genlock(genlock),
			    .locked(genlock_locked),
			    
			    .dummy(dummy),

			    .line_full_level(line_full_level[2:0]),
			    .line_empty_level(line_empty_level[2:0]),
			    .write_init_level(write_init_level),
			    .read_init_level(read_init_level),

			    .target_lead_pixels(target_lead_pixels),
			    .reset_lock_machine(reset_lock_machine),

			    .lock_tolerance(lock_tolerance),
			    .smartlock_on(smartlock_on)
			    );
   assign reset_lock_machine = comp_ctl[6];
   assign smartlock_on = comp_ctl[5];

   reg  hdmi_de_d;
   wire hdmi_de_rising;
   reg 	vsync_v_raw, hsync_v_raw;
   wire actual_reset;
   assign actual_reset = self_mode ? comp_ctl[4] : rx0_reset;
   // hdmi sync polarity detector
   always @(posedge tx0_pclk or posedge actual_reset) begin
      if(actual_reset) begin
	 hdmi_vsync_pol <= 0;
	 hdmi_hsync_pol <= 0;
	 vsync_v_raw <= 0;
	 hsync_v_raw <= 0;
      end else begin
	 hdmi_de_d <= de_sync;

	 if(tx0_cv) begin
	    vsync_v_raw <= tx0_vsync;
	    hsync_v_raw <= tx0_hsync;
	 end else begin
	    vsync_v_raw <= vsync_v_raw;
	    hsync_v_raw <= hsync_v_raw;
	 end
	 // the theory goes that de is always active high so use this to adjust sync polarities
	 if( hdmi_de_rising ) begin
	    hdmi_vsync_pol <= !vsync_v_raw;
	    hdmi_hsync_pol <= !hsync_v_raw;
	 end else begin
	    hdmi_vsync_pol <= hdmi_vsync_pol;
	    hdmi_hsync_pol <= hdmi_hsync_pol;
	 end
      end
   end // always @ (posedge hdmi_pclk)
   assign hdmi_de_rising = !hdmi_de_d & de_sync;

`define HDCP_MODULE  1
`ifdef HDCP_MODULE
   ///////
   // HDCP
   ///////
   // HDCP initialization procedure
   //
   // 1. Sniff An, KSV going across the wire
   // 2. Generate private key table for one of the KSV's
   // 3. Perform the Km computation using derived private key table
   // 4. Enter An, Km into the register for the HDCP cipher
   // 5. Initiate the authentication (pulse hdcpBlockCipher_init and 
   //    authentication high one cycle simultaneously)
   // 6. Wait until stream_ready is high
   //
   // Now begins the main loop:
   // There is an ambiguity in the spec. Either a rekey operation happens immediately
   // (since this happens during vertical retrace), or not. Either way, this is roughly
   // what happens.
   //
   // 1. If hdcp_ena activates (or of de and data island enable), advance cipher
   // 2. If vertical sync happens, pulse hdcpBlockCipher_init one cycle and wait 
   //    until stream_ready; return to 1
   // 3. If horizontal sync happens, pulse hdcpRekeyCipher once cycle, wait until
   //    stream_ready; return to 1
   //
   // That's it. So the only question is if vsync "happens" immediately after an authentication.
   // The test vectors would suggest this is the case but I can't find it in the state machine
   // diagrams, so perhaps good to try both options...?
   parameter HDCP_UNPLUG      = 18'b1 << 0;
   parameter HDCP_WAIT_AKSV   = 18'b1 << 1;
   parameter HDCP_AUTH_PULSE  = 18'b1 << 2;
   parameter HDCP_AUTH        = 18'b1 << 3;
   parameter HDCP_AUTH_WAIT   = 18'b1 << 4;
   parameter HDCP_AUTH_VSYNC_PULSE  = 18'b1 << 5;
   parameter HDCP_AUTH_VSYNC        = 18'b1 << 6;
   parameter HDCP_AUTH_VSYNC_WAIT   = 18'b1 << 7;
   parameter HDCP_WAIT_1001   = 18'b1 << 8;
   parameter HDCP_WAIT_1001_END = 18'b1 << 9;
   parameter HDCP_VSYNC       = 18'b1 << 10;
   parameter HDCP_VSYNC_PULSE = 18'b1 << 11;
   parameter HDCP_VSYNC_WAIT  = 18'b1 << 12;
   parameter HDCP_READY       = 18'b1 << 13;
   parameter HDCP_REKEY       = 18'b1 << 14;
   parameter HDCP_REKEY_PULSE = 18'b1 << 15;
   parameter HDCP_REKEY_WAIT  = 18'b1 << 16;
   parameter HDCP_WAIT_KMRDY  = 18'b1 << 17;

   parameter HDCP_nSTATES = 18;
   
   reg [(HDCP_nSTATES-1):0]     HDCP_cstate = {{(HDCP_nSTATES-1){1'b0}}, 1'b1};
   reg [(HDCP_nSTATES-1):0]     HDCP_nstate;

   reg 				auth_mode;
   reg 				hdcp_init;
   reg 				hdcp_rekey;
   wire 			hdcp_stream_ena;

   reg 				active_line;
   reg 				hdcp_rekey_2;
   reg 				hdcp_rekey_1;


   reg 				appleTV2_bug;

   // Apple TV version 2 has a bug where the 1001 EESS happens by far too early in the window of oppty
   // spec is a window 512 cycles after vsync start, in practice ATV2 is a window about 94 clocks after vsync
   always @ (posedge tx0_pclk or posedge tx0_reset) begin
      if( tx0_reset == 1'b1 ) begin
	 appleTV2_bug <= 0;
      end else begin
	 if( (HDCP_cstate == HDCP_VSYNC_WAIT) && (rx0_ctl_code[3:0] == 4'b1001) ) begin
	    appleTV2_bug <= 1;
	 end else if( (HDCP_cstate == HDCP_READY) ) begin
	    appleTV2_bug <= 0;
	 end else begin
	    appleTV2_bug <= appleTV2_bug;
	 end
      end // else: !if( tx0_reset == 1'b1 )
   end // always @ (posedge tx0_pclk or posedge tx0_reset)
   
   assign hdcp_is_ready = (HDCP_cstate == HDCP_READY);
   
   // compute active_line. This tells you if the last line had active data
   // in it or not. Reset the computation on falling edge of hsync
   always @ (posedge tx0_pclk or posedge tx0_reset) begin
      if( tx0_reset == 1'b1 ) begin
	 active_line <= 1'b0;
	 hdcp_rekey_2 <= 1'b0;
	 hdcp_rekey_1 <= 1'b0;
      end else begin
	 hdcp_rekey_2 <= hdcp_rekey_1;
	 hdcp_rekey_1 <= hdcp_rekey || 
			 (rx0_line_end && (HDCP_cstate == HDCP_READY) &&
			  tx0_de);
	 if( tx0_de ) begin
	    active_line <= 1'b1;
	 end else if( !hsync_v & hsync_v2 ) begin // hsync falling
	    active_line <= 1'b0;
	 end
      end
   end
   
   always @ (posedge tx0_pclk or posedge HPD_N or posedge tx0_reset or negedge rstbtn_n) begin
      if (~rstbtn_n | HPD_N | tx0_reset )
	HDCP_cstate <= HDCP_UNPLUG; 
      else
	if( Aksv14_write ) begin
	   HDCP_cstate <= HDCP_AUTH_PULSE; // hack for tivo series 3
	end else begin
	   HDCP_cstate <=#1 HDCP_nstate;
	end
   end

   always @ (*) begin
      case (HDCP_cstate) //synthesis parallel_case full_case
	HDCP_UNPLUG: begin
	   HDCP_nstate = HPD_N ? HDCP_UNPLUG : HDCP_WAIT_AKSV;
	end
	HDCP_WAIT_AKSV: begin
	   // wait until the 14th byte is written to the HDCP register set
	   // this is the MSB of AKsv, and this triggers an authentication event
	   HDCP_nstate = Aksv14_write ? HDCP_AUTH_PULSE : HDCP_WAIT_AKSV;
//	   HDCP_nstate = Aksv14_write ? HDCP_WAIT_KMRDY : HDCP_WAIT_AKSV;
	   // in this implementation, skip the HDCP_WAIT_KMRDY state
	end

	// this state is unreachable
	HDCP_WAIT_KMRDY: begin
	   HDCP_nstate = Km_ready ? HDCP_AUTH_PULSE : HDCP_WAIT_KMRDY;
	end
	
	////////
	// maybe put a state here to wait for Km to become ready
	// but for now, we assume host has pre-loaded Km. Km is fixed for every Tx/Rx HDMI pair.
	// So once you have computed it, it can be pre-loaded even before the transaction happens.
	// One way around this is to snag AKsv, Bksv; and if they are a new pair, compute Km 
	// and load it; and then override HPD_N high for a second to force a re-key *only* if
	// this is new pair. Thus, the first time you plug in a new device you *might* see it
	// flicker once, but it would never happen again, but I think typically you would
	// not notice because the screen would stay dark the entire time.
	//
	// --> above is the wait KMRDY state. The way this should work now is:
	// 1. Aksv is written, byte 14 triggers an interrupt to the CPU.
	// 2. CPU derives Km, writes Km, sets Km ready
	// 3. state machine then moves on to initiate auth pulse
	//
	////////
	HDCP_AUTH_PULSE: begin
	   HDCP_nstate = HDCP_AUTH;
	end
	HDCP_AUTH: begin
	   HDCP_nstate = stream_ready? HDCP_AUTH : HDCP_AUTH_WAIT;
	end
	HDCP_AUTH_WAIT: begin
	   HDCP_nstate = stream_ready ? HDCP_AUTH_VSYNC_PULSE : HDCP_AUTH_WAIT;
	end

	// this is a special vsync-update state just for after auth
	// because I don't know if there is more than 1 vsync period between
	// the conclusion of auth and the first 1001 assertion
	// if there is, then we end up unsynchronized on the Mi state
	HDCP_AUTH_VSYNC_PULSE: begin
	   HDCP_nstate = HDCP_AUTH_VSYNC;
	end
	HDCP_AUTH_VSYNC: begin
	   HDCP_nstate = stream_ready ? HDCP_AUTH_VSYNC : HDCP_AUTH_VSYNC_WAIT;
	end
	HDCP_AUTH_VSYNC_WAIT: begin
	   HDCP_nstate = stream_ready ? HDCP_WAIT_1001 : HDCP_AUTH_VSYNC_WAIT;
	end

	// our primary wait state
	HDCP_WAIT_1001: begin
	   HDCP_nstate = ((vsync_v && (rx0_ctl_code[3:0] == 4'b1001)) || appleTV2_bug) ? 
			 HDCP_WAIT_1001_END : HDCP_WAIT_1001;
	end
	HDCP_WAIT_1001_END: begin
	   HDCP_nstate = (vsync_v && (rx0_ctl_code[3:0] == 4'b1001)) ?
			 HDCP_WAIT_1001_END : HDCP_READY;
	end
	

	HDCP_VSYNC_PULSE: begin
	   HDCP_nstate = HDCP_VSYNC;
	end
	HDCP_VSYNC: begin
	   HDCP_nstate = stream_ready ? HDCP_VSYNC : HDCP_VSYNC_WAIT;
	end
	HDCP_VSYNC_WAIT: begin
	   HDCP_nstate = stream_ready ? HDCP_WAIT_1001 : HDCP_VSYNC_WAIT;
	end

	// our primary cipher state
	HDCP_READY: begin
//	   HDCP_nstate = (!rx0_de & tx0_de) ? HDCP_REKEY_PULSE :
	   // i've now got a signal banging rekey outside this state machine
	   // it's unclean, but necessary to get rekey to happen early enough
	   // to meet hdcp spec requirement for rekey time.
	   // Core assumption: the only way stream becomes un-ready during
	   // HDCP_READY is due to the external rekey event. vsync_rising
	   // will never result in this triggering because it itself must
	   // transition this state machine to a new state before stream_ready
	   // changes; and furthermore, stream_ready is guaranteed to be high
	   // upon return to this state.
	   HDCP_nstate = (stream_ready == 1'b0) ? HDCP_REKEY_WAIT : 
			 vsync_rising ? HDCP_VSYNC_PULSE :
			 HDCP_READY;
	end

	HDCP_REKEY_PULSE: begin
	   HDCP_nstate = HDCP_REKEY;
	end
	HDCP_REKEY: begin
	   HDCP_nstate = stream_ready ? HDCP_REKEY : HDCP_REKEY_WAIT;
	end
	HDCP_REKEY_WAIT: begin
	   HDCP_nstate = stream_ready ? HDCP_READY : HDCP_REKEY_WAIT;
	end
      endcase // case (HDCP_cstate)
   end

//   assign Km_ready = !Km_rdy2 & Km_rdy1; // rising edge pulse
   assign Km_ready = Km_rdy2; // for now make it level triggered ("cheezy mode")
		     
   always @ (posedge tx0_pclk or posedge HPD_N or posedge tx0_reset or negedge rstbtn_n) begin
      if( ~rstbtn_n | HPD_N | tx0_reset ) begin
	 auth_mode <=#1 1'b0;
	 hdcp_init <=#1 1'b0;
	 hdcp_rekey <=#1 1'b0;
	 hdcp_requested <=#1 1'b0;
	 
	 Km_rdy0 <= 1'b0;
	 Km_rdy1 <= 1'b0;
	 Km_rdy2 <= 1'b0;
      end else begin
	 Km_rdy0 <= comp_ctl[7];
	 Km_rdy1 <= Km_rdy0;
	 Km_rdy2 <= Km_rdy1;

	 case (HDCP_cstate) //synthesis parallel_case full_case
	   HDCP_UNPLUG: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   HDCP_WAIT_AKSV: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   
	   HDCP_WAIT_KMRDY: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   
	   HDCP_AUTH_PULSE: begin
	      auth_mode <=#1 1'b1;
	      hdcp_init <=#1 1'b1; // pulse just one cycle
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   HDCP_AUTH: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   HDCP_AUTH_WAIT: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end

	   HDCP_AUTH_VSYNC_PULSE: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b1;  // pulse init, but not with auth_mode
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   HDCP_AUTH_VSYNC: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   HDCP_AUTH_VSYNC_WAIT: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   
	   HDCP_WAIT_1001: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   HDCP_WAIT_1001_END: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b0;
	   end
	   
	   HDCP_VSYNC_PULSE: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b1;  // pulse init, but not with auth_mode
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b1;
	   end
	   HDCP_VSYNC: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b1;
	   end
	   HDCP_VSYNC_WAIT: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b1;
	   end
	   
	   HDCP_READY: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b1;
	   end
	   
	   HDCP_REKEY_PULSE: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
//	      hdcp_rekey <=#1 1'b1; // pulse rekey
	      hdcp_rekey <=#1 1'b0; // we're going to do this asychronously to save some cycles
	      // yes, it means hdcp_rekey gets optimized out
	      // but structurally this helps me remember what the code was intended to do
	      hdcp_requested <=#1 1'b1;
	   end
	   HDCP_REKEY: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b1;
	   end
	   HDCP_REKEY_WAIT: begin
	      auth_mode <=#1 1'b0;
	      hdcp_init <=#1 1'b0;
	      hdcp_rekey <=#1 1'b0;
	      hdcp_requested <=#1 1'b1;
	   end
	 endcase // case (HDCP_cstate)
      end // else: !if( ~rstbtn_n | HPD_N )
   end // always @ (posedge tx0_pclk)
   
   
   wire stream_ready;
   hdcp_cipher  cipher (
		.clk(tx0_pclk),
		.reset(tx0_rstin),
		.Km(Km),
		.An(An),
		.hdcpBlockCipher_init(hdcp_init),
		.authentication(auth_mode),
		.hdcpRekeyCipher(hdcp_rekey_2),
		.hdcpStreamCipher(rx0_hdcp_ena && (HDCP_cstate == HDCP_READY)),
		.pr_data(cipher_stream),
		.stream_ready(stream_ready)
		);
`endif //  `ifdef HDCP_MODULE

  //////////////////////////////////////
  // Status LED
  //////////////////////////////////////
   reg [22:0] counter;

   always @(posedge clk26buf) begin
      if( rstbtn_n == 1'b0 ) begin
	 counter <= 1'b0;
	 LED0 <= 1'b0;
      end else begin
	 counter <= counter + 1;
//	 LED <= counter[22] & !rx0_de;
//	 LED <= counter[22] & byp_error | dummy;
//	 LED <= counter[22] & dummy;
//	 LED0 <= dummy & snoop_ctl[7];
`ifdef PASSTHROUGH
//	 LED0 <= dummy;
	 LED0 <= vsync_v;
`else
//	 LED0 <= tx0_plllckd && m720p_locked & ss_locked;
`endif
	 
//	 LED1 <= counter[22];
//	 LED1 <= de_sync;
      end // else: !if( rstbtn_n == 1'b0 )

      LOWVOLT_NOTIFY <= LOWVOLT_N;
      HDCP_AKSV <= Aksv14_write; // retime it into this domain to not screw up timing closure
   end

  //assign LED[3:0] = {rx0_red_rdy | counter[23], rx0_green_rdy  | counter[23], rx0_blue_rdy  | counter[23], 
  //                rx0_de | counter[23]};


   ////////////////////////////////
   // serial number
   ////////////////////////////////

   reg clk2M_unbuf;
   (* clock_signal = "yes" *)
   (* PERIOD = "period 0.8125 MHz" *)
   wire clk2M;
   wire clk1M;
   reg 	clk1M_unbuf;
   always @(posedge clk26buf) begin
      clk2M_unbuf <= counter[4]; // 0.8MHz clock: device DNA only runs at 2 MHz
      clk1M_unbuf <= counter[6];
   end
   
   BUFG clk2M_buf(.I(clk2M_unbuf), .O(clk2M));
   BUFG clk1M_buf(.I(clk1M_unbuf), .O(clk1M));
   
   reg 	dna_pulse;
   reg 	dna_shift;
   wire dna_bit;
   DNA_PORT device_dna( .CLK(clk2M), .DIN(1'b0), .DOUT(dna_bit), .READ(dna_pulse), .SHIFT(dna_shift) );
   parameter DNA_INIT =    4'b1 << 0;
   parameter DNA_PULSE =   4'b1 << 1;
   parameter DNA_SHIFT =   4'b1 << 2;
   parameter DNA_DONE =    4'b1 << 3;

   parameter DNA_nSTATES = 4;

   reg [(DNA_nSTATES-1):0] DNA_cstate = {{(DNA_nSTATES-1){1'b0}}, 1'b1};
   reg [(DNA_nSTATES-1):0] DNA_nstate;
   reg [5:0] 		   dna_shift_count;

   always @ (negedge clk2M) begin
      if (~rstbtn_n)
	DNA_cstate <= DNA_INIT; 
      else
	DNA_cstate <=#1 DNA_nstate;
   end

   always @ (*) begin
      case (DNA_cstate) //synthesis parallel_case full_case
	DNA_INIT: begin
	   DNA_nstate = DNA_PULSE;
	end
	DNA_PULSE: begin
	   DNA_nstate = DNA_SHIFT;
	end
	DNA_SHIFT: begin
	   // depending on if MSB or LSB first, want to use 56 or 55
	   // especially if serial #'s are linear-incrementing
	   DNA_nstate = (dna_shift_count[5:0] == 6'd55) ? DNA_DONE : DNA_SHIFT;
	end
	DNA_DONE: begin
	   DNA_nstate = DNA_DONE;
	end
      endcase // case (DNA_cstate)
   end
   
   always @ (negedge clk2M) begin
      if( ~rstbtn_n ) begin
	   dna_shift_count <= 6'h0;
	   dna_data <= 56'h0;
	   dna_pulse <= 1'b0;
	   dna_shift <= 1'b0;
      end else begin
	 case (DNA_cstate) //synthesis parallel_case full_case
	   DNA_INIT: begin
	      dna_shift_count <= 6'h0;
	      dna_data <= 56'h0;
	      dna_pulse <= 1'b0;
	      dna_shift <= 1'b0;
	   end
	   DNA_PULSE: begin
	      dna_shift_count <= 6'h0;
	      dna_data <= 56'h0;
	      dna_pulse <= 1'b1;
	      dna_shift <= 1'b0;
	   end
	   DNA_SHIFT: begin
	      dna_shift_count <= dna_shift_count + 6'b1;
	      dna_data[55:0] <= {dna_data[54:0],dna_bit};
	      dna_pulse <= 1'b0;
	      dna_shift <= 1'b1;
	   end
	   DNA_DONE: begin
	      dna_shift_count <= dna_shift_count;
	      dna_data[55:0] <= dna_data[55:0];
	      dna_pulse <= 1'b0;
	      dna_shift <= 1'b0;
	   end
	 endcase // case (DNA_cstate)
      end // else: !if( ~rstbtn_n )
   end // always @ (posedge clk2M or posedge ~rstbtn_n)

   ////////////////////////////////
   // heartbeat
   ////////////////////////////////
   pwm heartbeat(.clk812k(clk1M), .reset(~rstbtn_n), .pwmout(blue_led),
		 .bright(12'b0000_1111_1000), .dim(12'b0000_0001_0000) );
   
   assign LED1 = blue_led | HPD_N;
   
endmodule
