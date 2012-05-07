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

/////////////
//  This module takes in video data via an LCD interface
//  and attempts to align it to an ongoing HDMI stream.
//
//  This is a reboot of the LCD module, version 3
//
//  In this iteration, we attempt to make a very strong guarantee that:
//   -- the incoming HDMI stream is the master source of all timing
//   -- that the LCD data is always squared into the HDMI stream, and strictly
//      resynchronized every V *and* H period
//   -- that at least 8 lines of buffering are available
//   -- in practice, I'm seeing about +/- 0.08% difference in clocks. That is
//      about +/- one full line of data due to absolute clock skew alone.
//   -- I see hiccoughs from the PXA168 on the order of 5-6 lines per frame
//
//   Version 4 takes advantage of synchronous clocking and makes a
//   feedback-loop controlled system that attempts to use genlock to "kick"
//   the host CPU into a closer sync relationship, within 8 lines of leading
//   the HDMI display.
//
//   The feedback loop *only* works if the host controller has the identical
//   timing to the incoming stream. Fortunately, a timing measurement module
//   has also been created so that the CPU can read out the timing of the incoming
//   stream and adjust the controller to match.
//
/////////////
`timescale 1 ns / 1 ps

module lcd_input(
		 input wire rstin,
		 input wire lcd_dclk,
		 input wire lcd_de,           // active high
		 input wire lcd_hsync,        // always active high
		 input wire lcd_vsync,        // always active high
		 input wire lcd_sync_pol,     // force to 1 at top level module
		 input wire [5:0] lcd_b,
		 input wire [5:0] lcd_g,
		 input wire [5:0] lcd_r,

		 input wire hdmi_pclk,
		 input wire hdmi_de,          // active high
		 input wire hdmi_vsync,       // use sync_pol
		 input wire hdmi_hsync,       // use sync_pol
		 input wire hdmi_sync_pol,
		 input wire hdmi_cv,          // indicates that hdmi sync signals are valid

		 output reg [7:0] hdmi_b,
		 output reg [7:0] hdmi_r,
		 output reg [7:0] hdmi_g,

		 output reg genlock,
		 output reg locked,

		 output wire dummy,

		 input wire [2:0] line_full_level, // nominally 2
		 input wire [2:0] line_empty_level, // nominally 2
		 input wire [3:0] write_init_level, // nominally 1
		 input wire [3:0] read_init_level, // nominally 2

		 input wire [23:0] target_lead_pixels,
		 input wire reset_lock_machine,

		 input wire [15:0] lock_tolerance,
		 input wire smartlock_on
		 );

   //////// FIFO connections
   wire [(NUMLINES-1) : 0] fifo_rst;
   
   wire [17:0] 		   fifo_din;
   wire [(NUMLINES-1) : 0] fifo_write;
   wire [(NUMLINES-1) : 0] fifo_full;
   wire [(NUMLINES-1) : 0] fifo_over;
   wire 		   lcd_en;
   
   wire [(NUMLINES-1) : 0] fifo_read;
   wire [(NUMLINES-1) : 0] fifo_empty;
   wire [(NUMLINES-1) : 0] fifo_under;
   wire 		   hdmi_en;

   wire [17:0] 		   fifo_dout0;
   wire [17:0] 		   fifo_dout1;
   wire [17:0] 		   fifo_dout2;
   wire [17:0] 		   fifo_dout3;
   wire [17:0] 		   fifo_dout4;
   wire [17:0] 		   fifo_dout5;
   wire [17:0] 		   fifo_dout6;
   wire [17:0] 		   fifo_dout7;
   

   /// other connections
   ////// HDMI clock domain
   reg 				    hdmi_hsync_v; // active when high
   reg 				    hdmi_hsync_v2;
   wire                             hdmi_hsync_rising;
   
   reg 				    hdmi_vsync_v;
   reg 				    hdmi_vsync_v2;
   wire 			    hdmi_vsync_rising;
   wire 			    hdmi_vsync_falling;
   
   wire 			    lcd_de_rising__hdmi;
   wire 			    lcd_de_falling__hdmi;
   wire 			    lcd_firstbyte__hdmi;
   
   reg 				    lcd_de_s__hdmi;
   reg 				    lcd_de__hdmi;
   reg 				    lcd_de_d__hdmi;
   
   reg 				    lcd_overflow_s__hdmi;
   reg 				    lcd_overflow__hdmi;

   reg 				    lcd_vsync_1hdmi;
   reg 				    lcd_vsync_2hdmi;
   reg 				    lcd_vsync_hdmi;
   wire 			    lcd_vsync_rising__hdmi;

   wire 			    hdmi_underflow;
   wire 			    hdmi_empty;
   wire [17:0] 			    hdmi_fifo_d;

   reg 				    hdmi_de_d;
   wire 			    hdmi_de_rising;
   wire 			    hdmi_de_falling;
   reg 				    hdmi_first_de;
   reg 				    hdmi_first_de_state;

   reg 				    hdmi_read_en;
      
   ////// LCD clock domain
   wire 			    lcd_de_vsync;
   reg 				    lcd_de_d;
   reg 				    lcd_de_rising;
   reg 				    lcd_de_falling;
   wire 			    lcd_overflow;
   reg 				    lcd_hsync_d;
   reg 				    lcd_hsync_rising;
      
   reg 				    hdmi_vsync_s__lcd; // synchronize hdmi vsync to LCD clock domain
   reg 				    hdmi_vsync__lcd;

   reg 				    hdmi_hsync_s__lcd; // synchronize hdmi hsync to LCD clock domain
   reg 				    hdmi_hsync__lcd;

   reg 				    hdmi_hsync_rising__lcd;
   reg 				    hdmi_vsync_rising__lcd;
   reg 				    hdmi_vsync_falling__lcd;
   reg 				    hdmi_hsync_d__lcd;
   reg 				    hdmi_vsync_d__lcd;

   wire advance_write;
   wire advance_read;

   ///////////////
   // Synchronous FIFO feedback loop.
   //
   // Assume: vsync from HDMI drives all timing.
   //
   // A counter is used to delay the genlock output to the CPU, based on a certain number of
   // pixel counts. The feedback loop attempts to tune the delay so that the actual measured
   // LCD vsync timing hits a target number of pixels before the HDMI vsync signal. If this
   // target is achieved, then fifo read/write timing is simply the LCD DE and HDMI DE signals.
   // 
   ///////////////
   parameter GENLOCK_LENGTH = 24'h400; // approx 6 us minimum pulse length, typ 12 us or so
   // just needs to be long enough for the CPU to catch the interrupt

   reg [23:0] 			    current_genlock_delay; // delay in pixels of genlock pulse from vsync
   reg [23:0] 			    genlock_count;
   reg [23:0] 			    measured_vsync_to_vsync;
   reg 				    hdmi_vsync_happened;

   reg 				    hdmi_vsync_happened_d;
   wire 			    hdmi_vsync_happened_rising;

   wire [23:0] 			    timing_difference;
   always @(posedge hdmi_pclk or posedge rstin) begin
      if( rstin || reset_lock_machine ) begin
	 current_genlock_delay <= 32'h40000; // get us a little closer to lock, but not all the way there
	 // this constant picked to not exceed final length for lowest resolution mode

	 genlock_count <= 0;

	 measured_vsync_to_vsync <= 0;
	 hdmi_vsync_happened <= 0;
	 hdmi_vsync_happened_d <= 0;
	 locked <= 0;
      end else begin // if ( rstin || reset_lock_machine )
	 // make a counter, starting at rising edge of vsync, to count time till genlock pulse
	 if(hdmi_vsync_rising) begin
	    genlock_count <= 0;
	 end else begin
	    genlock_count <= genlock_count + 24'b1;
	 end

	 // generate a genlock pulse when the count is between the current delay target and the
	 // specified pulse length
	 if( (genlock_count > current_genlock_delay) && 
	     (genlock_count < current_genlock_delay + GENLOCK_LENGTH) ) begin
	    // a flaw of this scheme is that we're always hitting genlock. This means we are
	    // building the jitter of the interrupt latency of the PXA168 into the loop every
	    // time we hit this. A potentially smarter way to do this is to eventually turn off
	    // the genlock interrupt and let the PXA168 LCD controller free-wheel once the
	    // delay is trimmed. This works because the controller and the HDMI stream are
	    // fully synchronous, so you will get no cumulative slip due to long term frequency
	    // offset between the two streams. However, a trivial circuit that just gates
	    // genlock based upon the "lock" bit comupted below will cause, on the next frame,
	    // vsync to come a full interrupt latency period earlier, which would "unlock"
	    // the circuit. In order to counter this, we can possibly just open up the lock
	    // state computation sufficiently wide so as to encompass the interrupt latency
	    // of the PXA168 -- but this is a little bit tricky in terms of loop dynamics
	    // so initially, we'll do a naive implementation and see if it's good enough.
	    if( smartlock_on && !locked ) begin
	       genlock <= 1'b1;
	    end else if( smartlock_on && locked ) begin
	       genlock <= 1'b0;
	    end else begin
	       genlock <= 1'b1;
	    end
	 end else begin
	    genlock <= 1'b0;
	 end

	 // this counter measures the time between when the LCD vsync pulse actually rises
	 // and when the HDMI vsync pulse actually rises.
	 // It resets on the rising edge of the LCD vsync pulse, and stops counting when
	 // the hdmi vsync pulse rises
	 if( lcd_vsync_rising__hdmi ) begin
	    measured_vsync_to_vsync <= 0;
	    hdmi_vsync_happened <= 0;
	 end else begin
	    if( !hdmi_vsync_happened ) begin
	       measured_vsync_to_vsync <= measured_vsync_to_vsync + 24'b1;
	    end else begin
	       measured_vsync_to_vsync <= measured_vsync_to_vsync;
	    end

	    if( hdmi_vsync_rising ) begin
	       hdmi_vsync_happened <= 1;
	    end else begin
	       hdmi_vsync_happened <= hdmi_vsync_happened;
	    end
	 end // else: !if( lcd_vsync_rising__hdmi )
	 hdmi_vsync_happened_d <= hdmi_vsync_happened;

	 // once the HDMI vsync pulse happens, and the difference counter has stopped,
	 // we can measure the value. If the difference is too big, take half the difference
	 // and add it to the current target. The difference is considered in absolute value
	 // so we need to pay attention to the sign in this logic, and also provide for
	 // a "zero state" that's fully locked.
	 if( hdmi_vsync_happened_rising ) begin
	    if( measured_vsync_to_vsync > target_lead_pixels ) begin
	       // in this case, LCD vsync is happening too early, so lengthen genlock delay
	       current_genlock_delay[23:0] <= current_genlock_delay[23:0] + {1'b0,timing_difference[23:1]};
	    end else if (measured_vsync_to_vsync < target_lead_pixels ) begin
	       // + becomes - in 2's compliment represenation with 1's extension...
	       current_genlock_delay[23:0] <= current_genlock_delay[23:0] + {1'b1,timing_difference[23:1]};
	    end else begin
	       current_genlock_delay <= current_genlock_delay;
	    end
	 end // if ( hdmi_vsync_happened_rising )

	 // In practice, because we are dividing by two to generate the offset, the
	 // lock can dither by a pixel around a zero point. 
	 // So, compute "locked" based upon a bit of a slop to avoid dithering.
	 // The few-pixel offset is fully absorbed by the FIFO that bridges the
	 // LCD to the HDMI domain.
	 if( timing_difference[23] ) begin // negative
	    // technically, I've done a 1's compliment below, but who's counting?
	    // it's a tolerance band anyways that will be large relative to 1 typically,
	    // so save the adder.
	    if( timing_difference[23:0] > {8'b1,~lock_tolerance[15:0]} ) begin
	       locked <= 1;
	    end else begin
	       locked <= 0;
	    end
	 end else begin
	    if( timing_difference[23:0] < {8'b0,lock_tolerance[15:0]} ) begin
	       locked <= 1;
	    end else begin
	       locked <= 0;
	    end
	 end // else: !if( timing_difference[23] )
	 
      end // else: !if( rstin || reset_lock_machine )
   end // always @ (posedge hdmi_pclk)
   assign hdmi_vsync_happened_rising = hdmi_vsync_happened & !hdmi_vsync_happened_d;
   assign timing_difference[23:0] = measured_vsync_to_vsync[23:0] - target_lead_pixels[23:0];

   reg hdmi_de_falling_d1;
   reg hdmi_de_falling_d2;
   ////////////////////////////////
   ///////// line update machine
   ////////////////////////////////
   assign advance_write = lcd_de_falling && !lines_full; 
   
   always @(posedge hdmi_pclk) begin
      hdmi_de_falling_d1 <= hdmi_de_falling; 
      hdmi_de_falling_d2 <= hdmi_de_falling;
   end
   // advance a little bit after DE is done, to give the fifo time to asynch reset
   assign advance_read = hdmi_de_falling_d2 & !lines_empty; // always assume data is ready for us...this may not be true

   assign hdmi_en = hdmi_de;

   assign lcd_en = lcd_de;
   
   //////////
   // Below is the line FIFO mechanism
   //
   // interfaces:
   // lines_full / lines_empty
   // hdmi_vsync_v -- resets the write and read trackers
   // advance_read -- advance the FIFO one line on the read side
   // advance_write -- advance the FIFO one line on the write side
   // hdmi_en -- read from the fifo
   // lcd_en -- write to the fifo
   //
   // write_init_level, read_init_level -- initial pointer settings for fifos
   // line_full_level, line_empty_level -- set high/low water mark for fifos
   
   /////////////////////////////////
   //// line-level empty/full detectors
   /////////////////////////////////
   reg lines_empty;
   reg lines_full;

   // full case would be:
   //                  --> roll direction
   // write_tracker:  00100000
   // read_tracker:   00010000
   // in other words, if you were to advance the write tracker one more
   // position, you'd start writing the actively read line.
   always @(posedge lcd_dclk) begin
      case (line_full_level)
	3'b000: begin // +2
	   if( write_tracker == {read_tracker[(NUMLINES-4) : 0], 
				 read_tracker[(NUMLINES-1) : (NUMLINES-3)]} ) begin
	      lines_full <= 1;
	   end else begin
	      lines_full <= 0;
	   end
	end
	3'b001: begin // +1
	   if( write_tracker == {read_tracker[(NUMLINES-3) : 0], 
				 read_tracker[(NUMLINES-1) : (NUMLINES-2)]} ) begin
	      lines_full <= 1;
	   end else begin
	      lines_full <= 0;
	   end
	end
	3'b010: begin // nominal
	   if( write_tracker == {read_tracker[(NUMLINES-2) : 0], read_tracker[NUMLINES-1]} ) begin
	      lines_full <= 1;
	   end else begin
	      lines_full <= 0;
	   end
	end
	3'b011: begin // -1
	   if( write_tracker == read_tracker ) begin
	      lines_full <= 1;
	   end else begin
	      lines_full <= 0;
	   end
	end
	3'b100: begin // -2
	   if( write_tracker == {read_tracker[0], read_tracker[NUMLINES-1 : 1]} ) begin
	      lines_full <= 1;
	   end else begin
	      lines_full <= 0;
	   end
	end
	default: begin
	   lines_full <= 0; // disable the tracker
	end
      endcase // case (line_full_level)
   end

   // empty case would be:
   //                  --> roll direction
   // write_tracker:  00001000
   // read_tracker:   00010000
   // in other words, if you were to advance the read tracker one more
   // position, you'd start reading the actively written line.
   always @(posedge hdmi_pclk) begin
      case (line_empty_level)
	3'b000: begin // +2
	   if( write_tracker == {read_tracker[2:0], read_tracker[(NUMLINES-1) : 3]} ) begin
	      lines_empty <= 1;
	   end else begin
	      lines_empty <= 0;
	   end
	end
	3'b001: begin // +1
	   if( write_tracker == {read_tracker[1:0], read_tracker[(NUMLINES-1) : 2]} ) begin
	      lines_empty <= 1;
	   end else begin
	      lines_empty <= 0;
	   end
	end
	3'b010: begin // nominal
	   if( write_tracker == {read_tracker[0], read_tracker[(NUMLINES-1) : 1]} ) begin
	      lines_empty <= 1;
	   end else begin
	      lines_empty <= 0;
	   end
	end
	3'b011: begin // -1
	   if( write_tracker == read_tracker ) begin
	      lines_empty <= 1;
	   end else begin
	      lines_empty <= 0;
	   end
	end
	3'b100: begin // -2
	   if( write_tracker == {read_tracker[(NUMLINES-2) : 0], read_tracker[NUMLINES-1]} ) begin
	      lines_empty <= 1;
	   end else begin
	      lines_empty <= 0;
	   end
	end
	default: begin
	   lines_empty <= 0;
	end
      endcase // case (line_empty_level)
   end

   /////////////////////////////////
   ///// active line tracking bits
   /////////////////////////////////
   parameter NUMLINES = 4'h8;
   
   reg [(NUMLINES-1) : 0] read_tracker;
   reg [(NUMLINES-1) : 0] write_tracker;

   // rolls to the "right"
   // init at "empty" state
   // reset line tracking at every hdmi vsync period, regardless of clock domai
   always @(posedge lcd_dclk or posedge rstin) begin
      if (rstin | hdmi_vsync_v) begin
//	 write_tracker[0] <= 1;
//	 write_tracker[1] <= 0;
	 write_tracker[3:0] <= write_init_level;
	 write_tracker[(NUMLINES-1):4] <= 0;
      end else if (advance_write) begin
	 write_tracker <= {write_tracker[0], write_tracker[(NUMLINES-1) : 1]};
      end
   end

   always @(posedge hdmi_pclk or posedge rstin) begin
      if (rstin | hdmi_vsync_v) begin
//	 read_tracker[0] <= 0;
//	 read_tracker[1] <= 1;
	 read_tracker[3:0] <= read_init_level;
	 read_tracker[(NUMLINES-1):4] <= 0;
      end else if (advance_read) begin
	 read_tracker <= {read_tracker[0], read_tracker[(NUMLINES-1) : 1]};
      end
   end

   //////
   //// fifo output mux
   //////
   reg [17:0] fifo_muxout;
   always @(read_tracker, fifo_dout0, fifo_dout1, fifo_dout2, fifo_dout3,
	    fifo_dout4, fifo_dout5, fifo_dout6, fifo_dout7) begin
      case( read_tracker ) // synthesis parallel_case full_case
	8'b00000001: fifo_muxout = fifo_dout0;
	8'b00000010: fifo_muxout = fifo_dout1;
	8'b00000100: fifo_muxout = fifo_dout2;
	8'b00001000: fifo_muxout = fifo_dout3;
	8'b00010000: fifo_muxout = fifo_dout4;
	8'b00100000: fifo_muxout = fifo_dout5;
	8'b01000000: fifo_muxout = fifo_dout6;
	8'b10000000: fifo_muxout = fifo_dout7;
      endcase // case ( read_tracker )
   end // always @ (read_tracker, fifo_dout0, fifo_dout1, fifo_dout2, fifo_dout3,...
   assign fifo_read[(NUMLINES-1) : 0] = read_tracker[(NUMLINES-1) : 0];

   assign fifo_write[(NUMLINES-1) : 0] = write_tracker[(NUMLINES-1) : 0];

   //// final connections to hdmi path
   // add pipeline register here
   always @(posedge hdmi_pclk or posedge rstin) begin
      if( rstin ) begin
	 hdmi_g <= 8'b0;
	 hdmi_b <= 8'b0;
	 hdmi_r <= 8'b0;
      end else begin
	 {hdmi_b[7:2],hdmi_g[7:2],hdmi_r[7:2]} <= fifo_muxout[17:0];
	 // duplicate LSB's so as to not offset colors by the LSB amount
//	 hdmi_g[1:0] <= {hdmi_g[2],hdmi_g[2]};
//       hdmi_r[1:0] <= {hdmi_r[2],hdmi_r[2]};
//	 hdmi_b[1:0] <= {hdmi_b[2],hdmi_b[2]};
	 // the world is a zany place. Turns out TVs compensate for zero'd LSBs, 
	 // and if you try to fix this it breaks color matching
	 hdmi_g[1:0] <= 2'b0;
	 hdmi_r[1:0] <= 2'b0;
	 hdmi_b[1:0] <= 2'b0;
      end // else: !if( rstin )
   end // always @ (posedge hdmi_pclk or posedge rstin)
   
   
   /////////////////////////////////
   ////// line fifos
   ////// need to manually instantiate every instance, I don't think you can do array
   ////// instantiations...
   /////////////////////////////////

   ////// warning: hard-coded against NUMLINES....
   // basically, reset myself if and only if I am the active line, and the read just finished.

   assign fifo_din = {lcd_b[5:0], lcd_g[5:0], lcd_r[5:0]};

   assign fifo_rst[0] = fifo_read[0] & hdmi_de_falling; // this is not parameterized
   assign fifo_rst[1] = fifo_read[1] & hdmi_de_falling;
   assign fifo_rst[2] = fifo_read[2] & hdmi_de_falling;
   assign fifo_rst[3] = fifo_read[3] & hdmi_de_falling;
   assign fifo_rst[4] = fifo_read[4] & hdmi_de_falling;
   assign fifo_rst[5] = fifo_read[5] & hdmi_de_falling;
   assign fifo_rst[6] = fifo_read[6] & hdmi_de_falling;
   assign fifo_rst[7] = fifo_read[7] & hdmi_de_falling;
   
   fifo_2kx18 line_fifo0(
			 .rst(rstin | fifo_rst[0] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[0] & !fifo_full[0] & lcd_en),
			 .wr_en(fifo_write[0] & lcd_en),
//			 .full(fifo_full[0]),
//			 .overflow(fifo_over[0]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[0] & !fifo_empty[0] & hdmi_en),
			 .rd_en(fifo_read[0] & hdmi_en),
			 .dout(fifo_dout0[17:0])
//			 .empty(fifo_empty[0]),
//			 .underflow(fifo_under[0])
			 );
   
   fifo_2kx18 line_fifo1(
			 .rst(rstin | fifo_rst[1] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[1] & !fifo_full[1] & lcd_en),
			 .wr_en(fifo_write[1] & lcd_en),
//			 .full(fifo_full[1]),
//			 .overflow(fifo_over[1]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[1] & !fifo_empty[1] & hdmi_en),
			 .rd_en(fifo_read[1] & hdmi_en),
			 .dout(fifo_dout1[17:0])
//			 .empty(fifo_empty[1]),
//			 .underflow(fifo_under[1])
			 );
   
   fifo_2kx18 line_fifo2(
			 .rst(rstin | fifo_rst[2] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[2] & !fifo_full[2] & lcd_en),
			 .wr_en(fifo_write[2] & lcd_en),
//			 .full(fifo_full[2]),
//			 .overflow(fifo_over[2]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[2] & !fifo_empty[2] & hdmi_en),
			 .rd_en(fifo_read[2] & hdmi_en),
			 .dout(fifo_dout2[17:0])
//			 .empty(fifo_empty[2]),
//			 .underflow(fifo_under[2])
			 );
   
   fifo_2kx18 line_fifo3(
			 .rst(rstin | fifo_rst[3] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[3] & !fifo_full[3] & lcd_en),
			 .wr_en(fifo_write[3] & lcd_en),
//			 .full(fifo_full[3]),
//			 .overflow(fifo_over[3]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[3] & !fifo_empty[3] & hdmi_en),
			 .rd_en(fifo_read[3] & hdmi_en),
			 .dout(fifo_dout3[17:0])
//			 .empty(fifo_empty[3]),
//			 .underflow(fifo_under[3])
			 );
   
   fifo_2kx18 line_fifo4(
			 .rst(rstin | fifo_rst[4] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[4] & !fifo_full[4] & lcd_en),
			 .wr_en(fifo_write[4] & lcd_en),
//			 .full(fifo_full[4]),
//			 .overflow(fifo_over[4]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[4] & !fifo_empty[4] & hdmi_en),
			 .rd_en(fifo_read[4] & hdmi_en),
			 .dout(fifo_dout4[17:0])
//			 .empty(fifo_empty[4]),
//			 .underflow(fifo_under[4])
			 );
   
   fifo_2kx18 line_fifo5(
			 .rst(rstin | fifo_rst[5] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[5] & !fifo_full[5] & lcd_en),
			 .wr_en(fifo_write[5] & lcd_en),
//			 .full(fifo_full[5]),
//			 .overflow(fifo_over[5]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[5] & !fifo_empty[5] & hdmi_en),
			 .rd_en(fifo_read[5] & hdmi_en),
			 .dout(fifo_dout5[17:0])
//			 .empty(fifo_empty[5]),
//			 .underflow(fifo_under[5])
			 );
   
   fifo_2kx18 line_fifo6(
			 .rst(rstin | fifo_rst[6] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[6] & !fifo_full[6] & lcd_en),
			 .wr_en(fifo_write[6] & lcd_en),
//			 .full(fifo_full[6]),
//			 .overflow(fifo_over[6]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[6] & !fifo_empty[6] & hdmi_en),
			 .rd_en(fifo_read[6] & hdmi_en),
			 .dout(fifo_dout6[17:0])
//			 .empty(fifo_empty[6]),
//			 .underflow(fifo_under[6])
			 );
   
   fifo_2kx18 line_fifo7(
			 .rst(rstin | fifo_rst[7] | hdmi_vsync_v),
		       
			 .wr_clk(lcd_dclk),
			 .din(fifo_din[17:0]),
//			 .wr_en(fifo_write[7] & !fifo_full[7] & lcd_en),
			 .wr_en(fifo_write[7] & lcd_en),
//			 .full(fifo_full[7]),
//			 .overflow(fifo_over[7]),
			 
			 .rd_clk(hdmi_pclk),
//			 .rd_en(fifo_read[7] & !fifo_empty[7] & hdmi_en),
			 .rd_en(fifo_read[7] & hdmi_en),
			 .dout(fifo_dout7[17:0])
//			 .empty(fifo_empty[7]),
//			 .underflow(fifo_under[7])
			 );

   assign dummy = 1'b0;
//   assign dummy = !( (fifo_under[(NUMLINES-1) : 0] != 0) || lines_empty);
   // trigger LED flash if we see any fifo go underflow, or if the overall line structure is empty

   /////////////////////////////////////
   ///// jellybean routines (synchronizers, rising edge finders, etc. etc.)
   /////////////////////////////////////
   // clean up the sync signals, as they are invalid
   // outside of control periods and have a programmable polarity
   always @(posedge hdmi_pclk or posedge rstin) begin
      if( rstin ) begin
	 hdmi_hsync_v <= 0;
	 hdmi_vsync_v <= 0;

	 hdmi_hsync_v2 <= 0;
	 hdmi_vsync_v2 <= 0;

	 hdmi_de_d <= 0;

	 hdmi_first_de <= 0;
	 hdmi_first_de_state <= 0;

	 lcd_vsync_1hdmi <= 0;
	 lcd_vsync_2hdmi <= 0;
	 lcd_vsync_hdmi <= 0;
      end else begin
	 hdmi_de_d <= hdmi_de;
	 if( hdmi_cv ) begin
	    hdmi_hsync_v <= hdmi_hsync ^ !hdmi_sync_pol;
	    hdmi_vsync_v <= hdmi_vsync ^ !hdmi_sync_pol;
	 end else begin
	    hdmi_hsync_v <= hdmi_hsync_v;
	    hdmi_vsync_v <= hdmi_vsync_v;
	 end

	 hdmi_hsync_v2 <= hdmi_hsync_v; // just a delayed version
	 hdmi_vsync_v2 <= hdmi_vsync_v; // just a delayed version

	 if( hdmi_vsync_v ) begin
	    hdmi_first_de <= 0;
	    hdmi_first_de_state <= 0;
	 end else begin
	    if( hdmi_de_rising && (hdmi_first_de_state == 0) ) begin
	       hdmi_first_de <= 1;
	       hdmi_first_de_state <= 1;
	    end else begin
	       hdmi_first_de <= 0;
	       hdmi_first_de_state <= hdmi_first_de_state;
	    end
	 end // else: !if( hdmi_vsync_v )

	 lcd_vsync_2hdmi <= lcd_vsync;
	 lcd_vsync_1hdmi <= lcd_vsync_2hdmi;
	 lcd_vsync_hdmi <= lcd_vsync_1hdmi;
      end // else: !if( rstin )
   end // always @ (posedge hdmi_pclk or posedge rstin)
   assign hdmi_hsync_rising = hdmi_hsync_v && !hdmi_hsync_v2;
   assign hdmi_vsync_rising = hdmi_vsync_v && !hdmi_vsync_v2;
   assign hdmi_de_rising = hdmi_de && !hdmi_de_d;
   assign hdmi_de_falling = !hdmi_de && hdmi_de_d;
   assign lcd_vsync_rising__hdmi = !lcd_vsync_hdmi & lcd_vsync_1hdmi;
   
   reg lcd_vsync_rising;
   reg lcd_vsync_falling;
   reg lcd_vsync_d;
   
   assign lcd_de_vsync = lcd_de & !(lcd_vsync ^ !lcd_sync_pol);
   // utility to find rising edges
   always @(posedge lcd_dclk or posedge rstin) begin
      if(rstin) begin
	 hdmi_hsync_rising__lcd <= 0;
	 hdmi_vsync_rising__lcd <= 0;
	 hdmi_vsync_falling__lcd <= 0;
	 hdmi_hsync_d__lcd <= 0;
	 hdmi_vsync_d__lcd <= 0;

	 lcd_de_d <= 0;
	 lcd_de_rising <= 0;

	 lcd_vsync_rising <= 0;
	 lcd_vsync_d <= 0;
	 lcd_de_rising <= 0;

	 lcd_hsync_d <= 0;
      end else begin // if (rstin)
	 lcd_hsync_d <= lcd_hsync ^ !lcd_sync_pol;
	 lcd_hsync_rising <= (lcd_hsync ^ !lcd_sync_pol) && !lcd_hsync_d;
	 
	 lcd_vsync_d <= lcd_vsync ^ !lcd_sync_pol;
	 lcd_vsync_rising <= (lcd_vsync ^ !lcd_sync_pol) && !lcd_vsync_d;
	 lcd_vsync_falling <= !(lcd_vsync ^ !lcd_sync_pol) && lcd_vsync_d;
			    
	 hdmi_hsync_d__lcd <= hdmi_hsync__lcd;
	 hdmi_vsync_d__lcd <= hdmi_vsync__lcd;
	 lcd_de_d <= lcd_de_vsync;

	 if( hdmi_hsync__lcd && !hdmi_hsync_d__lcd )
	   hdmi_hsync_rising__lcd <= 1;
	 else
	   hdmi_hsync_rising__lcd <= 0;

	 if( hdmi_vsync__lcd && !hdmi_vsync_d__lcd )
	   hdmi_vsync_rising__lcd <= 1;
	 else
	   hdmi_vsync_rising__lcd <= 0;

	 if( !hdmi_vsync__lcd && hdmi_vsync_d__lcd )
	   hdmi_vsync_falling__lcd <= 1;
	 else
	   hdmi_vsync_falling__lcd <= 0;

	 if( lcd_de_vsync && !lcd_de_d )
	   lcd_de_rising <= 1;
	 else
	   lcd_de_rising <= 0;

	 if( !lcd_de_vsync && lcd_de_d )
	   lcd_de_falling <= 1;
	 else
	   lcd_de_falling <= 0;
      end // else: !if(rstin)
   end // always @ (posedge lcd_dclk or posedge rstin)

   always @(posedge hdmi_pclk or posedge rstin) begin
      if(rstin) begin
	 lcd_de_d__hdmi <= 0;
      end else begin
	 lcd_de_d__hdmi <= lcd_de__hdmi;
      end
   end
   assign lcd_de_rising__hdmi = lcd_de__hdmi && !lcd_de_d__hdmi;
   assign lcd_de_falling__hdmi = !lcd_de__hdmi && lcd_de_d__hdmi;
	 
   // cross-domain synchronization
   always @(posedge lcd_dclk or posedge rstin) begin
      if(rstin) begin
	 hdmi_vsync_s__lcd <= 0;
	 hdmi_vsync__lcd <= 0;
	 hdmi_hsync_s__lcd <= 0;
	 hdmi_hsync__lcd <= 0;
      end else begin
	 hdmi_vsync_s__lcd <= hdmi_vsync_v;
	 hdmi_vsync__lcd <= hdmi_vsync_s__lcd;
	 hdmi_hsync_s__lcd <= hdmi_hsync_v;
	 hdmi_hsync__lcd <= hdmi_hsync_s__lcd;
      end // else: !if(rstin)
   end // always @ (posedge lcd_dclk or posedge rstin)

   always @(posedge hdmi_pclk or posedge rstin) begin
      if(rstin) begin
	 lcd_de_s__hdmi <= 0;
	 lcd_de__hdmi <= 0;

	 lcd_overflow_s__hdmi <= 0;
	 lcd_overflow__hdmi <= 0;
      end else begin
	 lcd_de_s__hdmi <= lcd_de_vsync;
	 lcd_de__hdmi <= lcd_de_s__hdmi;

	 lcd_overflow_s__hdmi <= lcd_overflow;
	 lcd_overflow__hdmi <= lcd_overflow_s__hdmi;
      end
   end // always @ (posedge hdmi_pclk or posedge rstin)

endmodule // lcd_input
