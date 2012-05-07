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
// An I2C bus snooper implementation. Oversampled for robustness.
//
// There are two versions, the EDID snooper and the HDCP snooper
//
// It's split because EDID records can be very large and the compiler should
// infer a block ram for the large records. However, the nature of the HDCP
// registers would cause the compiler to infer slice registers. Thus, this code
// is identical to the HDCP snoop with the exception that the HDCP read ports
// are removed which will allow the compiler to properly infer a LUT RAM.
///////////
`timescale 1 ns / 1 ps

module i2c_squash_edid (
		  // external host interface
		  input wire   SCL,      // the SCL pin state
		  input wire   SDA,
		  output reg  SDA_pu,  // overrides for SDA
		  output reg  SDA_pd,

		  input wire   clk,     // internal FPGA clock
		  input wire   reset,   // internal FPGA reset
		  // i2c configuration
		  input wire [7:0] i2c_snoop_addr,

			input wire [7:0] modeline_adr,
			input wire [7:0] modeline_dat,
			input wire modeline_write
		  );

   wire [7:0] modeline; // change wire -> reg if using legacy hard-coded roms
   
   /////// I2C physical layer components
   /// SDA is stable when SCL is high.
   /// If SDA moves while SCL is high, this is considered a start or stop condition.
   ///
   /// Otherwise, SDA can move around when SCL is low (this is where we suppress bits or 
   /// overdrive as needed). SDA is a wired-AND bus, so you only "drive" zero.
   ///
   /// In an oversampled implementation, a rising and falling edge de-glitcher is needed
   /// for SCL and SDA.
   ///

   // rise fall time cycles computation:
   // At 400kHz operation, 2.5us is a cycle. "chatter" from transition should be about
   // 5% of total cycle time max (just rule of thumb), so 0.125us should be the equiv
   // number of cycles.
   // For the demo board, a 25 MHz clock is provided, and 0.125us ~ 4 cycles
   // At 100kHz operation, 10us is a cycle, so 0.5us ~ 12 cycles
   parameter TRF_CYCLES = 5'd4;  // number of cycles for rise/fall time
   
   ////////////////
   ///// protocol-level state machine
   ////////////////
   parameter I2C_START     = 14'b1 << 0; // should only pass through this state for one cycle
   parameter I2C_RESTART   = 14'b1 << 1;
   parameter I2C_DADDR     = 14'b1 << 2;
   parameter I2C_ACK_DADDR = 14'b1 << 3;
   parameter I2C_ADDR      = 14'b1 << 4;
   parameter I2C_ACK_ADDR  = 14'b1 << 5;
   parameter I2C_WR_DATA   = 14'b1 << 6;
   parameter I2C_ACK_WR    = 14'b1 << 7;
   parameter I2C_END_WR    = 14'b1 << 8;
   parameter I2C_RD_DATA   = 14'b1 << 9;
   parameter I2C_ACK_RD    = 14'b1 << 10;
   parameter I2C_END_RD    = 14'b1 << 11;
   parameter I2C_END_RD2   = 14'b1 << 12;
   parameter I2C_WAITSTOP  = 14'b1 << 13;
   
   parameter I2C_nSTATES = 14;

   reg [(I2C_nSTATES-1):0]     I2C_cstate = {{(I2C_nSTATES-1){1'b0}}, 1'b1};  //current and next states
   reg [(I2C_nSTATES-1):0]     I2C_nstate;

//`define SIMULATION  
`ifdef SIMULATION
   // synthesis translate_off
   reg [8*20:1] 	                 I2C_state_ascii = "I2C_START          ";
   always @(I2C_cstate) begin
      if      (I2C_cstate == I2C_START)     I2C_state_ascii <= "I2C_START          ";
      else if (I2C_cstate == I2C_RESTART)   I2C_state_ascii <= "I2C_RESTART        ";
      else if (I2C_cstate == I2C_DADDR)     I2C_state_ascii <= "I2C_DADDR          ";
      else if (I2C_cstate == I2C_ACK_DADDR) I2C_state_ascii <= "I2C_ACK_DADDR      ";
      else if (I2C_cstate == I2C_ADDR)      I2C_state_ascii <= "I2C_ADDR           ";
      else if (I2C_cstate == I2C_ACK_ADDR)  I2C_state_ascii <= "I2C_ACK_ADDR       ";
      else if (I2C_cstate == I2C_WR_DATA)   I2C_state_ascii <= "I2C_WR_DATA        ";
      else if (I2C_cstate == I2C_ACK_WR)    I2C_state_ascii <= "I2C_ACK_WR         ";
      else if (I2C_cstate == I2C_END_WR)    I2C_state_ascii <= "I2C_END_WR         ";
      else if (I2C_cstate == I2C_RD_DATA)   I2C_state_ascii <= "I2C_RD_DATA        ";
      else if (I2C_cstate == I2C_ACK_RD)    I2C_state_ascii <= "I2C_ACK_RD         ";
      else if (I2C_cstate == I2C_END_RD)    I2C_state_ascii <= "I2C_END_RD         ";
      else if (I2C_cstate == I2C_END_RD2)   I2C_state_ascii <= "I2C_END_RD2        ";
      else if (I2C_cstate == I2C_WAITSTOP)  I2C_state_ascii <= "I2C_WAITSTOP       ";
      else I2C_state_ascii                          <= "WTF                ";
   end
   // synthesis translate_on
`endif
   
   reg [3:0] 		       I2C_bitcnt;
   reg [7:0] 		       I2C_addr;
   reg [7:0] 		       I2C_daddr;
   reg 			       I2C_reg_update;
   reg [7:0] 		       I2C_squashdata;
   reg 			       I2C_dosquash;
   
   always @ (posedge clk) begin
      if (reset || ((SCL_cstate == SCL_HIGH) && (SDA_cstate == SDA_RISE))) // stop condition always resets
	I2C_cstate <= I2C_START; 
      else
	I2C_cstate <=#1 I2C_nstate;
   end

   always @ (*) begin
      case (I2C_cstate) //synthesis parallel_case full_case
	I2C_START: begin // wait for the start condition
	   I2C_nstate = ((SDA_cstate == SDA_FALL) && (SCL_cstate == SCL_HIGH)) ? I2C_DADDR : I2C_START;
	end
	I2C_RESTART: begin // repeated start moves immediately to DADDR
	   I2C_nstate = I2C_DADDR;
	end
	I2C_DADDR: begin // 8 bits to get the address
	   I2C_nstate = ((I2C_bitcnt > 4'h7) && (SCL_cstate == SCL_FALL)) ? I2C_ACK_DADDR : I2C_DADDR;
	end
	I2C_ACK_DADDR: begin // depending upon W/R bit state, go to one of two branches
	   I2C_nstate = (SCL_cstate == SCL_FALL) ?
			(I2C_daddr[7:1] == i2c_snoop_addr[7:1]) ? 
			(I2C_daddr[0] == 1'b0 ? I2C_ADDR : I2C_RD_DATA) :
			I2C_WAITSTOP : // !I2C_daddr match
			I2C_ACK_DADDR; // !SCL_FALL
	end

	// device address branch
	I2C_ADDR: begin
	   I2C_nstate = ((I2C_bitcnt > 4'h7) && (SCL_cstate == SCL_FALL)) ? I2C_ACK_ADDR : I2C_ADDR;
	end
	I2C_ACK_ADDR: begin
	   I2C_nstate = (SCL_cstate == SCL_FALL) ? I2C_WR_DATA : I2C_ACK_ADDR;
	end
	
	// write branch
	I2C_WR_DATA: begin // 8 bits to get the write data
	   I2C_nstate = ((SDA_cstate == SDA_FALL) && (SCL_cstate == SCL_HIGH)) ? I2C_RESTART : // repeated start
			((I2C_bitcnt > 4'h7) && (SCL_cstate == SCL_FALL)) ? I2C_ACK_WR : I2C_WR_DATA;
	end
	I2C_ACK_WR: begin // trigger the ack response (pull SDA low until next falling edge)
	   // and stay in this state until the next falling edge of SCL
	   I2C_nstate = (SCL_cstate == SCL_FALL) ? I2C_END_WR : I2C_ACK_WR;
	end
	I2C_END_WR: begin // one-cycle state to update address+1, reset SDA pulldown
	   I2C_nstate = I2C_WR_DATA; // SCL is now low
	end

	// read branch
	I2C_RD_DATA: begin // 8 bits to get the read data
	   I2C_nstate = ((SDA_cstate == SDA_FALL) && (SCL_cstate == SCL_HIGH)) ? I2C_RESTART : // repeated start
			((I2C_bitcnt > 4'h7) && (SCL_cstate == SCL_FALL)) ? I2C_ACK_RD : I2C_RD_DATA;
	end
	I2C_ACK_RD: begin // wait for an (n)ack response
	   // need to sample (n)ack on a rising edge
	   I2C_nstate = (SCL_cstate == SCL_RISE) ? I2C_END_RD : I2C_ACK_RD;
	end
	I2C_END_RD: begin // if nack, just go to start state (don't explicitly check stop event)
	   // single cycle state for adr+1 update
	   I2C_nstate = (SDA_cstate == SDA_LOW) ? I2C_END_RD2 : I2C_START;
	end
	I2C_END_RD2: begin // before entering I2C_RD_DATA, we need to have seen a falling edge.
	   I2C_nstate = (SCL_cstate == SCL_FALL) ? I2C_RD_DATA : I2C_END_RD2;
	end

	// we're not the addressed device, so we just idle until we see a stop
	I2C_WAITSTOP: begin
	   I2C_nstate = (((SCL_cstate == SCL_HIGH) && (SDA_cstate == SDA_RISE))) ? // stop
			I2C_START : 
			(((SCL_cstate == SCL_HIGH) && (SDA_cstate == SDA_FALL))) ? // or start
			I2C_RESTART :
			I2C_WAITSTOP;
	end
      endcase // case (cstate)
   end

   always @ (posedge clk or posedge reset) begin
      if( reset ) begin
	 I2C_bitcnt <=#1 4'b0;
	 I2C_daddr <=#1 8'b0;
	 I2C_reg_update <=#1 1'b0;
	 I2C_addr <=#1 8'b0; // this persists across transactions

	 I2C_squashdata <=#1 8'hff;
	 I2C_dosquash <=#1 0;
	 SDA_pd <=#1 0;
	 SDA_pu <=#1 0;
      end else begin
	 case (I2C_cstate) // synthesis parallel_case full_case
	   I2C_START: begin // everything in reset
	      I2C_bitcnt <=#1 4'b0;
	      I2C_daddr <=#1 8'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end

	   I2C_RESTART: begin
	      I2C_bitcnt <=#1 4'b0;
	      I2C_daddr <=#1 8'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

//	      I2C_squashdata <=#1 I2C_squashdata;
	      // on restart, I2C_addr is valid, so grab squashdata again
	      I2C_squashdata <=#1 modeline[7:0];
	      I2C_dosquash <=#1 I2C_dosquash; 
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end

	   // get my i2c device address (am I being talked to?)
	   I2C_DADDR: begin // shift in the address on rising edges of clock
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <=#1 I2C_bitcnt + 4'b1;
		 I2C_daddr[7] <=#1 I2C_daddr[6];
		 I2C_daddr[6] <=#1 I2C_daddr[5];
		 I2C_daddr[5] <=#1 I2C_daddr[4];
		 I2C_daddr[4] <=#1 I2C_daddr[3];
		 I2C_daddr[3] <=#1 I2C_daddr[2];
		 I2C_daddr[2] <=#1 I2C_daddr[1];
		 I2C_daddr[1] <=#1 I2C_daddr[0];
		 I2C_daddr[0] <=#1 (SDA_cstate == SDA_HIGH) ? 1'b1 : 1'b0;
	      end else begin // we're oversampled so we need a hold-state gutter
		 I2C_bitcnt <=#1 I2C_bitcnt;
		 I2C_daddr <=#1 I2C_daddr;
	      end // else: !if( SCL_cstate == SCL_RISE )
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end // case: I2C_DADDR
	   I2C_ACK_DADDR: begin
	      I2C_daddr <=#1 I2C_daddr;
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end

	   // get my i2c "write" address (what we want to access inside me)
	   I2C_ADDR: begin
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <=#1 I2C_bitcnt + 4'b1;
		 I2C_addr[7] <=#1 I2C_addr[6];
		 I2C_addr[6] <=#1 I2C_addr[5];
		 I2C_addr[5] <=#1 I2C_addr[4];
		 I2C_addr[4] <=#1 I2C_addr[3];
		 I2C_addr[3] <=#1 I2C_addr[2];
		 I2C_addr[2] <=#1 I2C_addr[1];
		 I2C_addr[1] <=#1 I2C_addr[0];
		 I2C_addr[0] <=#1 (SDA_cstate == SDA_HIGH) ? 1'b1 : 1'b0;
	      end else begin // we're oversampled so we need a hold-state gutter
		 I2C_bitcnt <=#1 I2C_bitcnt;
		 I2C_addr <=#1 I2C_addr;
	      end // else: !if( SCL_cstate == SCL_RISE )
	      I2C_reg_update <=#1 1'b0;
	      I2C_daddr <=#1 I2C_daddr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end // case: I2C_ADDR
	   I2C_ACK_ADDR: begin
	      I2C_daddr <=#1 I2C_daddr;
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

	      // note mirror image of this on the I2C_END_RD2 branch
`ifdef POSTERITY
	      if( I2C_addr[7] ) begin // 0x80 and above
		 I2C_squashdata <=#1 modeline[7:0];
		 I2C_dosquash <=#1 1;
	      end else if( I2C_addr[7:0] == 8'h75 ) begin
		 I2C_squashdata <=#1 8'h09; // pixclock @ 90mhz
		 I2C_dosquash <=#1 1;
	      end else if( (I2C_addr[7:0] >= 8'h36) &&
			   (I2C_addr[7:0] <= 8'h47) ) begin
		 I2C_squashdata <=#1 8'h00;
		 I2C_dosquash <=#1 1;
	      end else if( I2C_addr[7:0] == 8'h7f ) begin
		 I2C_squashdata <=#1 8'hb7; // fixup checkusm
		 I2C_dosquash <=#1 1;
	      end else begin
		 I2C_squashdata <=#1 I2C_squashdata;
		 I2C_dosquash <=#1 0;
	      end
`endif //  `ifdef POSTERITY
	      // now that bank is 256 bytes, squash everything
	      I2C_squashdata <=#1 modeline[7:0];
	      I2C_dosquash <=#1 1;
	      
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end
	   
	   
	   // write branch
	   I2C_WR_DATA: begin // shift in data on rising edges of clock
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <=#1 I2C_bitcnt + 4'b1;
	      end else begin
		 I2C_bitcnt <=#1 I2C_bitcnt; // hold state gutter
	      end // else: !if( SCL_cstate == SCL_RISE )
	      I2C_daddr <=#1 I2C_daddr;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;
`ifdef SQUASHWRITES
	      if( SCL_cstate == SCL_FALL ) begin
		 I2C_squashdata[7:0] <=#1 {I2C_squashdata[6:0],I2C_squashdata[7]};
	      end else begin
		 I2C_squashdata <=#1 I2C_squashdata;
	      end
	      
	      I2C_dosquash <=#1 I2C_dosquash;
	      if( I2C_dosquash ) begin
		 if( I2C_squashdata[7] ) begin
		    SDA_pd <=#1 0;
		    SDA_pu <=#1 1;
		 end else begin
		    SDA_pd <=#1 1;
		    SDA_pu <=#1 0;
		 end
	      end else begin
		 SDA_pd <=#1 0;
		 SDA_pu <=#1 0;
	      end // else: !if( I2C_dosquash )
`else // !`ifdef SQUASHWRITES
	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
`endif	      
	   end // case: I2C_WR_DATA
	   I2C_ACK_WR: begin
	      I2C_daddr <=#1 I2C_daddr;
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b1;  // write the data now (over and over again while in state)
	      I2C_addr <=#1 I2C_addr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end
	   I2C_END_WR: begin
	      I2C_addr <=#1 I2C_addr + 8'b1; // this is a one-cycle state so this is safe
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_daddr <=#1 I2C_daddr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end

	   // read branch
	   I2C_RD_DATA: begin // shift out data on falling edges of clock
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <=#1 I2C_bitcnt + 4'b1;
	      end else begin
		 I2C_bitcnt <=#1 I2C_bitcnt; // hold state gutter
	      end
	      I2C_daddr <=#1 I2C_daddr;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

	      if( SCL_cstate == SCL_FALL ) begin
		 I2C_squashdata[7:0] <=#1 {I2C_squashdata[6:0],I2C_squashdata[7]};
	      end else begin
		 I2C_squashdata <=#1 I2C_squashdata;
	      end
	      
	      I2C_dosquash <=#1 I2C_dosquash;
	      // the purpose of dosquash was to do partial-bank squashing
	      // however, we should *always* squash. There is a logic bug discovered with the
	      // motorola DCH-3200 cable box where a repeated start was de-activating squashing
	      // for the very first byte in a multi-byte sequence. So, probably the logic
	      // for controlling I2C_dosquash is now considered legacy and non-functional
	      // an instead we short-circuit it here by forcing the if to always evaluate true
	      if( /*I2C_dosquash*/ 1'b1 ) begin  // we are always going to be squashing
		 if( I2C_squashdata[7] ) begin
		    SDA_pd <=#1 0;
		    SDA_pu <=#1 1;
		 end else begin
		    SDA_pd <=#1 1;
		    SDA_pu <=#1 0;
		 end
	      end else begin
		 SDA_pd <=#1 0;
		 SDA_pu <=#1 0;
	      end
	   end // case: I2C_RD_DATA
	   I2C_ACK_RD: begin
	      I2C_daddr <=#1 I2C_daddr;
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b1; // commit reads even to our internal bank
	      I2C_addr <=#1 I2C_addr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end
	   I2C_END_RD: begin
	      I2C_addr <=#1 I2C_addr + 8'b1; // this is a one-cycle state so this is safe
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_daddr <=#1 I2C_daddr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end
	   I2C_END_RD2: begin
	      I2C_daddr <=#1 8'b0;
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

`ifdef POSTERITY
	      if( I2C_addr[7] ) begin // 0x80 and above
		 I2C_squashdata <=#1 modeline[7:0];
		 I2C_dosquash <=#1 1;
	      end else if( I2C_addr[7:0] == 8'h75 ) begin
		 I2C_squashdata <=#1 8'h09; // pixclock @ 90mhz
		 I2C_dosquash <=#1 1;
	      end else if( (I2C_addr[7:0] >= 8'h36) &&
			   (I2C_addr[7:0] <= 8'h47) ) begin
		 I2C_squashdata <=#1 8'h00;
		 I2C_dosquash <=#1 1;
	      end else if( I2C_addr[7:0] == 8'h7f ) begin
		 I2C_squashdata <=#1 8'hb7; // fixup checkusm
		 I2C_dosquash <=#1 1;
	      end else begin
		 I2C_squashdata <=#1 I2C_squashdata;
		 I2C_dosquash <=#1 0;
	      end
`endif //  `ifdef POSTERITY
	      // now that bank is 256 bytes, squash everything
	      I2C_squashdata <=#1 modeline[7:0];
	      I2C_dosquash <=#1 1;
	      
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end
	   
	   I2C_WAITSTOP: begin
	      I2C_daddr <=#1 8'b0;
	      I2C_bitcnt <=#1 4'b0;
	      I2C_reg_update <=#1 1'b0;
	      I2C_addr <=#1 I2C_addr;

	      I2C_squashdata <=#1 I2C_squashdata;
	      I2C_dosquash <=#1 0;
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
	   end
	 endcase // case (cstate)
      end // else: !if( reset )
   end // always @ (posedge clk or posedge reset)

`ifdef POSTERITY
   	      if( I2C_addr[7:0] == 8'h63 ) begin
		 I2C_squashdata <=#1 8'h0a;
		 I2C_dosquash <=#1 1;
	      end else if( (I2C_addr[7:0] >= 8'h36) && (I2C_addr[7:0] <= 8'h59) ) begin
		 I2C_squashdata <=#1 modeline[7:0];
		 I2C_dosquash <=#1 1;
	      end else begin
		 I2C_squashdata <=#1 I2C_squashdata;
		 I2C_dosquash <=#1 0;
	      end
	      SDA_pd <=#1 0;
	      SDA_pu <=#1 0;
`endif

   ////////////////
   ///// SCL low-level sampling state machine
   ////////////////
   parameter SCL_HIGH = 4'b1 << 0; // should only pass through this state for one cycle
   parameter SCL_FALL = 4'b1 << 1;
   parameter SCL_LOW  = 4'b1 << 2;
   parameter SCL_RISE = 4'b1 << 3;
   parameter SCL_nSTATES = 4;

   reg [(SCL_nSTATES-1):0]     SCL_cstate = {{(SCL_nSTATES-1){1'b0}}, 1'b1};  //current and next states
   reg [(SCL_nSTATES-1):0]     SCL_nstate;

//`define SIMULATION  
`ifdef SIMULATION
   // synthesis translate_off
   reg [8*20:1] 	                 SCL_state_ascii = "SCL_HIGH           ";

   always @(SCL_cstate) begin
      if      (SCL_cstate == SCL_HIGH)     SCL_state_ascii <= "SCL_HIGH           ";
      else if (SCL_cstate == SCL_FALL)     SCL_state_ascii <= "SCL_FALL           ";
      else if (SCL_cstate == SCL_LOW )     SCL_state_ascii <= "SCL_LOW            ";
      else if (SCL_cstate == SCL_RISE)     SCL_state_ascii <= "SCL_RISE           ";
      else SCL_state_ascii                                 <= "WTF                ";
   end
   // synthesis translate_on
`endif

   reg [4:0] 		       SCL_rfcnt;
   reg 			       SCL_s, SCL_sync;
   reg 			       SDA_s, SDA_sync;

   always @ (posedge clk or posedge reset) begin
      if (reset)
	SCL_cstate <= SCL_HIGH; // always start here even if it's wrong -- easier to test
      else
	SCL_cstate <=#1 SCL_nstate;
   end

   always @ (*) begin
      case (SCL_cstate) //synthesis parallel_case full_case
	SCL_HIGH: begin
	   SCL_nstate = ((SCL_rfcnt > TRF_CYCLES) && (SCL_sync == 1'b0)) ? SCL_FALL : SCL_HIGH;
	end
	SCL_FALL: begin
	   SCL_nstate = SCL_LOW;
	end
	SCL_LOW: begin
	   SCL_nstate = ((SCL_rfcnt > TRF_CYCLES) && (SCL_sync == 1'b1)) ? SCL_RISE : SCL_LOW;
	end
	SCL_RISE: begin
	   SCL_nstate = SCL_HIGH;
	end
      endcase // case (cstate)
   end // always @ (*)

   always @ (posedge clk or posedge reset) begin
      if( reset ) begin
	 SCL_rfcnt <=#1 5'b0;
      end else begin
	 case (SCL_cstate) // synthesis parallel_case full_case
	   SCL_HIGH: begin
	      if( SCL_sync == 1'b1 ) begin
		 SCL_rfcnt <=#1 5'b0;
	      end else begin
		 SCL_rfcnt <=#1 SCL_rfcnt + 5'b1;
	      end
	   end
	   SCL_FALL: begin
	      SCL_rfcnt <=#1 5'b0;
	   end
	   SCL_LOW: begin
	      if( SCL_sync == 1'b0 ) begin
		 SCL_rfcnt <=#1 5'b0;
	      end else begin
		 SCL_rfcnt <=#1 SCL_rfcnt + 5'b1;
	      end
	   end
	   SCL_RISE: begin
	      SCL_rfcnt <=#1 5'b0;
	   end
	 endcase // case (cstate)
      end // else: !if( reset )
   end // always @ (posedge clk or posedge reset)


   ////////////////
   ///// SDA low-level sampling state machine
   ////////////////
   parameter SDA_HIGH = 4'b1 << 0; // should only pass through this state for one cycle
   parameter SDA_FALL = 4'b1 << 1;
   parameter SDA_LOW  = 4'b1 << 2;
   parameter SDA_RISE = 4'b1 << 3;
   parameter SDA_nSTATES = 4;

   reg [(SDA_nSTATES-1):0]     SDA_cstate = {{(SDA_nSTATES-1){1'b0}}, 1'b1};  //current and next states
   reg [(SDA_nSTATES-1):0]     SDA_nstate;

//`define SIMULATION  
`ifdef SIMULATION
   // synthesis translate_off
   reg [8*20:1] 	                 SDA_state_ascii = "SDA_HIGH           ";

   always @(SDA_cstate) begin
      if      (SDA_cstate == SDA_HIGH)     SDA_state_ascii <= "SDA_HIGH           ";
      else if (SDA_cstate == SDA_FALL)     SDA_state_ascii <= "SDA_FALL           ";
      else if (SDA_cstate == SDA_LOW )     SDA_state_ascii <= "SDA_LOW            ";
      else if (SDA_cstate == SDA_RISE)     SDA_state_ascii <= "SDA_RISE           ";
      else SDA_state_ascii                                 <= "WTF                ";
   end
   // synthesis translate_on
`endif

   reg [4:0] 		       SDA_rfcnt;

   always @ (posedge clk or posedge reset) begin
      if (reset)
	SDA_cstate <= SDA_HIGH; // always start here even if it's wrong -- easier to test
      else
	SDA_cstate <=#1 SDA_nstate;
   end

   always @ (*) begin
      case (SDA_cstate) //synthesis parallel_case full_case
	SDA_HIGH: begin
	   SDA_nstate = ((SDA_rfcnt > TRF_CYCLES) && (SDA_sync == 1'b0)) ? SDA_FALL : SDA_HIGH;
	end
	SDA_FALL: begin
	   SDA_nstate = SDA_LOW;
	end
	SDA_LOW: begin
	   SDA_nstate = ((SDA_rfcnt > TRF_CYCLES) && (SDA_sync == 1'b1)) ? SDA_RISE : SDA_LOW;
	end
	SDA_RISE: begin
	   SDA_nstate = SDA_HIGH;
	end
      endcase // case (cstate)
   end // always @ (*)

   always @ (posedge clk or posedge reset) begin
      if( reset ) begin
	 SDA_rfcnt <=#1 5'b0;
      end else begin
	 case (SDA_cstate) // synthesis parallel_case full_case
	   SDA_HIGH: begin
	      if( SDA_sync == 1'b1 ) begin
		 SDA_rfcnt <=#1 5'b0;
	      end else begin
		 SDA_rfcnt <=#1 SDA_rfcnt + 5'b1;
	      end
	   end
	   SDA_FALL: begin
	      SDA_rfcnt <=#1 5'b0;
	   end
	   SDA_LOW: begin
	      if( SDA_sync == 1'b0 ) begin
		 SDA_rfcnt <=#1 5'b0;
	      end else begin
		 SDA_rfcnt <=#1 SDA_rfcnt + 5'b1;
	      end
	   end
	   SDA_RISE: begin
	      SDA_rfcnt <=#1 5'b0;
	   end
	 endcase // case (cstate)
      end // else: !if( reset )
   end // always @ (posedge clk or posedge reset)

   
   
   /////////////////////
   /////// synchronizers
   /////////////////////
   always @ (posedge clk or posedge reset) begin
      if (reset) begin
	 SCL_s <= 0;
	 SCL_sync <= 0;
	 SDA_s <= 0;
	 SDA_sync <= 0;
      end else begin
	 SCL_s <= SCL;
	 SCL_sync <= SCL_s;
	 SDA_s <= SDA;
	 SDA_sync <= SDA_s;
      end // else: !if(reset)
   end // always @ (posedge clk or posedge reset)

   /////////////////////////
   // ram that contains the override modeline
   /////////////////////////
   (* RAM_STYLE="{AUTO | DISTRIBUTED | PIPE_DISTRIBUTED}" *)
   reg [7:0] moderam [255:0];

//   wire [7:0] modeline; // declared up top

   always @(posedge clk)
      if (modeline_write)
         moderam[modeline_adr[7:0]] <= modeline_dat[7:0];

   assign modeline[7:0] = moderam[I2C_addr[7:0]];
   
`ifdef POSTERITY
   wire [7:0] I2C_offset36;
   assign I2C_offset36[7:0] = I2C_addr[7:0] - 8'h36;

   always @(*) begin
      case( I2C_offset36[4:0] )
	5'h00: modeline = 8'h01; // master copy
	5'h01: modeline = 8'h1d;
	5'h02: modeline = 8'h00;
	5'h03: modeline = 8'hbc;
	5'h04: modeline = 8'h52;
	5'h05: modeline = 8'hd0;
	5'h06: modeline = 8'h1e;
	5'h07: modeline = 8'h20;
	5'h08: modeline = 8'hb8;
	5'h09: modeline = 8'h28;
	5'h0a: modeline = 8'h55;
	5'h0b: modeline = 8'h40;
	5'h0c: modeline = 8'ha0;
	5'h0d: modeline = 8'h5a;
	5'h0e: modeline = 8'h00;
	5'h0f: modeline = 8'h00;
	5'h10: modeline = 8'h00;
	5'h11: modeline = 8'h1e;
	
	5'h12: modeline = 8'h01; // alias copy
	5'h13: modeline = 8'h1d;
	5'h14: modeline = 8'h00;
	5'h15: modeline = 8'hbc;
	5'h16: modeline = 8'h52;
	5'h17: modeline = 8'hd0;
	5'h18: modeline = 8'h1e;
	5'h19: modeline = 8'h20;
	5'h1a: modeline = 8'hb8;
	5'h1b: modeline = 8'h28;
	5'h1c: modeline = 8'h55;
	5'h1d: modeline = 8'h40;
	5'h1e: modeline = 8'ha0;
	5'h1f: modeline = 8'h5a;
      endcase // case ( I2C_addr )
   end // always @ (*)
`endif // !`ifdef POSTERITY
`ifdef POSTERITY
   always @(*) begin
      case( I2C_addr[6:0] )
        7'h00: modeline = 8'h02;
        7'h01: modeline = 8'h03;
        7'h02: modeline = 8'h1f;
        7'h03: modeline = 8'hf2;
        7'h04: modeline = 8'h4b;
        7'h05: modeline = 8'h93;
        7'h06: modeline = 8'h04;
        7'h07: modeline = 8'h12;
        7'h08: modeline = 8'h83;
        7'h09: modeline = 8'h14;
        7'h0a: modeline = 8'h05;
        7'h0b: modeline = 8'h20;
        7'h0c: modeline = 8'h00;
        7'h0d: modeline = 8'h00;
        7'h0e: modeline = 8'h00;
        7'h0f: modeline = 8'h00;
        7'h10: modeline = 8'h23;
        7'h11: modeline = 8'h09;
        7'h12: modeline = 8'h07;
        7'h13: modeline = 8'h07;
        7'h14: modeline = 8'h83;
        7'h15: modeline = 8'h01;
        7'h16: modeline = 8'h00;
        7'h17: modeline = 8'h00;
        7'h18: modeline = 8'h66;
        7'h19: modeline = 8'h03;
        7'h1a: modeline = 8'h0c;
        7'h1b: modeline = 8'h00;
        7'h1c: modeline = 8'h10;
        7'h1d: modeline = 8'h00;
        7'h1e: modeline = 8'h80;
        7'h1f: modeline = 8'h8c;
        7'h20: modeline = 8'h0a;
        7'h21: modeline = 8'hd0;
        7'h22: modeline = 8'h8a;
        7'h23: modeline = 8'h20;
        7'h24: modeline = 8'he0;
        7'h25: modeline = 8'h2d;
        7'h26: modeline = 8'h10;
        7'h27: modeline = 8'h10;
        7'h28: modeline = 8'h3e;
        7'h29: modeline = 8'h96;
        7'h2a: modeline = 8'h00;
        7'h2b: modeline = 8'ha0;
        7'h2c: modeline = 8'h5a;
        7'h2d: modeline = 8'h00;
        7'h2e: modeline = 8'h00;
        7'h2f: modeline = 8'h00;
        7'h30: modeline = 8'h18;
        7'h31: modeline = 8'h01;
        7'h32: modeline = 8'h1d;
        7'h33: modeline = 8'h00;
        7'h34: modeline = 8'h72;
        7'h35: modeline = 8'h51;
        7'h36: modeline = 8'hd0;
        7'h37: modeline = 8'h1e;
        7'h38: modeline = 8'h20;
        7'h39: modeline = 8'h6e;
        7'h3a: modeline = 8'h28;
        7'h3b: modeline = 8'h55;
        7'h3c: modeline = 8'h00;
        7'h3d: modeline = 8'ha0;
        7'h3e: modeline = 8'h5a;
        7'h3f: modeline = 8'h00;
        7'h40: modeline = 8'h00;
        7'h41: modeline = 8'h00;
        7'h42: modeline = 8'h1e;
        7'h43: modeline = 8'h01;
        7'h44: modeline = 8'h1d;
        7'h45: modeline = 8'h80;
        7'h46: modeline = 8'hd0;
        7'h47: modeline = 8'h72;
        7'h48: modeline = 8'h1c;
        7'h49: modeline = 8'h16;
        7'h4a: modeline = 8'h20;
        7'h4b: modeline = 8'h10;
        7'h4c: modeline = 8'h2c;
        7'h4d: modeline = 8'h25;
        7'h4e: modeline = 8'h80;
        7'h4f: modeline = 8'ha0;
        7'h50: modeline = 8'h5a;
        7'h51: modeline = 8'h00;
        7'h52: modeline = 8'h00;
        7'h53: modeline = 8'h00;
        7'h54: modeline = 8'h9e;
        7'h55: modeline = 8'h01;
        7'h56: modeline = 8'h1d;
        7'h57: modeline = 8'h80;
        7'h58: modeline = 8'h18;
        7'h59: modeline = 8'h71;
        7'h5a: modeline = 8'h1c;
        7'h5b: modeline = 8'h16;
        7'h5c: modeline = 8'h20;
        7'h5d: modeline = 8'h58;
        7'h5e: modeline = 8'h2c;
        7'h5f: modeline = 8'h25;
        7'h60: modeline = 8'h00;
        7'h61: modeline = 8'ha0;
        7'h62: modeline = 8'h5a;
        7'h63: modeline = 8'h00;
        7'h64: modeline = 8'h00;
        7'h65: modeline = 8'h00;
        7'h66: modeline = 8'h9e;
        7'h67: modeline = 8'h8c;
        7'h68: modeline = 8'h0a;
        7'h69: modeline = 8'hd0;
        7'h6a: modeline = 8'h90;
        7'h6b: modeline = 8'h20;
        7'h6c: modeline = 8'h40;
        7'h6d: modeline = 8'h31;
        7'h6e: modeline = 8'h20;
        7'h6f: modeline = 8'h0c;
        7'h70: modeline = 8'h40;
        7'h71: modeline = 8'h55;
        7'h72: modeline = 8'h00;
        7'h73: modeline = 8'ha0;
        7'h74: modeline = 8'h5a;
        7'h75: modeline = 8'h00;
        7'h76: modeline = 8'h00;
        7'h77: modeline = 8'h00;
        7'h78: modeline = 8'h18;
        7'h79: modeline = 8'h00;
        7'h7a: modeline = 8'h00;
        7'h7b: modeline = 8'h00;
        7'h7c: modeline = 8'h00;
        7'h7d: modeline = 8'h00;
        7'h7e: modeline = 8'h00;
        7'h7f: modeline = 8'ha3;
      endcase // case ( I2C_addr )
   end // always @ (*)
`endif // !`ifdef POSTERITY
   
endmodule // i2c_slave
