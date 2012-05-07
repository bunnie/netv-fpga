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
// A simple slave implementation. Oversampled for robustness.
// The slave is extended into the snoop & surpress version for the DDC bus;
// this is just a starting point for basic testing and also simple comms
// with the CPU.
//
// i2c slave module requires the top level module to implement the IOBs
// This is just to keep the tri-state easy to implemen across the hierarchy
//
// The code required on the top level is:
//   IBUF IBUF_sda (.I(SDA), .O(SDA_int));
//   IOBUF #(.DRIVE(12), .SLEW("SLOW")) IOBUF_sda (.IO(SDA), .I(1'b0), .T(!SDA_pd));
//
///////////
`timescale 1 ns / 1 ps

module i2c_slave (
		  // external host interface
		  input wire   SCL,      // the SCL pin state
		  output wire  SCL_pd,  // signals to IOB to pull the SCL bus low
		  input wire   SDA,
		  output reg   SDA_pd,
		  output wire  SDA_pu,  // for overriding SDA...in the future.

		  input wire   clk,     // internal FPGA clock
		  input wire   glbl_reset,   // internal FPGA reset
		  // i2c configuration
		  input wire [7:0] i2c_device_addr,

		  // internal slave interface
//		  input wire [7:0] reg_addr,
//		  input wire       wr_stb,
//		  input wire [7:0] reg_data_in,
		  output wire [7:0] reg_0,
		  output wire [7:0] reg_1,
		  input  wire [7:0] reg_2,
		  output wire [7:0] reg_3,

		  
		  output wire [7:0] reg_4,
		  output wire [7:0] reg_5,
		  output wire [7:0] reg_6,
		  output wire [7:0] reg_7,
		  
//		  output wire [7:0] reg_8,
//		  output wire [7:0] reg_9,
//		  output wire [7:0] reg_a,
//		  output wire [7:0] reg_b,
		  input wire [7:0] reg_8,
		  input wire [7:0] reg_9,
		  input wire [7:0] reg_a,
		  input wire [7:0] reg_b,
		  
		  output wire [7:0] reg_c,
		  output wire [7:0] reg_d,
		  output wire [7:0] reg_e,
		  output wire [7:0] reg_f,

		  input wire [7:0] reg_18, // note this is input now, not output
		  
		  output wire [7:0] reg_19,
		  output wire [7:0] reg_1a,
		  output wire [7:0] reg_1b,
		  output wire [7:0] reg_1c,
		  output wire [7:0] reg_1d,
		  output wire [7:0] reg_1e,
		  output wire [7:0] reg_1f,

		  input wire [7:0] reg_10,

		  output wire [7:0] reg_11,
		  output wire [7:0] reg_12,

		  output wire [7:0] reg_13,
		  output wire [7:0] reg_14,
		  output wire [7:0] reg_15,
		  output wire [7:0] reg_16,
		  output wire [7:0] reg_17,

		  input wire [7:0] reg_20, // read-only bank starts here
		  input wire [7:0] reg_21,
		  input wire [7:0] reg_22,
		  input wire [7:0] reg_23,
		  input wire [7:0] reg_24,
		  input wire [7:0] reg_25,
		  input wire [7:0] reg_26,
		  input wire [7:0] reg_27,
		  input wire [7:0] reg_28,
		  input wire [7:0] reg_29,
		  input wire [7:0] reg_2a,
		  input wire [7:0] reg_2b,
		  input wire [7:0] reg_2c,
		  input wire [7:0] reg_2d,
		  input wire [7:0] reg_2e,
		  input wire [7:0] reg_2f,
		  input wire [7:0] reg_30,
		  input wire [7:0] reg_31,
		  input wire [7:0] reg_32,
		  input wire [7:0] reg_33,
		  input wire [7:0] reg_34,
		  input wire [7:0] reg_35,
		  input wire [7:0] reg_36,
		  input wire [7:0] reg_37,
		  input wire [7:0] reg_38,
		  input wire [7:0] reg_39,
		  input wire [7:0] reg_3a,
		  input wire [7:0] reg_3b,
		  input wire [7:0] reg_3c,
		  input wire [7:0] reg_3d,

		  input wire [7:0] reg_3e,
		  input wire [7:0] reg_3f
		  );

   wire 			   reset;
   sync_reset  i2c_slave_reset(
			       .clk(clk),
			       .glbl_reset(glbl_reset),
			       .reset(reset) );
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
   
   // just some tie-offs for future functionality not yet implemented...
   assign SDA_pu = 1'b0;
   assign SCL_pd = 1'b0;
   
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
   reg [7:0] 		       I2C_wdata;
   reg [7:0] 		       I2C_rdata;
   reg 			       I2C_reg_update;

   ///// register block definitions
   parameter RAM_WIDTH = 8;
   parameter RAM_ADDR_BITS = 5; // note parameter width exception in reg_a* assign block below
   
   reg [RAM_WIDTH-1:0] I2C_regblock [(2**RAM_ADDR_BITS)-1:0];
   reg [RAM_WIDTH-1:0] I2C_regread_async;

   wire [RAM_ADDR_BITS-1:0] I2C_ramaddr;

//   reg 		       wr_stb_d;

   ////////// code begins here
   always @ (posedge clk) begin
      if (reset || ((SCL_cstate == SCL_HIGH) && (SDA_cstate == SDA_RISE))) // stop condition always resets
	I2C_cstate <= I2C_START; 
      else
	I2C_cstate <= I2C_nstate;
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
			(I2C_daddr[7:1] == i2c_device_addr[7:1]) ?
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

   always @ (posedge clk) begin
      if( reset ) begin
	 I2C_bitcnt <= 4'b0;
	 I2C_daddr <= 8'b0;
	 I2C_wdata <= 8'b0;
	 SDA_pd <= 1'b0;
	 I2C_reg_update <= 1'b0;
	 I2C_rdata <= 8'b0;
	 I2C_addr <= 8'b0; // this persists across transactions
      end else begin
	 case (I2C_cstate) // synthesis parallel_case full_case
	   I2C_START: begin // everything in reset
	      I2C_bitcnt <= 4'b0;
	      I2C_daddr <= 8'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= 8'b0;
	      SDA_pd <= 1'b0;
	      I2C_reg_update <= 1'b0;
	      I2C_addr <= I2C_addr;
	   end

	   I2C_RESTART: begin
	      I2C_bitcnt <= 4'b0;
	      I2C_daddr <= 8'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= 8'b0;
	      SDA_pd <= 1'b0;
	      I2C_reg_update <= 1'b0;
	      I2C_addr <= I2C_addr;
	   end

	   // get my i2c device address (am I being talked to?)
	   I2C_DADDR: begin // shift in the address on rising edges of clock
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <= I2C_bitcnt + 4'b1;
		 I2C_daddr[7] <= I2C_daddr[6];
		 I2C_daddr[6] <= I2C_daddr[5];
		 I2C_daddr[5] <= I2C_daddr[4];
		 I2C_daddr[4] <= I2C_daddr[3];
		 I2C_daddr[3] <= I2C_daddr[2];
		 I2C_daddr[2] <= I2C_daddr[1];
		 I2C_daddr[1] <= I2C_daddr[0];
		 I2C_daddr[0] <= (SDA_cstate == SDA_HIGH) ? 1'b1 : 1'b0;
	      end else begin // we're oversampled so we need a hold-state gutter
		 I2C_bitcnt <= I2C_bitcnt;
		 I2C_daddr <= I2C_daddr;
	      end // else: !if( SCL_cstate == SCL_RISE )
	      SDA_pd <= 1'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= 8'b0;
	      I2C_reg_update <= 1'b0;
	      I2C_addr <= I2C_addr;
	   end // case: I2C_DADDR
	   I2C_ACK_DADDR: begin
	      SDA_pd <= 1'b1;  // active pull down ACK
	      I2C_daddr <= I2C_daddr;
	      I2C_bitcnt <= 4'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= I2C_regread_async;
	      I2C_reg_update <= 1'b0;
	      I2C_addr <= I2C_addr;
	   end

	   // get my i2c "write" address (what we want to access inside me)
	   I2C_ADDR: begin
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <= I2C_bitcnt + 4'b1;
		 I2C_addr[7] <= I2C_addr[6];
		 I2C_addr[6] <= I2C_addr[5];
		 I2C_addr[5] <= I2C_addr[4];
		 I2C_addr[4] <= I2C_addr[3];
		 I2C_addr[3] <= I2C_addr[2];
		 I2C_addr[2] <= I2C_addr[1];
		 I2C_addr[1] <= I2C_addr[0];
		 I2C_addr[0] <= (SDA_cstate == SDA_HIGH) ? 1'b1 : 1'b0;
	      end else begin // we're oversampled so we need a hold-state gutter
		 I2C_bitcnt <= I2C_bitcnt;
		 I2C_addr <= I2C_addr;
	      end // else: !if( SCL_cstate == SCL_RISE )
	      SDA_pd <= 1'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= 8'b0;
	      I2C_reg_update <= 1'b0;
	      I2C_daddr <= I2C_daddr;
	   end // case: I2C_ADDR
	   I2C_ACK_ADDR: begin
	      SDA_pd <= 1'b1;  // active pull down ACK
	      I2C_daddr <= I2C_daddr;
	      I2C_bitcnt <= 4'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= I2C_regread_async; // update my read data here
	      I2C_reg_update <= 1'b0;
	      I2C_addr <= I2C_addr;
	   end
	   
	   
	   // write branch
	   I2C_WR_DATA: begin // shift in data on rising edges of clock
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <= I2C_bitcnt + 4'b1;
		 I2C_wdata[7] <= I2C_wdata[6];
		 I2C_wdata[6] <= I2C_wdata[5];
		 I2C_wdata[5] <= I2C_wdata[4];
		 I2C_wdata[4] <= I2C_wdata[3];
		 I2C_wdata[3] <= I2C_wdata[2];
		 I2C_wdata[2] <= I2C_wdata[1];
		 I2C_wdata[1] <= I2C_wdata[0];
		 I2C_wdata[0] <= (SDA_cstate == SDA_HIGH) ? 1'b1 : 1'b0;
	      end else begin
		 I2C_bitcnt <= I2C_bitcnt; // hold state gutter
		 I2C_wdata <= I2C_wdata;
	      end // else: !if( SCL_cstate == SCL_RISE )
	      SDA_pd <= 1'b0;
	      I2C_daddr <= I2C_daddr;
	      I2C_reg_update <= 1'b0;
	      I2C_rdata <= I2C_rdata;
	      I2C_addr <= I2C_addr;
	   end // case: I2C_WR_DATA
	   I2C_ACK_WR: begin
	      SDA_pd <= 1'b1;  // active pull down ACK
	      I2C_daddr <= I2C_daddr;
	      I2C_bitcnt <= 4'b0;
	      I2C_wdata <= I2C_wdata;
	      I2C_reg_update <= 1'b1;  // write the data now (over and over again while in state)
	      I2C_rdata <= I2C_rdata;
	      I2C_addr <= I2C_addr;
	   end
	   I2C_END_WR: begin
	      SDA_pd <= 1'b0; // let SDA rise (host may look for this to know ack is done
	      I2C_addr <= I2C_addr + 8'b1; // this is a one-cycle state so this is safe
	      I2C_bitcnt <= 4'b0;
	      I2C_wdata <= 8'b0;
	      I2C_rdata <= I2C_rdata;
	      I2C_reg_update <= 1'b0;
	      I2C_daddr <= I2C_daddr;
	   end

	   // read branch
	   I2C_RD_DATA: begin // shift out data on falling edges of clock
	      SDA_pd <= I2C_rdata[7] ? 1'b0 : 1'b1;
	      if( SCL_cstate == SCL_RISE ) begin
		 I2C_bitcnt <= I2C_bitcnt + 4'b1;
	      end else begin
		 I2C_bitcnt <= I2C_bitcnt; // hold state gutter
	      end
	      
	      if( SCL_cstate == SCL_FALL ) begin
		 I2C_rdata[7] <= I2C_rdata[6];
		 I2C_rdata[6] <= I2C_rdata[5];
		 I2C_rdata[5] <= I2C_rdata[4];
		 I2C_rdata[4] <= I2C_rdata[3];
		 I2C_rdata[3] <= I2C_rdata[2];
		 I2C_rdata[2] <= I2C_rdata[1];
		 I2C_rdata[1] <= I2C_rdata[0];
		 I2C_rdata[0] <= 1'b0;
	      end else begin
		 I2C_rdata <= I2C_rdata;
	      end // else: !if( SCL_cstate == SCL_RISE )
	      I2C_daddr <= I2C_daddr;
	      I2C_reg_update <= 1'b0;
	      I2C_wdata <= I2C_wdata;
	      I2C_addr <= I2C_addr;
	   end // case: I2C_RD_DATA
	   I2C_ACK_RD: begin
	      SDA_pd <= 1'b0;  // in ack state don't pull down, we are listening to host
	      I2C_daddr <= I2C_daddr;
	      I2C_bitcnt <= 4'b0;
	      I2C_rdata <= I2C_rdata;
	      I2C_reg_update <= 1'b0;
	      I2C_wdata <= I2C_wdata;
	      I2C_addr <= I2C_addr;
	   end
	   I2C_END_RD: begin
	      SDA_pd <= 1'b0; // let SDA rise (host may look for this to know ack is done
	      I2C_addr <= I2C_addr + 8'b1; // this is a one-cycle state so this is safe
	      I2C_bitcnt <= 4'b0;
	      I2C_rdata <= I2C_rdata;
	      I2C_reg_update <= 1'b0;
	      I2C_wdata <= I2C_wdata;
	      I2C_daddr <= I2C_daddr;
	   end
	   I2C_END_RD2: begin
	      SDA_pd <= 1'b0;
	      I2C_daddr <= 8'b0;
	      I2C_bitcnt <= 4'b0;
	      I2C_rdata <= I2C_regread_async; // update my read data here
	      I2C_reg_update <= 1'b0;
	      I2C_wdata <= I2C_wdata;
	      I2C_addr <= I2C_addr;
	   end

	   I2C_WAITSTOP: begin
	      SDA_pd <= 1'b0;
	      I2C_daddr <= 8'b0;
	      I2C_bitcnt <= 4'b0;
	      I2C_rdata <= I2C_rdata;
	      I2C_reg_update <= 1'b0;
	      I2C_wdata <= I2C_wdata;
	      I2C_addr <= I2C_addr;
	   end
	 endcase // case (cstate)
      end // else: !if( reset )
   end // always @ (posedge clk or posedge reset)


   ////////////////
   ///// register bank management
   ////////////////
//   always @(posedge clk) begin 
//      wr_stb_d <= wr_stb;
//      end else if (wr_stb & !wr_stb_d) begin // only act on the rising pulse of wr_stb
//	 I2C_regblock[reg_addr] <= reg_data_in; // vestigal remnant?? changes programming model
//	 // slightly, need to look into this....
//      end else if (I2C_reg_update) begin // this should be multiple cycles
   always @(posedge clk) begin 
      if (I2C_reg_update) begin // this should be multiple cycles
         I2C_regblock[I2C_ramaddr] <= I2C_wdata;
      end
   end

   always @(*) begin
      case (I2C_addr)
	6'h2: begin
	   I2C_regread_async = reg_2;
	end
	6'h8: begin
	   I2C_regread_async = reg_8;
	end
	6'h9: begin
	   I2C_regread_async = reg_9;
	end
	6'ha: begin
	   I2C_regread_async = reg_a;
	end
	6'hb: begin
	   I2C_regread_async = reg_b;
	end

	6'h10: begin
	   I2C_regread_async = reg_10;
	end
	6'h18: begin
	   I2C_regread_async = reg_18;
	end
	
	6'h20: begin
	   I2C_regread_async = reg_20;
	end
	6'h21: begin
	   I2C_regread_async = reg_21;
	end
	6'h22: begin
	   I2C_regread_async = reg_22;
	end
	6'h23: begin
	   I2C_regread_async = reg_23;
	end
	6'h24: begin
	   I2C_regread_async = reg_24;
	end
	6'h25: begin
	   I2C_regread_async = reg_25;
	end
	6'h26: begin
	   I2C_regread_async = reg_26;
	end
	6'h27: begin
	   I2C_regread_async = reg_27;
	end
	6'h28: begin
	   I2C_regread_async = reg_28;
	end
	6'h29: begin
	   I2C_regread_async = reg_29;
	end
	6'h2a: begin
	   I2C_regread_async = reg_2a;
	end
	6'h2b: begin
	   I2C_regread_async = reg_2b;
	end
	6'h2c: begin
	   I2C_regread_async = reg_2c;
	end
	6'h2d: begin
	   I2C_regread_async = reg_2d;
	end
	6'h2e: begin
	   I2C_regread_async = reg_2e;
	end
	6'h2f: begin
	   I2C_regread_async = reg_2f;
	end
  
	6'h30: begin
	   I2C_regread_async = reg_30;
	end
	6'h31: begin
	   I2C_regread_async = reg_31;
	end
	6'h32: begin
	   I2C_regread_async = reg_32;
	end
	6'h33: begin
	   I2C_regread_async = reg_33;
	end
	6'h34: begin
	   I2C_regread_async = reg_34;
	end
	6'h35: begin
	   I2C_regread_async = reg_35;
	end
	6'h36: begin
	   I2C_regread_async = reg_36;
	end
	6'h37: begin
	   I2C_regread_async = reg_37;
	end
	
	6'h38: begin
	   I2C_regread_async = reg_38;
	end
	6'h39: begin
	   I2C_regread_async = reg_39;
	end
	6'h3a: begin
	   I2C_regread_async = reg_3a;
	end
	6'h3b: begin
	   I2C_regread_async = reg_3b;
	end
	6'h3c: begin
	   I2C_regread_async = reg_3c;
	end
	6'h3d: begin
	   I2C_regread_async = reg_3d;
	end

	6'h3e: begin
	   I2C_regread_async = reg_3e;
	end
	6'h3f: begin
	   I2C_regread_async = reg_3f;
	end
	
	default: begin
	   I2C_regread_async = I2C_regblock[I2C_ramaddr];
	end
      endcase // case I2C_ramaddr
   end // always @ (*)
   
   assign I2C_ramaddr = I2C_addr[RAM_ADDR_BITS-1:0];

   ///////// ick, had to hard-code the width against RAM_ADDR_BITS which is parameterized
   assign reg_0 = I2C_regblock[5'h0];
   assign reg_1 = I2C_regblock[5'h1];
//   assign reg_2 = I2C_regblock[5'h2];
   assign reg_3 = I2C_regblock[5'h3];
   assign reg_4 = I2C_regblock[5'h4];
   assign reg_5 = I2C_regblock[5'h5];
   assign reg_6 = I2C_regblock[5'h6];
   assign reg_7 = I2C_regblock[5'h7];
//   assign reg_8 = I2C_regblock[5'h8];
//   assign reg_9 = I2C_regblock[5'h9];
//   assign reg_a = I2C_regblock[5'ha];
//   assign reg_b = I2C_regblock[5'hb];

   assign reg_c = I2C_regblock[5'hc];
   assign reg_d = I2C_regblock[5'hd];
   assign reg_e = I2C_regblock[5'he];
   assign reg_f = I2C_regblock[5'hf];

   assign reg_11 = I2C_regblock[5'h11];
   assign reg_12 = I2C_regblock[5'h12];

   assign reg_13 = I2C_regblock[5'h13];
   assign reg_14 = I2C_regblock[5'h14];

   assign reg_15 = I2C_regblock[5'h15];
   assign reg_16 = I2C_regblock[5'h16];
   assign reg_17 = I2C_regblock[5'h17];
   
//   assign reg_18 = I2C_regblock[5'h18];

   assign reg_19 = I2C_regblock[5'h19]; // lsb of Km
   assign reg_1a = I2C_regblock[5'h1a];
   assign reg_1b = I2C_regblock[5'h1b];
   assign reg_1c = I2C_regblock[5'h1c];
   assign reg_1d = I2C_regblock[5'h1d];
   assign reg_1e = I2C_regblock[5'h1e];
   assign reg_1f = I2C_regblock[5'h1f]; // msb of Km
   
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

   always @ (posedge clk) begin
      if (reset)
	SCL_cstate <= SCL_HIGH; // always start here even if it's wrong -- easier to test
      else
	SCL_cstate <= SCL_nstate;
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

   always @ (posedge clk) begin
      if( reset ) begin
	 SCL_rfcnt <= 5'b0;
      end else begin
	 case (SCL_cstate) // synthesis parallel_case full_case
	   SCL_HIGH: begin
	      if( SCL_sync == 1'b1 ) begin
		 SCL_rfcnt <= 5'b0;
	      end else begin
		 SCL_rfcnt <= SCL_rfcnt + 5'b1;
	      end
	   end
	   SCL_FALL: begin
	      SCL_rfcnt <= 5'b0;
	   end
	   SCL_LOW: begin
	      if( SCL_sync == 1'b0 ) begin
		 SCL_rfcnt <= 5'b0;
	      end else begin
		 SCL_rfcnt <= SCL_rfcnt + 5'b1;
	      end
	   end
	   SCL_RISE: begin
	      SCL_rfcnt <= 5'b0;
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

   always @ (posedge clk) begin
      if (reset)
	SDA_cstate <= SDA_HIGH; // always start here even if it's wrong -- easier to test
      else
	SDA_cstate <= SDA_nstate;
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

   always @ (posedge clk) begin
      if( reset ) begin
	 SDA_rfcnt <= 5'b0;
      end else begin
	 case (SDA_cstate) // synthesis parallel_case full_case
	   SDA_HIGH: begin
	      if( SDA_sync == 1'b1 ) begin
		 SDA_rfcnt <= 5'b0;
	      end else begin
		 SDA_rfcnt <= SDA_rfcnt + 5'b1;
	      end
	   end
	   SDA_FALL: begin
	      SDA_rfcnt <= 5'b0;
	   end
	   SDA_LOW: begin
	      if( SDA_sync == 1'b0 ) begin
		 SDA_rfcnt <= 5'b0;
	      end else begin
		 SDA_rfcnt <= SDA_rfcnt + 5'b1;
	      end
	   end
	   SDA_RISE: begin
	      SDA_rfcnt <= 5'b0;
	   end
	 endcase // case (cstate)
      end // else: !if( reset )
   end // always @ (posedge clk or posedge reset)

   
   
   /////////////////////
   /////// synchronizers
   /////////////////////
   always @ (posedge clk) begin
      SCL_s <= SCL;
      SCL_sync <= SCL_s;
      SDA_s <= SDA;
      SDA_sync <= SDA_s;
   end // always @ (posedge clk or posedge reset)
   
endmodule // i2c_slave

