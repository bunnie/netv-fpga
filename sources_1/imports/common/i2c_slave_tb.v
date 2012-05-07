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
`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:30:45 04/05/2011
// Design Name:   i2c_slave
// Module Name:   C:/largework/fpga/hdmi/impl4/common/i2c_slave_tb.v
// Project Name:  impl4
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: i2c_slave
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module i2c_slave_tb;
   
   reg 	      SDA; // physical wire state
   reg 	      SCL;

   // Inputs
   reg clk;
   reg reset;
   reg [7:0] i2c_device_addr;
   reg [7:0] reg_addr;
   reg 	     wr_stb;
   reg [7:0] reg_data_in;
   
   // Outputs
   wire      SCL_pd;
   wire      SDA_pd;
   wire      SDA_pu;
   wire [7:0] reg_a0;
   wire [7:0] reg_a1;
   wire [7:0] reg_a2;
   wire [7:0] reg_a3;
   
   // Instantiate the Unit Under Test (UUT)
   i2c_slave uut (
		  .SCL(SCL), 
		  .SCL_pd(SCL_pd), 
		  .SDA(SDA), 
		  .SDA_pd(SDA_pd), 
		  .SDA_pu(SDA_pu), 
		  .clk(clk), 
		  .reset(reset), 
		  .i2c_device_addr(i2c_device_addr), 
		  .reg_addr(reg_addr), 
		  .wr_stb(wr_stb), 
		  .reg_data_in(reg_data_in), 
		  .reg_0(reg_a0), 
		  .reg_1(reg_a1), 
		  .reg_2(reg_a2), 
		  .reg_3(reg_a3)
		  );

   reg 	      sda_host; // what the host is driving to
   reg 	      scl_host;
   reg        ack;  // what the ack state is
   reg [7:0]  readdata;

//   always @(SCL_pd, SDA_pd, SDA_pu, sda_host, scl_host) begin
   always @(*) begin // lazy
      // scl equations
      case( {SCL_pd, scl_host} )
	2'b00:  SCL <= 1'b0;
	2'b01:  SCL <= 1'b1;
	2'b10:  SCL <= 1'b0;
	// conflict case
	2'b11:  SCL <= 1'bX;
	// handle tristate case
	2'b0Z:  SCL <= 1'b1;
	2'b1Z:  SCL <= 1'b0;
	default: SCL <= 1'bX;
      endcase // case ( {SCL_pd, scl_host} )

      case( {SDA_pd, SDA_pu, sda_host} )
	3'b000: SDA <= 1'b0;
	3'b001: SDA <= 1'b1;
	3'b010: SDA <= 1'bX; // change to 1'b1 for override
	3'b011: SDA <= 1'b1;
	3'b100: SDA <= 1'b0;
	3'b101: SDA <= 1'bX; // change to 1'b0 for override
	3'b110: SDA <= 1'bX;
	3'b111: SDA <= 1'bX;

	// tristate case
	3'b00Z: SDA <= 1'b1;
	3'b01Z: SDA <= 1'b1;
	3'b10Z: SDA <= 1'b0;
	3'b11Z: SDA <= 1'bX;
      endcase // case ( {SDA_pd, SDA_pu, sda_host} )
   end
   
   parameter PERIOD = 16'd40;   // 25 MHz
   parameter I2C_PD = 16'd2464; // make it odd to try and catch non-phase synced issues
   parameter I2C_TH = 16'd114;
   parameter I2C_TS = 16'd217;
   
   always begin
      clk = 1'b0;
      #(PERIOD/2) clk = 1'b1;
      #(PERIOD/2);
   end

   task I2C_idle;
      begin
	 scl_host = 1'bZ;
	 sda_host = 1'bZ;
	 #I2C_PD;
      end
   endtask // I2C_idle

   task I2C_start;
      begin
	 scl_host = 1'bZ;
	 sda_host = 1'bZ;
	 #(I2C_PD/2);
	 scl_host = 1'bZ;
	 sda_host = 1'b0;
	 #(I2C_PD/2);
      end
   endtask // I2C_start

   task I2C_stop;
      begin
	 scl_host = 1'bZ;
	 sda_host = 1'b0;
	 #(I2C_PD/2);
	 scl_host = 1'bZ;
	 sda_host = 1'bZ;
	 #(I2C_PD/2);
      end
   endtask // I2C_start

   task I2C_tx_bit; // tx from host ( from testbench )
      input  bitval;
      
      begin
	 scl_host = 1'b0;
	 #I2C_TH;
	 sda_host = bitval;
	 #(I2C_PD/2);
	 scl_host = 1'bZ;
	 sda_host = bitval;
	 #(I2C_PD/2);
      end
   endtask // I2C_tx_bit

   task I2C_rx_bit; // rx to host ( to testbench )
      output bitval;
      
      begin
	 scl_host = 1'b0;
	 #(I2C_TH/2);
	 sda_host = 1'bz;
	 #(I2C_PD/2);
	 scl_host = 1'bZ;
	 sda_host = 1'bz;
	 #1;
	 bitval = SDA;
	 #(I2C_PD/2);
      end
   endtask // I2C_start

   task I2C_ack_low;
      begin
	 scl_host = 1'b0;
	 #(I2C_PD/2);
      end
   endtask // I2C_ack_low
   

   task I2C_tx_daddr;
      input [7:0] daddr;
      output 	  rack;

      begin
	 I2C_tx_bit( daddr[7] );
	 I2C_tx_bit( daddr[6] );
	 I2C_tx_bit( daddr[5] );
	 I2C_tx_bit( daddr[4] );
	 I2C_tx_bit( daddr[3] );
	 I2C_tx_bit( daddr[2] );
	 I2C_tx_bit( daddr[1] );
	 I2C_tx_bit( daddr[0] );
	 I2C_rx_bit(rack);
	 I2C_ack_low();
      end
   endtask // I2C_TX_DADDR
   
   task I2C_rx_daddr;
      output [7:0] daddr;
      input 	   nack;

      begin
	 I2C_rx_bit(daddr[7]);
	 I2C_rx_bit(daddr[6]);
	 I2C_rx_bit(daddr[5]);
	 I2C_rx_bit(daddr[4]);
	 I2C_rx_bit(daddr[3]);
	 I2C_rx_bit(daddr[2]);
	 I2C_rx_bit(daddr[1]);
	 I2C_rx_bit(daddr[0]);
	 I2C_tx_bit( nack );
	 I2C_ack_low();
      end
   endtask // I2C_RX_DADDR
   
   initial begin
      // Initialize Inputs
      clk = 0;
      reset = 0;
      i2c_device_addr = 8'h72;
      reg_addr = 0;
      wr_stb = 0;
      reg_data_in = 0;

      $stop;
      
      I2C_idle();
      // run an actual reset cycle
      #(PERIOD*4);
      reset = 1;
      #(PERIOD*4);
      reset = 0;
      #(PERIOD*4);

      // now pre-set a few I2C registers
      reg_addr = 0;
      wr_stb = 1;
      reg_data_in = 8'hDE;
      #(PERIOD*4);
      
      wr_stb = 0;
      #(PERIOD*1);
      
      reg_addr = 1;
      wr_stb = 1;
      reg_data_in = 8'hAD;
      #(PERIOD*2);

      wr_stb = 0;
      #(PERIOD*1);
      
      reg_addr = 2;
      wr_stb = 1;
      reg_data_in = 8'hBE;
      #(PERIOD*2);

      wr_stb = 0;
      #(PERIOD*1);
      
      reg_addr = 3;
      wr_stb = 1;
      reg_data_in = 8'hEF;
      #(PERIOD*2);


      // let it soak for a bit for good measure
      #(PERIOD*10);

      // now the real sim starts
      I2C_idle();

      // write some data
      I2C_start();
      I2C_tx_daddr(8'h72, ack); // write to device 72
      I2C_tx_daddr(8'h01, ack); // address 01
      I2C_tx_daddr(8'h33, ack); // data 55
      I2C_stop();
      #(I2C_PD*5);

      // do a multi-cycle read
      I2C_start();
      I2C_tx_daddr(8'h72, ack); // dummy write to 72
      I2C_tx_daddr(8'h00, ack); // address 00
      I2C_start();
      I2C_tx_daddr(8'h73, ack); // read from 72
//      #(I2C_PD*3);
      I2C_rx_daddr(readdata, 1'b0); // data @ address 0
//      #(I2C_PD*3);
      I2C_rx_daddr(readdata, 1'b0); // data @ address 0
      I2C_rx_daddr(readdata, 1'b0); // data @ address 0
      I2C_rx_daddr(readdata, 1'bz); // data @ address 0
      I2C_stop();
      #(I2C_PD*5);

      // do a multi-cycle write
      I2C_start();
      I2C_tx_daddr(8'h72, ack); // write to device 70
      I2C_tx_daddr(8'h01, ack); // address 01
      I2C_tx_daddr(8'hFA, ack); 
      I2C_tx_daddr(8'hCE, ack); 
      I2C_tx_daddr(8'h69, ack); 
      I2C_stop();
      #(I2C_PD*5);

      // read back one address at a time
      I2C_start();
      I2C_tx_daddr(8'h72, ack); // dummy write to 72
      I2C_tx_daddr(8'h00, ack); // address 00
      
      #(I2C_PD*5);
      I2C_start();
      I2C_tx_daddr(8'h73, ack); // read from 72
      I2C_rx_daddr(readdata, 1'bz); // one read
      I2C_stop();

      // this is the only questionable vector
      // if you do an isolated read, should the address have
      // incremeted from the previous read, or
      // should it be the same. I have implemented it so
      // that it increments.
      I2C_start();
      I2C_tx_daddr(8'h73, ack); // read from 72 
      I2C_rx_daddr(readdata, 1'bz); // one read
      I2C_stop();

      #(I2C_PD*5);

      I2C_start();
      I2C_tx_daddr(8'h73, ack); // read from 72
      I2C_rx_daddr(readdata, 1'b0); // one read
      I2C_rx_daddr(readdata, 1'bz); // one read
      I2C_stop();
      
      
      // write to another device not us
      I2C_start();
      I2C_tx_daddr(8'hA0, ack); // write to device a0
      I2C_tx_daddr(8'h01, ack); // address 01
      I2C_tx_daddr(8'h55, ack); // data 55  -- this should be ignored
      I2C_stop();

      #I2C_PD;
      #I2C_PD;
   end
   
endmodule

