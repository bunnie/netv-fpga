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

//
//  data packet consists of 7 x 4 = 28 bytes + 4 bytes of parity
//  data header consists of 3 bytes + 1 byte of parity

`timescale 1 ns / 1 ps

module data_island (
		    input wire pclk;
		    input wire reset;
		    
		    input wire [3:0] data_ch2;
		    input wire [3:0] data_ch1;
		    input wire data_header_ch0;
		    input wire data_gb;
		    input wire video_gb;
		    
		    output wire [7:0] packet_type;
		    output wire [15:0] header_data;
		    output wire [7:0] packet_data; // this is an 8 entry, 8-bit wide RAM
		    input wire [2:0] packet_index;
		    );

   parameter RAM_WIDTH = <ram_width>;
   parameter RAM_ADDR_BITS = <ram_addr_bits>;

   (* RAM_STYLE="{AUTO | BLOCK |  BLOCK_POWER1 | BLOCK_POWER2}" *)
   reg [RAM_WIDTH-1:0] <ram_name> [(2**RAM_ADDR_BITS)-1:0];
   reg [RAM_WIDTH-1:0] <output_dataA>, <output_dataB>;

   <reg_or_wire> [RAM_ADDR_BITS-1:0] <addressA>, <addressB>;
   <reg_or_wire> [RAM_WIDTH-1:0] <input_dataA>;

   //  The following code is only necessary if you wish to initialize the RAM 
   //  contents via an external file (use $readmemb for binary data)
   initial
      $readmemh("<data_file_name>", <rom_name>, <begin_address>, <end_address>);

   always @(posedge <clock>) begin
      if (<enableA>) begin
         if (<write_enableA>)
            <ram_name>[<addressA>] <= <input_dataA>;
         <output_dataA> <= <ram_name>[<addressA>];
      end
      if (<enableB>)
         <output_dataB> <= <ram_name>[<addressB>];
   end      

endmodule // data_island
