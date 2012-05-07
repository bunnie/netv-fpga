////////////////////////////////////////////////
// Copyright (c) 2012, Andrew "bunnie" Huang  
// (bunnie _aht_ bunniestudios "dote" com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//     Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in
//     the documentation and/or other materials provided with the
//     distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////

/// according to Xilinx WP272, all flip flops are reset to a "known value"
/// by GSR. You're supposed to trust that. Of course, this "known value"
/// isn't very explicitly stated, searching through the xilinx manuals
/// it seems everything defaults to 0 except for stuff that's presetable.

/// anyways, this module generates a local, synchronized reset based upon
/// a global reset. The idea is to instantiate one of these near every
/// terminal reset sink, so as to avoid loading down a global reset network.

/// this should optimize utilization and speed a bit, and also allow the
/// synthesizer to get more aggressive about using larger primitives

//////////
// the input is the asychronous reset of interest
// and the clock to synchronize it to
// the output is a synchronized reset that is at least four clock cycles wide
module sync_reset (
		   input wire glbl_reset, // async reset
		   input wire clk,
		   output wire reset 
	      );

   wire [3:0]       reschain;
   
   FDPE fdres0( .Q(reschain[0]), .C(clk), .CE(1'b1), .D(1'b0), .PRE(glbl_reset) );
   FDPE fdres1( .Q(reschain[1]), .C(clk), .CE(1'b1), .D(reschain[0]), .PRE(glbl_reset) );
   FDPE fdres2( .Q(reschain[2]), .C(clk), .CE(1'b1), .D(reschain[1]), .PRE(glbl_reset) );
   FDPE fdres3( .Q(reschain[3]), .C(clk), .CE(1'b1), .D(reschain[2]), .PRE(glbl_reset) );

   assign reset = reschain[3];

endmodule // sync_reset
