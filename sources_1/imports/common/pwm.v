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
// this module does simple PWM modulation to create the "breathing" LED effect
//

`timescale 1 ns / 1 ps

module pwm(
	   input wire clk812k, // use clock from device DNA block, 812.5kHz
	   input wire reset,
	   output reg pwmout,
	   input wire [11:0] bright,
	   input wire [11:0] dim
	   );

   reg [9:0] 		    pwm_count;
   reg 			    pwmreg;
   reg [11:0] 		    interpolate;
   reg 			    countdn;
   wire [9:0] 		    interp;
      
   always @(posedge clk812k) begin
      if( reset ) begin
	 pwm_count <= 0;
	 interpolate[11:0] <= dim[11:0];
	 countdn <= 0;
      end else begin
	 if( interpolate[11:0] >= bright[11:0] ) begin
	    countdn <= 1;
	 end else if( interpolate[11:0] <= dim[11:0] ) begin
	    countdn <= 0;
	 end else begin
	    countdn <= countdn;
	 end
	 
	 if( pwm_count[9:0] == 10'h0 ) begin
	    if( countdn == 1'b1 ) begin
	       interpolate[11:0] <= interpolate[11:0] - 12'b1;
	    end else begin
	       interpolate[11:0] <= interpolate[11:0] + 12'b1;
	    end
	 end else begin
	    interpolate[11:0] <= interpolate[11:0];
	 end
	 
	 pwm_count[9:0] <= pwm_count[9:0] + 10'b1;
      end

      pwmreg <= (pwm_count[9:0] < interp[9:0]);

   end // always @ (posedge clk812k or posedge reset)

   assign interp[9:0] = interpolate[11:2];

   always @(posedge clk812k) begin
      // make it registered to ease up routing congestion to the edge
      pwmout <= !pwmreg;
   end

endmodule // pwm
