`include "primitives.v";

module spi_ad7324(DOUT, DIN, CS, CLK_IN, R, CLK_OUT, DATA_READ, HOLD, READ_ALL);


   //Inputs
   input DOUT;
   input CLK_IN;
   input R;
   input HOLD;
	input READ_ALL;

   //Outputs
   output DIN;
   output CS;
   output CLK_OUT;
   output[15:0] DATA_READ;

   //Parameters
   parameter STATE_START         	= 4'd0;
   parameter STATE_CONFIG_1      	= 4'd1;
   parameter STATE_CONFIG_PAUSE_1 	= 4'd2;
   parameter STATE_CONFIG_2      	= 4'd3;   
   parameter STATE_CONFIG_PAUSE_2  	= 4'd4;
   parameter STATE_CONFIG_3      	= 4'd5;
   parameter STATE_SETUP        	= 4'd6;
   parameter STATE_READ        		= 4'd7;
   parameter STATE_LOAD        	 	= 4'd8;
   parameter STATE_LOAD2         	= 4'd9;
  
	
   reg[48:0] CONFIG_PARAM;
	
   //Internal nets; not set by SM
   wire[15:0]   sr_mux;
   wire[15:0]   sr_out;
   wire[15:0]   co_mux;
   wire[15:0]   co_out;
   wire[15:0]   ou_mux;
   wire[3:0]    sm_state;
   

   //Internal nets; set by SM
   reg      sr_clear;
   reg      sr_shift;
   reg      co_clear;
   reg      co_count;
   reg[3:0] sm_next;
   reg      sp_config;
   reg      ou_sample;
   reg      CS;

   //Shift Register for input
   DFFA	   #(
                 .WORD(16)
           )
           U0(   .CLK(CLK_IN), 
                 .R(R),
                 .D(sr_mux),
                 .Q(sr_out)
             );

   MUX3A   #(
                  .WORD(16)
            )
            U1(   .A(16'd0),
                  .B({sr_out[14:0],DOUT}),
                  .C(sr_out),
                  .S({   sr_clear,
                         (~sr_clear & sr_shift),
                         (~sr_clear & ~sr_shift)
                     }),
                  .O(sr_mux)
            );

   //Counter used for timing stuff, simplifies SM
   DFFA	   #(
                 .WORD(16)
           )
           U2(   .CLK(CLK_IN), 
                 .R(R),
                 .D(co_mux),
                 .Q(co_out)
             );

   MUX3A   #(
                  .WORD(16)
            )
            U3(   .A(16'd0),
                  .B(co_out + 16'd1),
                  .C(co_out),
                  .S({   co_clear,
                         (~co_clear & co_count),
                         (~co_clear & ~co_count)
                    }),
                  .O(co_mux)
            );

   //DFF for sampling output
   DFFA	   #(
                 .WORD(16)
           )
           U4(   .CLK(CLK_IN), 
                 .R(R),
                 .D(ou_mux),
                 .Q(DATA_READ)
             );

   MUX2A   #(
                  .WORD(16)
            )
            U5(   .A(sr_out),
                  .B(DATA_READ),
                  .S({   ou_sample,
                         ~ou_sample
                    }),
                  .O(ou_mux)
            );

   //State Machine
   DFFA	   #(
                 .WORD(4)
           )
           U6(   .CLK(CLK_IN), 
                 .R(R),
                 .D(sm_next),
                 .Q(sm_state)
             );

  always@(*)
     begin
        if(READ_ALL == 1)
		CONFIG_PARAM <= 48'h2C01007F07FD;
        else
		CONFIG_PARAM <= 48'h0C01000707FD;
     end 
//				 
   always@(*)
   begin
   
      case(sm_state)
         STATE_START:
         begin
            sm_next <= STATE_CONFIG_1;
         end

         STATE_CONFIG_1:
         begin
            if(co_out > 16'd14)
               sm_next <= STATE_CONFIG_PAUSE_1;
            else
               sm_next <= STATE_CONFIG_1;
         end

         STATE_CONFIG_PAUSE_1:
         begin
               sm_next <= STATE_CONFIG_2;
         end
		 
		 STATE_CONFIG_2:
         begin
            if(co_out > 16'd30)
               sm_next <= STATE_CONFIG_PAUSE_2;
            else
               sm_next <= STATE_CONFIG_2;
         end

         STATE_CONFIG_PAUSE_2:
         begin
               sm_next <= STATE_CONFIG_3;
         end
		 		 
         STATE_CONFIG_3:
         begin
            if(co_out > 16'd46)
               sm_next <= STATE_SETUP;
            else
               sm_next <= STATE_CONFIG_3;
         end

         STATE_SETUP:
         begin
            sm_next <= STATE_READ;
         end

         STATE_READ:
         begin
            if(co_out > 16'd14)
               sm_next <= STATE_LOAD;
            else
               sm_next <= STATE_READ;
         end

         STATE_LOAD:
         begin
            if(!HOLD)
               sm_next <= STATE_LOAD;
            else
               sm_next <= STATE_LOAD2;
         end

         STATE_LOAD2:
         begin
            if(HOLD)
               sm_next <= STATE_LOAD2;
            else
               sm_next <= STATE_READ;
         end

         default:
         begin
            sm_next <= STATE_START;
         end

      endcase

   end


   always@(*)
   begin

      case(sm_state)

         STATE_START:
         begin
            CS = 1'b1;
            ou_sample= 1'b0;
            sp_config= 1'b0;
            sr_clear = 1'b1;
            sr_shift = 1'b0;
            co_clear = 1'b1;
            co_count = 1'b0;
         end

         STATE_CONFIG_1:
         begin
            CS = 1'b0;
            ou_sample= 1'b0;
            sp_config= 1'b1;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b0;
            co_count = 1'b1;
         end

	 STATE_CONFIG_PAUSE_1:
         begin
            CS = 1'b1;
            ou_sample= 1'b0;
            sp_config= 1'b1;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b0;
            co_count = 1'b0;
         end
		 
	STATE_CONFIG_2:
         begin
            CS = 1'b0;
            ou_sample= 1'b0;
            sp_config= 1'b1;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b0;
            co_count = 1'b1;
         end

	 STATE_CONFIG_PAUSE_2:
         begin
            CS = 1'b1;
            ou_sample= 1'b0;
            sp_config= 1'b1;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b0;
            co_count = 1'b0;
         end
		 
	 STATE_CONFIG_3:
         begin
            CS = 1'b0;
            ou_sample= 1'b0;
            sp_config= 1'b1;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b0;
            co_count = 1'b1;
         end

	 STATE_SETUP:
         begin
            CS = 1'b1;
            ou_sample= 1'b0;
            sp_config= 1'b0;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b1;
            co_count = 1'b0;
         end

         STATE_READ:
         begin
            CS = 1'b0;
            ou_sample= 1'b0;
            sp_config= 1'b0;
            sr_clear = 1'b0;
            sr_shift = 1'b1;
            co_clear = 1'b0;
            co_count = 1'b1;
         end

         STATE_LOAD:
         begin
            CS = 1'b1;
            ou_sample= 1'b1;
            sp_config= 1'b0;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b1;
            co_count = 1'b0;
         end

         STATE_LOAD2:
         begin
            CS = 1'b1;
            ou_sample= 1'b1;
            sp_config= 1'b0;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b1;
            co_count = 1'b0;
         end

         default:
         begin
            CS = 1'b1;
            ou_sample= 1'b0;
            sp_config= 1'b0;
            sr_clear = 1'b0;
            sr_shift = 1'b0;
            co_clear = 1'b0;
            co_count = 1'b0;
         end

      endcase

   end

   //Static logic

   assign DIN = sp_config ? CONFIG_PARAM[co_out] : 1'b0;
   assign CLK_OUT = CLK_IN;

endmodule
