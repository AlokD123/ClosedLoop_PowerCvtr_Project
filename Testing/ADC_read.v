//To read from ADC

//`include "spi_ad7324_v2.v"

module main(clk, GPIO_0)
	//External inputs
	input clk;
	//input MEAS_SWITCH_PULSE; //Signal to cycle between channels
	
	//NOTE: IF ISSUE WITH GPIO_0[1], rename in Pin mapping as DOUT!!!!!!!!!
	
	//Name GPIO pins
	'define DOUT		[0]GPIO_0
	'define DIN 		[16]GPIO_0
	'define CS 			[17]GPIO_0
	'define SCLK		[18]GPIO_0
	//Number states
	parameter STATE_RESET         	= 4'd0;
	parameter STATE_START      		= 4'd1;
	parameter STATE_GETDATA 		= 4'd2;
	
	//Inputs from GPIO
	input DOUT; 
	input DATA_READ;
	//Outputs to GPIO
	output DIN;
	output CS;
	output SCLK;
	
	
	//Outputs to spi_adc block
	//CLK20M (from PLL)
	reg RSTp; reg HOLD;
	//READ_ALL=1;
	//Inputs from spi_adc block
	reg [15:0] DATA_READ;
	
	
	//Holders
	reg countBits; //Holds count of number of bits read during each transmission (to find start and end)
	reg [1:0] chID; //Channel ID
	reg [12:0] data; //Data read (all measurements)
	//Measured values... TO DO: convert to outputs of this ADC block later
	reg [12:0] Vout;
	reg [12:0] Temp;
	reg [12:0] Vin;
	reg [12:0] Iout;
	
	//Use SPI interface to get data from ADC
	spi_ad7324(DOUT,DIN,CS,CLK20M,RSTp,SCLK,DATA_READ,HOLD,1);
	
	
	//State Machine
   DFFA	   #(
                 .WORD(4)
           )
           U6(   .CLK(SCLK), 	//USE 20MHz SCLK from PLL
                 .R(1),			//ALWAYS RUN STATE MACHINE
                 .D(sm_next),
                 .Q(sm_state)
             );
	
	always@(*)
	   begin
	   
		  case(sm_state)
			 STATE_RESET:
			 begin
				if(!rst)
				   sm_next <= STATE_START;
				else
				   sm_next <= STATE_RESET;
			 end

			 STATE_START:
			 begin
				if(!rst)
				   sm_next <= STATE_COUNT;
				else
				   sm_next <= STATE_RESET;
			 end

			 STATE_GETDATA:
			 begin
				if(!rst)
				   sm_next <= STATE_GETDATA;
				else
				   sm_next <= STATE_RESET;
			 end
			 
			endcase
		end
		
	
	always@(*)
		begin

			case(sm_state)

			 STATE_RESET:
			 begin
				RSTp<=1; 	 //Reset ADC
				HOLD<=0; //Do not start ADC measurements yet
			 end

			 STATE_START:
			 begin
				RSTp<=0;
				HOLD<=1; //Start ADC measurements (starting from CH0)
			 end
			 
			 STATE_GETDATA:
			 begin
				if(countBits<20)
					HOLD<=0;	//Turn off hold (pulse)
					countBits = countBits + 1; //Increment count
					
				else
					countBits=0; //Reset count at end
					chID = DATA_READ[14:13]; //Get channel ID
					data = DATA_READ[12:0]; //Get data
					
					case(chID)
						2'b00 Vout=data; 
						2'b01 Temp=data;
						2'b10 Vin=data;
						2'b11 Iout=data;
					endcase
					
			 end
			
			endcase
		end
	
endmodule