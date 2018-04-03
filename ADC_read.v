//To read from ADC

`include "spi_ad7324_v2.v"
`include "LCD_display.v"

//Set value of M (ADC error output)
	`define M 12
	//Set resolution of bits of binary on LCD for each measurement
	`define M_LCD 8

module ADC_read(CLOCK_50,CLK20M,GPIO,GPIODOUT, MEAS_SWITCH_PULSE, KEY, Vout, Temp, Vin, Iout,
SW,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,
LCD_ON,LCD_BLON,LCD_RW,LCD_EN,LCD_RS,LCD_DATA, LEDR_in);
  
  //External inputs... from main
	input CLOCK_50;
	input CLK20M;
	input [1:0]KEY;
	output [15:0]LEDR_in;
	
	input [1:0] SW;		
	output [6:0]HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7;
	output LCD_ON;
	output LCD_BLON;
	output LCD_RW;
	output LCD_EN;
	output LCD_RS;
	inout [7:0] LCD_DATA;
	
	input MEAS_SWITCH_PULSE; //Signal to cycle between channels .... always@(posedge ) HOLD=MEAS_SWITCH_PULSE
	//External output... to main
	output reg [`M:0] Vout, Temp, Vin, Iout; //Measured (POSITIVE) ERROR values
	
	//Name GPIO pins
	`define D_OUT		GPIODOUT //OLD: In Pin Planner, renamed GPIO_0[1] to GPIO_0_DOUT
	`define D_IN 		GPIO[4]
	`define CS 			GPIO[2]
	`define SCLK		GPIO[0]
	//Number states
	parameter STATE_RESET         	= 4'd0;
	parameter STATE_START      		= 4'd1;
	parameter STATE_GETDATA 			= 4'd2;
	
	//Inputs from GPIO
	input GPIODOUT;  //INPUT: D_OUT
	//Outputs to GPIO
	output [4:0]GPIO; //OUTPUT: D_IN, CS and SCLK
	
	
	//Outputs to spi_adc block
	//input CLK20M
	reg RSTp; reg HOLD;
	//READ_ALL=1 ...................... Hardwired
	//Inputs from spi_adc block
	wire [15:0] DATA_READ;
	
	
	//Holders
	reg countBits; //Holds count of number of bits read during each transmission (to find start and end)
	reg [1:0] chID; //Channel ID
	reg [12:12-`M] errData; //Error data read (all measurements)
	reg [12:12-`M] posErrData; //Absolute value of error data read (all measurements)
	//LCD values
	reg [(`M_LCD-1):0] Vout_LCD, Temp_LCD, Vin_LCD, Iout_LCD;
	//State and next state values
	wire [3:0] sm_state;
	reg [3:0] sm_next;
	//Reset signal
	wire rstHI;
	
	
	//Use button 1 for ADC reset
	assign rstHI=~KEY[1];
	
	//Use SPI interface to get data from ADC
	spi_ad7324 read_ADC(`D_OUT,`D_IN,`CS,CLK20M,RSTp,`SCLK,DATA_READ,HOLD,1);
	
	assign LEDR_in = DATA_READ;
		
	
	//State Machine
   DFFA	   #(
                 .WORD(4)
           )
           U6(   .CLK(`SCLK), 	//USE 20MHz SCLK from PLL
                 .R(1),			//ALWAYS RUN STATE MACHINE
                 .D(sm_next),
                 .Q(sm_state)
             );
	
	always@(*)
	   begin
	   
		  case(sm_state)
			 STATE_RESET:
			 begin
				if(!rstHI)
				   sm_next <= STATE_START;
				else
				   sm_next <= STATE_RESET;
			 end

			 STATE_START:
			 begin
				if(!rstHI)
				   sm_next <= STATE_GETDATA;
				else
				   sm_next <= STATE_RESET;
			 end

			 STATE_GETDATA:
			 begin
				if(!rstHI)
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
				HOLD<=0; 	//Do not start ADC measurements yet
			 end

			 STATE_START:
			 begin
				RSTp<=0;
				HOLD<=1; //Start ADC measurements (starting from CH0)
			 end
			 
			 STATE_GETDATA:
			 begin
				if(countBits<20)
					begin
						HOLD<=0;	//Turn off hold (pulse)
						countBits = countBits + 1; //Increment count
					end
					
				else
					begin
						countBits=0; //Reset count at end
						chID = DATA_READ[14:13]; //Get channel ID
						errData = DATA_READ[12:12-`M]; //Get first M bits of error data (possibly lowered resolution) ... TO DO: PROVIDE TO COMPENSATOR
						
						begin //Convert to straight binary (positive) by adding LOWER BOUND VALUE (2C addition)
							case(`M)
								1 : posErrData=errData+2'b10;
								2 : posErrData=errData+3'b100;
								3 : posErrData=errData+4'b1000;
								4 : posErrData=errData+5'b10000;
								5 : posErrData=errData+6'b100000;
								6 : posErrData=errData+7'b1000000;
								7 : posErrData=errData+8'b10000000;
								8 : posErrData=errData+9'b100000000;
								9 : posErrData=errData+10'b1000000000;
								10 : posErrData=errData+11'b10000000000;
								11 : posErrData=errData+12'b100000000000;
								12 : posErrData=errData+13'b1000000000000;
							endcase
						end
							
						//TO DO: ACCOUNT FOR M<M_LCD cases (M_LCD being number of LCD digits available)
						case(chID)
							2'b00 : begin 
										Vout=posErrData;						//Output positive error data (first M bits)
										Vout_LCD=posErrData[`M:(`M-`M_LCD+1)]; 	//Display positive error data (first M_LCD bits)
									end
							2'b01 : begin Temp=posErrData; Temp_LCD=posErrData[`M:(`M-`M_LCD+1)]; end
							2'b10 : begin Vin=posErrData; Vin_LCD=posErrData[`M:(`M-`M_LCD+1)]; end
							2'b11 : begin Iout=posErrData; Iout_LCD=posErrData[`M:(`M-`M_LCD+1)]; end
						endcase
						
					end
					
			 end
			
			endcase
		end
		
	//Display on LCD
  LCD_display lcd(CLOCK_50,KEY,SW,
	HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,
	Vin_LCD[7:4],Vin_LCD[3:0],Vout_LCD[7:4],Vout_LCD[3:0],Iout_LCD[7:4],Iout_LCD[3:0],Temp_LCD[7:4],Temp_LCD[3:0],
	LCD_ON,LCD_BLON,LCD_RW,LCD_EN,LCD_RS,LCD_DATA);
	
endmodule