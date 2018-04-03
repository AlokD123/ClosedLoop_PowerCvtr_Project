//To test open-loop (fully automatic)
// Note: GPIO_O[32] is on top, [34] is on bottom

//Set value of M (ADC error output)
	`define M 12
	//Set resolution of bits of binary on LCD for each measurement
	`define M_LCD 8

module main(CLOCK_50,GPIO_0,SW,KEY);
	//External input
	input CLOCK_50;
	input [1:0] SW;
	input [1:0] KEY;
	//Define switches
	wire EN_SW=SW[0];
	wire RSTn_SW=SW[1];
	//Ouputs
	output [34:0]GPIO_0;
	
	//wire GPIOX;
	//assign GPIO_0[0]=GPIOX;
		
	//Inputs (set manually)
	reg [7:0] duty_8b; 			
	reg [3:0] freq_4b;					
	reg [2:0] dt1_3b; reg [2:0] dt2_3b;
	
	//Holder variables
	wire [9:0] maxcount;								//Vm for DPWM.............................................. SET in FreqConverter block
	wire [9:0] duty;									//Original duty cycle to set................................ SET in DutyCycleConverter block
	wire [9:0] adjDutyCycle; 							//ACTUAL duty cycle over time, accounting for soft start
	wire C_1, C_2;										//DPWM output values, before dead-time added
	wire deadTime1_AndBit, deadTime2_AndBit; 			//"AND" bits for generating dead time
	reg softStart;										//Holds soft start state flag
	reg clkCount=0;										//First clock edge detection (for initial reset)
	reg MEAS_SWITCH_PULSE=0;							//Signal for cycling through measurements (if high)
	//Hold ADC measured data (ERROR VALUES)
	wire [`M:0] Vout, Temp, Vin, Iout; //Measured (POSITIVE) ERROR values
	
	
	// Reset when ?? Disable when out of bounds
	reg EN, resetn; 				//Reset on negedge and Enable registers
	reg DIS_sig, reset_sig; 	//Software tunable signals for DISABLE and Reset.
	//NOTES:
	//EN - EN_SW must be ON at start! BOTH enable signal AND EN_SW must be ON in order to enable!
	//resetn - RSTn_SW must be ON at start! EITHER NOT_reset signal OR RSTn_SW can fall in order to reset!
	always@(*) begin
		EN = (EN_SW && !DIS_sig);
		resetn = (RSTn_SW && !reset_sig);//.................................................. TO DO: reset_sig=1!!!!!!
	end
	
	//Reset on very first clock edge
	always@(posedge CLOCK_50) begin
		if(clkCount==0) begin
			clkCount=1;
			reset_sig=1; //Reset high
			reset_sig=0; //Come out of reset right away.................................................. TO DO: DECIDE IF RESET EDGE OR STATE!!!!!!
		end
	end
	//Reset procedure
	always@(negedge resetn) begin
		//MAIN INITIALIZATION VALUES............ Tune values HERE ONLY! (Initialized at reset)
		// Initialize to fs=140kHz, duty=0.6
		duty_8b=8'b01001111;		//Normalized to maxcount=250 --> dutyCount=150
		freq_4b=4'b0110; 			//fs=140kHz
		dt1_3b=0; dt2_3b=0; 		//No initial dead-time
		softStart = 1; 			//Ignore soft start for testing
		DIS_sig=0; 					//After reset complete, enable again
	end
	

	//Use PLL to generate 20MHz clock
	pll GenClk20(CLOCK_50,CLK20M);
	
	//Generate a pulse to switch measurement every 1/20MHz
	always@(posedge CLK20M) begin
		MEAS_SWITCH_PULSE=1;		//1 pulse every time (ALWAYS CYCLE)
	end
	//Read from ADC
	ADC_read ADC1(CLOCK_50,CLK20M,GPIO_0,GPIO_0_DOUT, MEAS_SWITCH_PULSE, KEY, Vout, Temp, Vin, Iout);
	
	//Set frequency and duty cycle in digital (count) form
	FreqConverter freqCvtr(freq_4b, CLOCK_50, maxcount);
	DutyCycleConverter DCCvtr(duty_8b, CLOCK_50, freq_4b, duty);
	
	//Adjust nominal duty cycle, "duty", to account for different value during soft starting
	AdjustDuty adjDC(CLOCK_50,resetn,softStart,duty,adjDutyCycle);//,GPIOX);
	//Run DPWM with this adjusted value
	DPWM runDPWM(CLOCK_50,resetn,EN,maxcount,adjDutyCycle,C_1,C_2);
	
	//Add dead time "AND" bit generation using shift registers
	shiftn deadTime1(dt1_3b, C_1, CLOCK_50, deadTime1_AndBit);
	shiftn deadTime2(dt2_3b, C_2, CLOCK_50, deadTime2_AndBit);

	//Create output signals
	assign GPIO_0[32] = C_1 & deadTime1_AndBit;
	assign GPIO_0[34] = C_2 & deadTime2_AndBit;

endmodule

// Creates PWM signal with two adjustable dead times.
// Note: SECOND state is C1=1,C2=0 ..... FIRST state is C1=0,C2=1. FIRST STATE COMES BEFORE SECOND STATE
module DPWM (CLOCK_50,resetn,EN,maxcount,adjDutyCycle,C_1,C_2);
	input CLOCK_50;
	input resetn; input EN;
	input [9:0] maxcount;
	input [9:0] adjDutyCycle;
	
	reg [9:0] counter; //counter for DPWM sawtooth
	
	output reg C_1; output reg C_2;
		
	
	always @(posedge CLOCK_50, negedge resetn)
		begin
		   if (!resetn)		//If reset...
				begin
					C_1<=0;
					C_2<=0;
					//MAIN INITIALIZATION VALUE
					counter<=0; // Initialize to zero
				end
			else if(!EN) 	//If disabled...
				begin
					C_1<=0;
					C_2<=0;
				end 
			else
				begin 
					if (counter < maxcount) // Create sawtooth
						counter = counter + 1;	//count up every 20ns until reaching maxCount
					else 
						counter = 0;		// reset upon reaching max (counter==maxcount)
					//Based on required duty cycle at current time (possibly before steady-state), toggle switches
					if (counter > adjDutyCycle) 
						begin					//For SECOND state....
						   C_2<=1'b0;
							C_1 <= 1'b1;
						end 
					else	
						begin					//For FIRST state....
							C_1<=1'b0;
							C_2 <= 1'b1;
						end

				end
				
		end

endmodule

module AdjustDuty (CLOCK_50,resetn,softStart,dutyCycle,adjDutyCycle);//,GPIOX);		//Adjusts duty cycle (provides adjusted value, to DPWM block)
	input CLOCK_50;
	input softStart; 			//signal to indicate initial startup
	input [9:0] dutyCycle;	//original duty cycle to set
	input resetn;	
	//output reg GPIOX;
	
	
	output reg [9:0] adjDutyCycle; // Adjusted duty cycle, to set ACTUAL duty cycle (lower during soft start)
	
		
	always @(posedge CLOCK_50)
		begin
			if(!resetn)
				//MAIN INITIALIZATION VALUE
				adjDutyCycle = 10'b0000000000; // Upon reset, initialize actual duty cycle to zero
			else begin
				if(softStart) 	// TO DO: will need to be provided this signal until reached steady-state after startup
					begin
						if (adjDutyCycle < dutyCycle)			//In soft start, linearly increase adjDutyCycle until reached steady-state value(at time of steady-state)
							begin
								adjDutyCycle <= adjDutyCycle + 1;//Count up every 20ns for linear increase
								//GPIOX = 0; 
							end
						else begin
								adjDutyCycle <= dutyCycle;
								//GPIOX = 1; //Check that soft start working
							end
					end
				else
					begin 
						adjDutyCycle <= dutyCycle;
					end
			end
				
		end


endmodule

module DutyCycleConverter (duty_8b, CLOCK_50, frequency, dutyCount);
	input CLOCK_50;
	input [3:0] frequency;
	input [7:0] duty_8b;
	output reg [9:0] dutyCount;
	
	always@ (*)
		case (frequency) /// TO DO: saturate duty to maxCount (duty = duty_8b...>maxCount ? maxCount : duty_8b...)
		
			4'b0000: dutyCount = duty_8b*8'b0000_0100;//1000 and freq = 50khz //Don't go beyond ~1000!!!!!!!!!
			4'b0001: dutyCount = duty_8b*8'b0000_0100;//769 and freq = 65khz
			4'b0010: dutyCount = duty_8b*8'b0000_0100;//625 and freq = 80khz
			4'b0011: dutyCount = duty_8b*8'b0000_0100;//526 and freq = 95khz
			4'b0100: dutyCount = duty_8b*8'b0000_0010;//455 and freq = 110khz
			4'b0101: dutyCount = duty_8b*8'b0000_0010;//400 and freq = 125khz
			4'b0110: dutyCount = duty_8b*8'b0000_0010;//357 and freq = 140khz //Don't go beyond ~160!!!!!!!!!
			4'b0111: dutyCount = duty_8b*8'b0000_0010;//322 and freq = 155khz
			4'b1000: dutyCount = duty_8b*8'b0000_0010;//294 and freq = 170khz
			4'b1001: dutyCount = duty_8b*8'b0000_0010; //270 and freq = 185khz
			4'b1010: dutyCount = duty_8b*8'b0000_0001; //250 and freq = 200khz //Don't go beyond ~64!!!!!!!!!
			
			default: dutyCount = duty_8b*8'b0000_0100;
		endcase	
endmodule 


module FreqConverter(freq_4b, CLOCK_50, maxcount);
	input CLOCK_50;
	input [3:0] freq_4b;
	output reg [9:0] maxcount;
	
	always@ (*)
		case (freq_4b)
		
			4'b0000: maxcount=10'b1111101000;//1000 and freq = 50khz
			4'b0001: maxcount=10'b1100000001;//769 and freq = 65khz
			4'b0010: maxcount=10'b1001110001;//625 and freq = 80khz
			4'b0011: maxcount=10'b1000001110;//526 and freq = 95khz
			4'b0100: maxcount=10'b0111000111;//455 and freq = 110khz
			4'b0101: maxcount=10'b0110010000;//400 and freq = 125khz
			4'b0110: maxcount=10'b0101100101;//357 and freq = 140khz
			4'b0111: maxcount=10'b0101000010;//322 and freq = 155khz
			4'b1000: maxcount=10'b0100100110;//294 and freq = 170khz
			4'b1001: maxcount=10'b0100001110;//270 and freq = 185khz
			4'b1010: maxcount=10'b0011111010;//250 and freq = 200khz
			
			default: maxcount=10'b1111101000;
		endcase

endmodule

//Shift register, to create dead time "AND" bits
module shiftn (binary, w, Clock, bit_value);
	input [2:0]binary;
	input w, Clock;
	output reg bit_value;
	
	integer n;
	always@ (binary)
		case (binary)
		
			3'b001: n=0;
			3'b010: n=1;
			3'b011: n=2;
			3'b100: n=3;
			3'b101: n=4;
			default: n=0;
		endcase



	reg [4:0]Q;
	integer k;
	
	always @(posedge Clock)
			begin
				for (k = 0; k < 4; k = k+1)
					Q[k] <= Q[k+1];
				
				Q[n] <= w;
				bit_value =Q[0];
			end

endmodule













module ADC_read(CLOCK_50,CLK20M,GPIO_0,GPIO_0_DOUT, MEAS_SWITCH_PULSE, KEY, Vout, Temp, Vin, Iout);
  
  //External inputs... from main
	input CLOCK_50;
	input CLK20M;
	input [1:0]KEY;
	input MEAS_SWITCH_PULSE; //Signal to cycle between channels .... always@(posedge ) HOLD=MEAS_SWITCH_PULSE
	//External output... to main
	output reg [`M:0] Vout, Temp, Vin, Iout; //Measured (POSITIVE) ERROR values
	
	//Name GPIO pins
	`define D_OUT		GPIO_0_DOUT //In Pin Planner, renamed GPIO_0[1] to GPIO_0_DOUT!!!!!!!!!!!!!!!!!!!!!!!!!!!
	`define D_IN 		GPIO_0[7]
	`define CS 			GPIO_0[5]
	`define SCLK		GPIO_0[3]
	//Number states
	parameter STATE_RESET         	= 4'd0;
	parameter STATE_START      		= 4'd1;
	parameter STATE_GETDATA 		= 4'd2;
	
	//Inputs from GPIO
	input GPIO_0_DOUT;  //INPUT: D_OUT
	//Outputs to GPIO
	output [7:3]GPIO_0; //OUTPUT: D_IN, CS and SCLK
	
	
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
	assign rstHI=KEY[1];
	
	//Use SPI interface to get data from ADC
	spi_ad7324 read_ADC(`D_OUT,`D_IN,`CS,CLK20M,RSTp,`SCLK,DATA_READ,HOLD,1);
		
	
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








module LCD_display(
  input CLOCK_50,    //    50 MHz clock
  input [0:0] KEY,      //    Pushbutton[0]
  input [1:0] SW,		//Using 2 switches
  output [6:0]    HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,  // Seven Segment Digits
  input [3:0]	VinH,VinL,VoutH,VoutL,IoutH,IoutL,TempH,TempL,
  output [1:0] LEDR,  //    LED Red
//    LCD Module 16X2
  output LCD_ON,    // LCD Power ON/OFF
  output LCD_BLON,    // LCD Back Light ON/OFF
  output LCD_RW,    // LCD Read/Write Select, 0 = Write, 1 = Read
  output LCD_EN,    // LCD Enable
  output LCD_RS,    // LCD Command/Data Select, 0 = Command, 1 = Data
  inout [7:0] LCD_DATA    // LCD Data bus 8 bits
);
//Add reset button
wire RST;
assign RST = KEY[0];

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(    .iCLK(CLOCK_50),.oRESET(DLY_RST) );

// Send switches to red leds 
//assign LEDR = SW[1:0];

// turn LCD ON
assign    LCD_ON        =    1'b1;
assign    LCD_BLON    =    1'b1;

//Take ADC input 
//OPTION1: 8 bits x 4 measurements
//Will write to each output unit as hex character, not bit, so padded with 3 zeros at start
/* reg [3:0] hex[31:0]; //32 output units, each a hex character
wire [3:0] output[31:0];
integer i, j;
always@(*) begin
    for (i = 0; i < 3; i = i +1) begin
        hex[i][0]=VinL[i];
        for (j = 1; j < 3; j = j +1) begin
            hex[i][j]=1'b0;
        end
    end
 end
assign output=hex; */

//OPTION2: 2 hex x 4 measurement (+names)
assign hex0 = VinL;
assign hex1 = VinH;
assign hex2 = VoutL;
assign hex3 = VoutH;
assign hex4 = IoutL;
assign hex5 = IoutH;
assign hex6 = TempL;
assign hex7 = TempH;

LCD_Display u1(
// Host Side
   .iCLK_50MHZ(CLOCK_50),
   .iRST_N(DLY_RST),
   .hex0(hex0),.hex1(hex1),.hex2(hex2),.hex3(hex3),
   .hex4(hex4),.hex5(hex5),.hex6(hex6),.hex7(hex7),
// LCD Side
   .DATA_BUS(LCD_DATA),
   .LCD_RW(LCD_RW),
   .LCD_E(LCD_EN),
   .LCD_RS(LCD_RS)
);


// blank unused 7-segment digits
assign HEX0 = 7'b111_1111;
assign HEX1 = 7'b111_1111;
assign HEX2 = 7'b111_1111;
assign HEX3 = 7'b111_1111;
assign HEX4 = 7'b111_1111;
assign HEX5 = 7'b111_1111;
assign HEX6 = 7'b111_1111;
assign HEX7 = 7'b111_1111;

endmodule





module LCD_Display(iCLK_50MHZ, iRST_N, hex0, hex1, hex2, hex3, hex4, hex5, hex6, hex7, LCD_RS,LCD_E,LCD_RW,DATA_BUS);
	input iCLK_50MHZ, iRST_N;
	input [3:0] hex0, hex1, hex2, hex3, hex4, hex5, hex6, hex7;
	output LCD_RS, LCD_E, LCD_RW;
	inout [7:0] DATA_BUS;

parameter
HOLD = 4'h0,
FUNC_SET = 4'h1,
DISPLAY_ON = 4'h2,
MODE_SET = 4'h3,
Print_String = 4'h4,
LINE2 = 4'h5,
RETURN_HOME = 4'h6,
DROP_LCD_E = 4'h7,
RESET1 = 4'h8,
RESET2 = 4'h9,
RESET3 = 4'ha,
DISPLAY_OFF = 4'hb,
DISPLAY_CLEAR = 4'hc;

reg [3:0] state, next_command;
// Enter new ASCII hex data above for LCD Display
reg [7:0] DATA_BUS_VALUE;
wire [7:0] Next_Char;
reg [19:0] CLK_COUNT_400HZ;
reg [4:0] CHAR_COUNT;
reg CLK_400HZ, LCD_RW_INT, LCD_E, LCD_RS;

// BIDIRECTIONAL TRI STATE LCD DATA BUS
assign DATA_BUS = (LCD_RW_INT? 8'bZZZZZZZZ: DATA_BUS_VALUE);

LCD_display_string u1(
.index(CHAR_COUNT),
.out(Next_Char),
.hex0(hex0),.hex1(hex1),.hex2(hex2),.hex3(hex3),
.hex4(hex4),.hex5(hex5),.hex6(hex6),.hex7(hex7)
 );

assign LCD_RW = LCD_RW_INT;

always @(posedge iCLK_50MHZ or negedge iRST_N)
    if (!iRST_N)
    begin
       CLK_COUNT_400HZ <= 20'h00000;
       CLK_400HZ <= 1'b0;
    end
    else if (CLK_COUNT_400HZ < 20'h0F424)
    begin
       CLK_COUNT_400HZ <= CLK_COUNT_400HZ + 1'b1;
    end
    else
    begin
      CLK_COUNT_400HZ <= 20'h00000;
      CLK_400HZ <= ~CLK_400HZ;
    end
// State Machine to send commands and data to LCD DISPLAY

always @(posedge CLK_400HZ or negedge iRST_N)
    if (!iRST_N)
    begin
     state <= RESET1;
    end
    else
    case (state)
    RESET1:            
// Set Function to 8-bit transfer and 2 line display with 5x8 Font size
// see Hitachi HD44780 family data sheet for LCD command and timing details
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= RESET2;
      CHAR_COUNT <= 5'b00000;
    end
    RESET2:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= RESET3;
    end
    RESET3:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= FUNC_SET;
    end
// EXTRA STATES ABOVE ARE NEEDED FOR RELIABLE PUSHBUTTON RESET OF LCD

    FUNC_SET:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h38;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_OFF;
    end

// Turn off Display and Turn off cursor
    DISPLAY_OFF:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h08;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_CLEAR;
    end

// Clear Display and Turn off cursor
    DISPLAY_CLEAR:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h01;
      state <= DROP_LCD_E;
      next_command <= DISPLAY_ON;
    end

// Turn on Display and Turn off cursor
    DISPLAY_ON:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h0C;
      state <= DROP_LCD_E;
      next_command <= MODE_SET;
    end

// Set write mode to auto increment address and move cursor to the right
    MODE_SET:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h06;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// Write ASCII hex character in first LCD character location
    Print_String:
    begin
      state <= DROP_LCD_E;
      LCD_E <= 1'b1;
      LCD_RS <= 1'b1;
      LCD_RW_INT <= 1'b0;
    // ASCII character to output
      if (Next_Char[7:4] != 4'h0)
        DATA_BUS_VALUE <= Next_Char;
        // Convert 4-bit value to an ASCII hex digit
      else if (Next_Char[3:0] >9)
        // ASCII A...F
         DATA_BUS_VALUE <= {4'h4,Next_Char[3:0]-4'h9};
      else
        // ASCII 0...9
         DATA_BUS_VALUE <= {4'h3,Next_Char[3:0]};
    // Loop to send out 32 characters to LCD Display  (16 by 2 lines)
      if ((CHAR_COUNT < 31) && (Next_Char != 8'hFE))
         CHAR_COUNT <= CHAR_COUNT + 1'b1;
      else
         CHAR_COUNT <= 5'b00000; 
    // Jump to second line?
      if (CHAR_COUNT == 15)
        next_command <= LINE2;
    // Return to first line?
      else if ((CHAR_COUNT == 31) || (Next_Char == 8'hFE))
        next_command <= RETURN_HOME;
      else
        next_command <= Print_String;
    end

// Set write address to line 2 character 1
    LINE2:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'hC0;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// Return write address to first character postion on line 1
    RETURN_HOME:
    begin
      LCD_E <= 1'b1;
      LCD_RS <= 1'b0;
      LCD_RW_INT <= 1'b0;
      DATA_BUS_VALUE <= 8'h80;
      state <= DROP_LCD_E;
      next_command <= Print_String;
    end

// The next three states occur at the end of each command or data transfer to the LCD
// Drop LCD E line - falling edge loads inst/data to LCD controller
    DROP_LCD_E:
    begin
      LCD_E <= 1'b0;
      state <= HOLD;
    end
// Hold LCD inst/data valid after falling edge of E line                
    HOLD:
    begin
      state <= next_command;
    end
    endcase
endmodule

module LCD_display_string(index,out,hex0,hex1, hex2, hex3, hex4, hex5, hex6, hex7);
input [4:0] index;
input [3:0] hex0, hex1, hex2, hex3, hex4, hex5, hex6, hex7;
output [7:0] out;
reg [7:0] out;
// ASCII hex values for LCD Display
// Enter Live Hex Data Values from hardware here
// LCD DISPLAYS THE FOLLOWING:
//----------------------------
//| Vin=hh Vout=hh			  |
//| Iout=hh Temp=hh           |
//----------------------------
// Line 1
   always 
     case (index)
    5'h00: out <= 8'h56;
    5'h01: out <= 8'h69;
    5'h02: out <= 8'h6E;
    5'h03: out <= 8'h3D;
    5'h04: out <= {4'h0,hex1};
    5'h05: out <= {4'h0,hex0};
    5'h06: out <= 8'h56;
    5'h07: out <= 8'h6F;
    5'h08: out <= 8'h75;
	5'h09: out <= 8'h74;
    5'h0A: out <= 8'h3D;
	5'h0B: out <= {4'h0,hex3};
    5'h0C: out <= {4'h0,hex2};
// Line 2
    5'h10: out <= 8'h49;
    5'h11: out <= 8'h6F;
    5'h12: out <= 8'h75;
	5'h13: out <= 8'h74;
    5'h14: out <= 8'h3D;
	5'h15: out <= {4'h0,hex5};
    5'h16: out <= {4'h0,hex4};
	5'h17: out <= 8'h54;
    5'h18: out <= 8'h65;
    5'h19: out <= 8'h6D;
	5'h1A: out <= 8'h70;
    5'h1B: out <= 8'h3D;
	5'h1C: out <= {4'h0,hex7};
    5'h1D: out <= {4'h0,hex6};
    default: out <= 8'h20;
     endcase
endmodule




module    Reset_Delay(iCLK,oRESET);
input        iCLK;
output reg    oRESET;
reg    [19:0]    Cont;

always@(posedge iCLK)
begin
    if(Cont!=20'hFFFFF)
    begin
        Cont    <=    Cont+1'b1;
        oRESET    <=    1'b0;
    end
    else
    oRESET    <=    1'b1;
end

endmodule




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





//-----2 Input Mux-----
module MUX2(A, B, S, O);
	
	//Input Definitions
	input		A, B;
	input[1:0]	S;
	
	//Output Defintions
	output O;
	
	//Clocked Logic
	reg O;
	
	always@(A or B or S)
	begin
		case(S)
			2'b10		:	O = A;
			2'b01		:	O = B;
			default	:	O = 1'b0;
		endcase
	end

endmodule

//-----2 Input Arrayed Mux-----
module MUX2A(A, B, S, O);
	
	//Parameters
	parameter	WORD	=	10;
	
	//Input Definitions
	input[(WORD-1):0]	A, B;
	input[1:0]			S;
	
	//Output Definitions
	output[(WORD-1):0]	O;
	
	//n Parallel MUXs
	genvar i;
	generate
		for(i = 0; i < WORD; i = i + 1)
		begin : MUX2
			MUX2 U1(		.A(A[i]),
							.B(B[i]), 
							.S(S), 
							.O(O[i])
						);
		end
	endgenerate
	
endmodule

//-----3 Input Mux-----
module MUX3(A, B, C, S, O);
	
	//Input Definitions
	input		A, B, C;
	input[2:0]	S;
	
	//Output Defintions
	output		O;
	
	//Clocked Logic
	reg			O;

	always@(A or B or C or S)
	begin
		case(S)
			3'b100	:	O = A;
			3'b010	:	O = B;
			3'b001	:	O = C;
			default	:	O = 1'b0;
		endcase
	end

endmodule

//-----3 Input Arrayed Mux-----
module MUX3A(A, B, C, S, O);

	//Parameters
	parameter	WORD	=	10;
	
	//Input Definitions
	input[(WORD-1):0]	A, B, C;
	input[2:0]			S;
	
	//Output Definitions
	output[(WORD-1):0]	O;

	//n Parallel MUXs
	genvar i;
	generate
		for(i = 0; i < WORD; i = i + 1)
		begin : MUX3
		
			MUX3 U1(	.A(A[i]),
						.B(B[i]),
						.C(C[i]),
						.S(S),
						.O(O[i])
					);
						
		end
	endgenerate
	
endmodule

//-----4 Input Mux-----
module MUX4(A, B, C, D, S, O);
	
	//Input Definitions
	input		A, B, C, D;
	input[3:0]	S;
	
	//Output Defintions
	output		O;
	
	//Clocked Logic
	reg			O;

	always@(A or B or C or D or S)
	begin
		case(S)
			4'b1000	:	O = A;
			4'b0100	:	O = B;
			4'b0010	:	O = C;
			4'b0001	:	O = D;
			default	:	O = 1'b0;
		endcase
	end

endmodule

//-----4 Input Arrayed Mux-----
module MUX4A(A, B, C, D, S, O);

	//Parameters
	parameter	WORD	=	10;
	
	//Input Definitions
	input[(WORD-1):0]	A, B, C, D;
	input[3:0]			S;
	
	//Output Definitions
	output[(WORD-1):0]	O;

	//n Parallel MUXs
	genvar i;
	generate
		for(i = 0; i < WORD; i = i + 1)
		begin : MUX4
		
			MUX4 U1(		.A(A[i]),
							.B(B[i]),
							.C(C[i]),
							.D(D[i]),
							.S(S),
							.O(O[i])
						);
						
		end
	endgenerate
	
endmodule

//-----5 Input Mux-----
module MUX5(A, B, C, D, E, S, O);
	
	//Input Defintions
	input		A, B, C, D, E;
	input[4:0]	S;
	
	//Output Definitions
	output		O;
	
	//Clocked Logic
	reg			O;

	always@(A or B or C or D or E or S)
	begin
		case(S)
			5'b10000	:	O = A;
			5'b01000	:	O = B;
			5'b00100	:	O = C;
			5'b00010	:	O = D;
			5'b00001	:	O = E;
			default	:	O = 1'b0;
		endcase
	end

endmodule

//-----5 Input Arrayed Mux-----
module MUX5A(A, B, C, D, E, S, O);

	//Parameters
	parameter	WORD	=	10;
	
	//Input definitions
	input[(WORD-1):0]	A, B, C, D, E;
	input[4:0]			S;
	
	//Output defintions
	output[(WORD-1):0]	O;
	
	//n Parallel MUXs
	genvar i;
	generate
		for(i = 0; i < WORD; i = i + 1)
		begin : MUX5
		
			MUX5 U1(		.A(A[i]),
							.B(B[i]),
							.C(C[i]),
							.D(D[i]),
							.E(E[i]),
							.S(S),
							.O(O[i])
						);
						
		end
	endgenerate
	
endmodule

//-----DFF-----
module DFFS(CLK, R, D, Q);

	//Input definitions
	input	CLK, R, D;

	//Output defintions
	output	Q;	
	
	//Clocked Logic
	reg		Q;

	always@(posedge CLK or negedge R)
	begin
		if(~R)
		begin
			Q <= 1'b0;
		end
		else
		begin
			Q <= D; 
		end
	end

endmodule

//-----DFF with negative clock edge-----
module DFFSN(CLK, R, D, Q);

        //Input definitions
        input   CLK, R, D;

        //Output defintions
        output  Q;

        //Clocked Logic
        reg             Q;

        always@(negedge CLK or negedge R)
        begin
                if(~R)
                begin
                        Q <= 1'b0;
                end
                else
                begin
                        Q <= D;
                end
        end

endmodule


//-----Arrayed DFF-----
module DFFA(CLK, R, D, Q);

	//Parameters
	parameter	WORD	=	10;
	
	//Input defintions
	input 				CLK, R;
	input[(WORD-1):0]	D;
	
	//Output Defintions
	output[(WORD-1):0]	Q;
	
	//n Parallel DFFs
	genvar i;
	generate
		for(i = 0; i < WORD; i = i + 1)
		begin : DFF
			DFFS U1 (		.CLK(CLK),
							.R(R),
							.D(D[i]),
							.Q(Q[i])
						);
		end
	endgenerate

endmodule

//-----Arrayed DFF with negative clock edge-----
module DFFAN(CLK, R, D,  Q);

        //Parameters
        parameter       WORD    =       10;

        //Input defintions
        input                           CLK, R;
        input[(WORD-1):0]       D;

        //Output Defintions
        output[(WORD-1):0]      Q;

        //n Parallel DFFs
        genvar i;
        generate
                for(i = 0; i < WORD; i = i + 1)
                begin : dff
                        DFFSN U1 (              .CLK(CLK),
                                                        .R(R),
                                                        .D(D[i]),
                                                        .Q(Q[i])
                                                );
                end
        endgenerate

endmodule






module pll (
	inclk0,
	c0);

	input	  inclk0;
	output	  c0;

	wire [5:0] sub_wire0;
	wire [0:0] sub_wire4 = 1'h0;
	wire [0:0] sub_wire1 = sub_wire0[0:0];
	wire  c0 = sub_wire1;
	wire  sub_wire2 = inclk0;
	wire [1:0] sub_wire3 = {sub_wire4, sub_wire2};

	altpll	altpll_component (
				.inclk (sub_wire3),
				.clk (sub_wire0),
				.activeclock (),
				.areset (1'b0),
				.clkbad (),
				.clkena ({6{1'b1}}),
				.clkloss (),
				.clkswitch (1'b0),
				.configupdate (1'b0),
				.enable0 (),
				.enable1 (),
				.extclk (),
				.extclkena ({4{1'b1}}),
				.fbin (1'b1),
				.fbmimicbidir (),
				.fbout (),
				.fref (),
				.icdrclk (),
				.locked (),
				.pfdena (1'b1),
				.phasecounterselect ({4{1'b1}}),
				.phasedone (),
				.phasestep (1'b1),
				.phaseupdown (1'b1),
				.pllena (1'b1),
				.scanaclr (1'b0),
				.scanclk (1'b0),
				.scanclkena (1'b1),
				.scandata (1'b0),
				.scandataout (),
				.scandone (),
				.scanread (1'b0),
				.scanwrite (1'b0),
				.sclkout0 (),
				.sclkout1 (),
				.vcooverrange (),
				.vcounderrange ());
	defparam
		altpll_component.clk0_divide_by = 5,
		altpll_component.clk0_duty_cycle = 50,
		altpll_component.clk0_multiply_by = 2,
		altpll_component.clk0_phase_shift = "0",
		altpll_component.compensate_clock = "CLK0",
		altpll_component.inclk0_input_frequency = 20000,
		altpll_component.intended_device_family = "Cyclone II",
		altpll_component.lpm_hint = "CBX_MODULE_PREFIX=pll",
		altpll_component.lpm_type = "altpll",
		altpll_component.operation_mode = "NORMAL",
		altpll_component.port_activeclock = "PORT_UNUSED",
		altpll_component.port_areset = "PORT_UNUSED",
		altpll_component.port_clkbad0 = "PORT_UNUSED",
		altpll_component.port_clkbad1 = "PORT_UNUSED",
		altpll_component.port_clkloss = "PORT_UNUSED",
		altpll_component.port_clkswitch = "PORT_UNUSED",
		altpll_component.port_configupdate = "PORT_UNUSED",
		altpll_component.port_fbin = "PORT_UNUSED",
		altpll_component.port_inclk0 = "PORT_USED",
		altpll_component.port_inclk1 = "PORT_UNUSED",
		altpll_component.port_locked = "PORT_UNUSED",
		altpll_component.port_pfdena = "PORT_UNUSED",
		altpll_component.port_phasecounterselect = "PORT_UNUSED",
		altpll_component.port_phasedone = "PORT_UNUSED",
		altpll_component.port_phasestep = "PORT_UNUSED",
		altpll_component.port_phaseupdown = "PORT_UNUSED",
		altpll_component.port_pllena = "PORT_UNUSED",
		altpll_component.port_scanaclr = "PORT_UNUSED",
		altpll_component.port_scanclk = "PORT_UNUSED",
		altpll_component.port_scanclkena = "PORT_UNUSED",
		altpll_component.port_scandata = "PORT_UNUSED",
		altpll_component.port_scandataout = "PORT_UNUSED",
		altpll_component.port_scandone = "PORT_UNUSED",
		altpll_component.port_scanread = "PORT_UNUSED",
		altpll_component.port_scanwrite = "PORT_UNUSED",
		altpll_component.port_clk0 = "PORT_USED",
		altpll_component.port_clk1 = "PORT_UNUSED",
		altpll_component.port_clk2 = "PORT_UNUSED",
		altpll_component.port_clk3 = "PORT_UNUSED",
		altpll_component.port_clk4 = "PORT_UNUSED",
		altpll_component.port_clk5 = "PORT_UNUSED",
		altpll_component.port_clkena0 = "PORT_UNUSED",
		altpll_component.port_clkena1 = "PORT_UNUSED",
		altpll_component.port_clkena2 = "PORT_UNUSED",
		altpll_component.port_clkena3 = "PORT_UNUSED",
		altpll_component.port_clkena4 = "PORT_UNUSED",
		altpll_component.port_clkena5 = "PORT_UNUSED",
		altpll_component.port_extclk0 = "PORT_UNUSED",
		altpll_component.port_extclk1 = "PORT_UNUSED",
		altpll_component.port_extclk2 = "PORT_UNUSED",
		altpll_component.port_extclk3 = "PORT_UNUSED";


endmodule