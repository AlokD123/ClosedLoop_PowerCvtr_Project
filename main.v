
//To test open-loop (fully automatic)
// Note: GPIO_O[32] is on top, [34] is on bottom

`include "ADC_read.v"
`include "pll.v"

	//Set value of M (ADC error output)
	`define M 12
	//Set resolution of bits of binary on LCD for each measurement
	`define M_LCD 8

module main(CLOCK_50,GPIO_0,SW,KEY,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,LCD_ON,LCD_BLON,LCD_RW,LCD_EN,LCD_RS,LCD_DATA, LEDR);
	//External input
	input CLOCK_50;
	input [7:0] SW;
	input [2:0] KEY;		
	output [6:0]HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7;
	output LCD_ON;
	output LCD_BLON;
	output LCD_RW;
	output LCD_EN;
	output LCD_RS;
	inout [7:0] LCD_DATA;
	
	output [15:0]LEDR;
	wire [15:0]LEDR_w;
	
	//input GPIO_0_DOUT;  //INPUT: D_OUT
	//Define switches
	wire EN_SW=SW[0];
	wire RSTn_SW=SW[1];
	//Ouputs
	inout [34:0]GPIO_0;
	
	//wire GPIOX;
	//assign GPIO_0[0]=GPIOX;
		
	//Inputs (set manually)
	reg [7:0] duty_8b; 			
	reg [3:0] freq_4b;					
	reg [2:0] dt1_3b; 
	reg [2:0] dt2_3b;
	
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
		freq_4b=4'b0111; 			//fs=140kHz
		dt1_3b=0; 
		dt2_3b=0; 		//No initial dead-time
		softStart = 0; 			//Ignore soft start for testing
		DIS_sig=0; 					//After reset complete, enable again
	end
	

	wire CLK20M; //Clock inputs
	//Use PLL to generate 20MHz clock
	pll GenClk20M(CLOCK_50,CLK20M);
	
	//Generate a pulse to switch measurement every 1/20MHz
	always@(posedge CLK20M) begin
		MEAS_SWITCH_PULSE=1;		//1 pulse every time (ALWAYS CYCLE)
	end
	//Read from ADC
//	ADC_read ADC1(CLOCK_50,CLK20M,GPIO_0[7:3],GPIO_0[1], MEAS_SWITCH_PULSE, KEY[1:0], Vout, Temp, Vin, Iout,
//	SW[1:0],HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,
//	LCD_ON,LCD_BLON,LCD_RW,LCD_EN,LCD_RS,LCD_DATA, LEDR_w);
	//assignment 
	assign LEDR = LEDR_w;
	
	
	//Set frequency and duty cycle in digital (count) form
	FreqConverter freqCvtr(freq_4b, CLOCK_50, maxcount);
	DutyCycleConverter DCCvtr(duty_8b, CLOCK_50, freq_4b, duty);
	
	//Adjust nominal duty cycle, "duty", to account for different value during soft starting
	AdjustDuty adjDC(CLOCK_50,resetn,softStart,duty,adjDutyCycle);//,GPIOX);
	//Run DPWM with this adjusted value
	DPWM runDPWM(CLOCK_50,resetn,EN,maxcount,adjDutyCycle,C_1,C_2);
	
	//Add dead time "AND" bit generation using shift registers
	shiftn deadTime1(3'b101, C_1, CLOCK_50, deadTime1_AndBit);
	shiftn deadTime2(3'b101, C_2, CLOCK_50, deadTime2_AndBit);

	//Create output signals
	assign GPIO_0[32] = C_1 & deadTime1_AndBit;
	assign GPIO_0[34] = C_2 & deadTime2_AndBit;
	
	
//	LCD_display lcd(CLOCK_50,KEY[2:0],SW,
//	HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,
//	4'h0F,4'h0F,4'h00,4'h00,4'h00,4'h00,4'h00,4'h00,
//	LCD_ON,LCD_BLON,LCD_RW,LCD_EN,LCD_RS,LCD_DATA);
	
	//Do load-step
	Load_step(CLOCK_50,SW[2],GPIO_0[28],GPIO_0[30]); //Use SW2 to enable stepping test mode

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
			3'b110: n=5;
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

module Load_step(CLOCK_50,EN,step,EN_driver);

	input CLOCK_50;
	input EN; 			 //Signal to enable and disable load step testing (changeable by key)
	output reg step;   	 //Load step transistor gating signal
	output reg EN_driver;//Enable signal for gate driver

	parameter step_count = 20'd500000;// Number of clock cycles before down step (using tclk=20ns and tstep=10ms)
	parameter count50Hz = 20'd1000000;// Number of clock cycles before down step (using tclk=20ns and trep=20ms)
	reg [19:0]counter;		//50Hz counter

	always@(posedge CLOCK_50) begin
		if(EN) begin
			EN_driver=1;
			// Divide clock to 50Hz
			if (counter < count50Hz) begin
				counter = counter + 1;	//count up every 20ns until reaching count50Hz
				if(counter>step_count) //Do down-step upon reaching end of settling time
					step = 0;
			end
			else begin
				counter = 0;		// reset upon reaching max (counter==count50Hz)
				step = 1;				//Up-step upon resetting
			end
		end
		else begin
			counter=0;
			EN_driver=0;
		end
	end

endmodule