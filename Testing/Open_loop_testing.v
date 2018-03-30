//To test open-loop (fully automatic)
// Note: GPIO_O[32] is on top, [34] is on bottom

module main(clk, GPIO_0)
	//Inputs (set manually)
	reg [7:0] duty_8b;
	reg [4:0] freq_4b;
	reg [2:0] dt1_3b;
	reg [2:0] dt2_3b;

	//External input
	input clk;

	// Reset when ?? Disable when out of bounds
	reg EN;
	reg resetn;

	//Ouputs
	output [34:0]GPIO_0;

	//Holders
	reg [9:0] maxcount;		//Vm for DPWM.............................................. SET in FreqConverter block
	reg [9:0] duty;			//original duty cycle to set............................... SET in DutyCycleConverter block
	reg [9:0] adjDutyCycle; //ACTUAL duty cycle over time, accounting for soft start
	reg C_1, C_2;			//DPWM output values, before dead-time added
	reg deadTime1_AndBit, deadTime2_AndBit; //AND bits for generating dead time

	
	//MAIN INITIALIZATION VALUES............ Tune values HERE ONLY!
	initial begin
		// Initialize to fs=140kHz, duty=0.6
		duty_8b=8'b1001111; //Normalized to maxcount=250 --> dutyCount=150
		freq_4b=8'b0110;		//fs=140kHz
		adjDutyCycle = 10'b0; // Also initialize actual duty cycle to zero
	end

	
	//Set frequency and duty cycle in digital (count) form
	FreqConverter freqCvtr(freq_4b, clk, maxcount);
	DutyCycleConverter DCCvtr(duty_8b, clk, freq_4b, duty);
	
	//Adjust nominal duty cycle, "duty", to account for different value during soft starting
	AdjustDuty adjDC(clk,duty,adjDutyCycle);
	//Run DPWM with this adjusted value
	DPWM runDPWM(clk,resetn,EN,maxcount,adjDutyCycle,C_1,C_2);
	
	//Add dead time "AND" bit generation using shift registers
	shiftn deadTime1(dt1_3b, C_1, clk, deadTime1_AndBit);
	shiftn deadTime1(dt2_3b, C_2, clk, deadTime2_AndBit);

	//Create output signals
	assign GPIO_0[32] = bit_value1 & C_1;
	assign GPIO_0[34] = bit_value2 & C_2;

endmodule

// Creates PWM signal with two adjustable dead times.
// Note: SECOND state is C1=1,C2=0 ..... FIRST state is C1=0,C2=1. FIRST STATE COMES BEFORE SECOND STATE
module DPWM (clk,resetn,EN,maxcount,adjDutyCycle,C_1,C_2);
	input clk;
	input resetn; input EN;
	input [9:0] maxcount;
	input [9:0] adjDutyCycle;
	
	reg [9:0] counter; //counter for DPWM sawtooth
	
	output C_1; output C_2;
	
	initial begin
		counter = 10'b0; // Initialize to zero
	end
	
	
	always @(posedge clk, negedge resetn)
		begin
		   if (!resetn)		//If reset...
				begin
					C_1<=0;
					C_2<=0;
					counter<=0;
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

module AdjustDuty (clk,softStart,dutyCycle,adjDutyCycle);		//Adjusts duty cycle (provides adjusted value, to DPWM block)
	input clk;
	input softStart; 		//signal to indicate initial startup
	input [9:0] dutyCycle;	//original duty cycle to set
	
	output [9:0] adjDutyCycle; // Adjusted duty cycle, to set ACTUAL duty cycle (lower during soft start)
		
	
	always @(*)
		begin
			if(softStart) 	// TO DO: will need to be provided this signal until reached steady-state after startup
				begin
					if (adjDutyCycle < dutyCycle)			//In soft start, linearly increase adjDutyCycle until reached steady-state value(at time of steady-state)
						adjDutyCycle <= adjDutyCycle + 1;
					else
						adjDutyCycle <= dutyCycle;
					end
				end
			else
				begin 
					adjDutyCycle <= dutyCycle;
				end
				
		end


endmodule

module DutyCycleConverter (duty_8b, clk, frequency, duty);
	input clk;
	input [3:0] frequency;
	input [7:0] duty_8b;
	output [9:0] duty;

	always@ (*)
		case (frequency) /// TO DO: saturate duty to maxCount (duty = duty_8b...>maxCount ? maxCount : duty_8b...)
		
			4'b0000: duty = duty_8b*8'b0000_0100;//1000 and freq = 50khz //Don't go beyond ~1000!!!!!!!!!
			4'b0001: duty = duty_8b*8'b0000_0100;//769 and freq = 65khz
			4'b0010: duty = duty_8b*8'b0000_0100;//625 and freq = 80khz
			4'b0011: duty = duty_8b*8'b0000_0100;//526 and freq = 95khz
			4'b0100: duty = duty_8b*8'b0000_0010;//455 and freq = 110khz
			4'b0101: duty = duty_8b*8'b0000_0010;//400 and freq = 125khz
			4'b0110: duty = duty_8b*8'b0000_0010;//357 and freq = 140khz //Don't go beyond ~160!!!!!!!!!
			4'b0111: duty = duty_8b*8'b0000_0010;//322 and freq = 155khz
			4'b1000: duty = duty_8b*8'b0000_0010;//294 and freq = 170khz
			4'b1001: duty = duty_8b*8'b0000_0010; //270 and freq = 185khz
			4'b1010: duty = duty_8b*8'b0000_0001; //250 and freq = 200khz //Don't go beyond ~64!!!!!!!!!
			
			default: duty = duty_8b*8'b0000_0100;
		endcase	
endmodule 


module FreqConverter (freq_4b, clk, maxcount);
	input clk;
	input [3:0] freq_4b;
	output [9:0] maxcount;

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


	input w, Clock;

	reg [4:0]Q;
	integer k;
	output bit_value;
	always @(posedge Clock)
			begin
				for (k = 0; k < 4; k = k+1)
					Q[k] <= Q[k+1];
				
				Q[n] <= w;
				bit_value =Q[0];
			end

endmodule