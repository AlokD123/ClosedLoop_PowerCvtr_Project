// Creates PWM signal with two adjustable dead times.
// Note: SECOND state is C1=1,C2=0 ..... FIRST state is C1=0,C2=1. FIRST STATE COMES BEFORE SECOND STATE
// Note: GPIO_O[32] is on top, [34] is on bottom
// Mubarek version (shift registers)

module main (SW, KEY, clk, GPIO_0);
input [17:0] SW;
input [1:0] KEY;//KEY[0]=resetn and KEY[1]= EN
input clk;
reg C1, C2;
output [34:0]GPIO_0;
reg [9:0] maxcount;

always@ (*)
	case (SW[3:0])
	
		4'b0000: maxcount=10'b1111101000;//1000 and freq = 50khz
		4'b0001: maxcount=10'b1100000001;//769 and freq = 65khz
		4'b0010: maxcount=10'b1001110001;//625 and freq = 80khz
		4'b0011: maxcount=10'b1000001110;//526 and freq = 95khz
		4'b0101: maxcount=10'b0110010000;//400 and freq = 125khz
		4'b0110: maxcount=10'b0101100101;//357 and freq = 140khz
		4'b0111: maxcount=10'b0101000010;//322 and freq = 155khz
		4'b1000: maxcount=10'b0100100110;//294 and freq = 170khz
		4'b1001: maxcount=10'b0100001110;//270 and freq = 185khz
		4'b1010: maxcount=10'b0011111010;//250 and freq = 200khz
		
		default: maxcount=10'b1111101000;
	endcase
		
	reg [9:0] counter;//is this initalized to zero by default???
	
	
  reg  C_1, C_2; //temporary variables
  
  wire [9:0]dutyCylce;
  
  DutyCylceConverter dcc(SW[11:4], clk, frequency, dutyCylce);
  
	
	
	always @(posedge clk, negedge KEY[0])
	begin
	   if (!KEY[0])
			begin
				C_1<=0;
				C_2<=0;
				counter = 10'b0;
			end 
		else if(!KEY[1]) 
			begin
				C_1<=0;
				C_2<=0;
			end 
		else 
			begin 
					if (counter < maxcount) // at every 0.5 seconds, activate 
						counter = counter + 1;
					else 
						counter = 0;
			//the duty cycle adjusted based on the switch input
			if (counter > dutyCylce) 
				begin
				   C_2<=1'b0;
					C_1 <= 1'b1;
				end 
			else	
				begin
					C_1<=1'b0;
					C_2 <= 1'b1;
				end

			end

	end
	
wire bit_value1, bit_value2;



shiftn forDT1(SW[14:12], C_1, clk, bit_value1);

shiftn forDT2(SW[17:15], C_2, clk, bit_value2);


		assign GPIO_0[32] = bit_value1 & C_1;
		assign GPIO_0[34] = bit_value2 & C_2;

	
endmodule 

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
output reg bit_value;
always @(posedge Clock)
		begin
			for (k = 0; k < 4; k = k+1)
				Q[k] <= Q[k+1];
			
			Q[n] <= w;
			bit_value =Q[0];
		end

endmodule


module DutyCylceConverter (SW_value, clk2, frequency, duty);
input clk2;
input [3:0] frequency;
input [7:0] SW_value;
output reg [9:0] duty;

always@ (*)
	case (frequency)
	
		4'b0000: duty = SW_value*8'b0000_0100;//1000 and freq = 50khz
		4'b0001: duty = SW_value*8'b0000_0100;//769 and freq = 65khz
		4'b0010: duty = SW_value*8'b0000_0100;//625 and freq = 80khz
		4'b0011: duty = SW_value*8'b0000_0100;//526 and freq = 95khz
		4'b0100: duty = SW_value*8'b0000_0010;//455 and freq = 110khz
		4'b0101: duty = SW_value*8'b0000_0010;//400 and freq = 125khz
		4'b0110: duty = SW_value*8'b0000_0010;//357 and freq = 140khz
		4'b0111: duty = SW_value*8'b0000_0010;//322 and freq = 155khz
		4'b1000: duty = SW_value*8'b0000_0010;//294 and freq = 170khz
		4'b1001: duty = SW_value*8'b0000_0010; //270 and freq = 185khz
		4'b1010: duty = SW_value*8'b0000_0001; //250 and freq = 200khz, whe should not go beyond 250!!!!!!!!!
		
		default: duty = SW_value*8'b0000_0100;
	endcase	
endmodule 
