// Creates PWM signal with two adjustable dead times.
// Note: SECOND state is C1=1,C2=0 ..... FIRST state is C1=0,C2=1. FIRST STATE COMES BEFORE SECOND STATE
// Note: GPIO_O[0] is on left, [1] is on right

module main (SW, KEY, clk, GPIO_0);
input [17:0] SW;
input [1:0] KEY;			//KEY[0]=resetn and KEY[1]= EN
input clk;
reg C1, C2;
output [1:0]GPIO_0;
reg [9:0] maxcount;

always@ (*)
	case (SW[3:0])
	
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
		
	reg [9:0] counter;//is this initalized to zero by default???
	
	
  reg  C_1, C_2; //temporary variables
  wire C_R1, C_R2;
  wire [9:0] counterWire;		
  wire [9:0] dutyCycle;
  
  dutyCycleConverter dcc(SW[11:4], clk, frequency, dutyCycle);		// Convert SW[11:4]-input and frequency setting to duty cycle COUNT
  
	// Turn off Main Counter (counter) until dead time count (SW[14:12]-input) reached. 			Output is desired signal C1 for STATE 2!!!(C_R1)
	DeadTime1 dt1(dutyCycle, counter, SW[14:12], clk, C_R1);	
	// Same as above, but for C2. 																					Output is desired signal C2 for STATE 1!!! (C_R2)
	DeadTime2 dt2(dutyCycle,counter, SW[17:15], clk, C_R2);
	
	
	always @(posedge clk, negedge KEY[0])
	begin
	   if (!KEY[0])				// If Reset
			begin
				C_1<=0;
				C_2<=0;
				counter<= 10'b0000000000;
			end 
		else if(!KEY[1])			// If Disabled
			begin
				C_1<=0;
				C_2<=0;
			end
		else 
			begin 
				if (counter < maxcount) 		// count up every 20ns until reaching maxCount
					counter <= counter + 10'b0000000001;
				else 
					counter<= 10'b0000000000; // reset upon reaching max (counter==maxcount)
			
				//the duty cycle adjusted based on the switch input
				if (counter > (dutyCycle+10'b0000000001)) 	//For SECOND state....
					begin
						C_1 = C_R1; //1'b1;//
						C_2=1'b0;
					end
				else if (counter < (dutyCycle-10'b0000000001)) //For FIRST state....
					begin
						C_1=1'b0;
						C_2 = C_R2; //1'b1;//
					end
				else								
					begin
						C_1=1'b0;
						C_2=1'b0;
					end 
			end
		
		
	end
		
		assign GPIO_0[0] =C_1;
		assign GPIO_0[1] =C_2;
//always @(*) 
	
endmodule 


// Dead time generator FOR SECOND STATE (C1)...  Controlled by SW[14:12]
module DeadTime1(DutyCycle, mainCounter, Dt1, clk, out1);
input [9:0]mainCounter;
input [9:0]DutyCycle;
input [2:0]Dt1;
input clk;
output reg out1;
parameter DT_MULTIPLIER_1 = 2'b10; //MAXIMUM VALUE OF 16

reg [5:0] dt1Count=0;//is this initalized to zero by default???

	always @(posedge clk)
	   if (mainCounter > (DutyCycle+10'b0000000001))		// IF IN SECOND STATE...
			begin
				if(dt1Count < Dt1*DT_MULTIPLIER_1)					// If dead time not yet complete...
					begin
						dt1Count <= dt1Count + 3'b001;	// Count to complete dead time
						out1 <= 1'b0;							// No output during dead time
					end
				else										// If dead time complete...
					begin
						//dt1Count <= 3'b000;				// Reset dead time count
						out1 <= 1'b1;					// Turn ON C1
					end
			end
		else
			begin
				dt1Count <= 3'b000;
				out1 <= 1'b0;
			end
//		else
//			out1 <= 1'b0;


//		else if (mainCounter == DutyCycle)// For first cycle, initialize dead time count = 0 and NO OUTPUT
//			begin
//				out1 <= 1'b0;
//				dt1Count <= 3'b000;
//			end
//		else										// FOR STATE 1....
//			begin
//				dt1Count <= 3'b000;					// Set count=0
//				out1 <= 1'b0;					// No output C1
//			end
endmodule


// Dead time generator FOR FIRST STATE (C2)
module DeadTime2(DutyCycle,mainCounter, Dt2, clk, out2);
input [9:0]DutyCycle;
input [9:0]mainCounter;
input [2:0]Dt2;
input clk;
output reg out2;
reg [5:0] dt2Count=0;//---is this initalized to zero by default???
parameter DT_MULTIPLIER_2 = 2'b10; //MAXIMUM VALUE OF 16
	
	always @(posedge clk)
	   if ( (mainCounter < DutyCycle - 10'b0000000001) ) // IF IN STATE 1
			begin
				if(dt2Count < Dt2*DT_MULTIPLIER_2)
					begin
						dt2Count <= dt2Count + 3'b001;
						out2 <= 1'b0; 
					end
				else
					begin
						//dt2Count <= 3'b000;				// Reset dead time count
						out2 <= 1'b1;					// Turn ON C2
					end
			end
		else
			begin
				dt2Count <= 3'b000;
				out2 <= 1'b0;
			end	
//		else
//			out2 <= 1'b0;


//		else if (mainCounter == 0)
//			begin
//					out2 <= 1'b0;
//					dt2Count <= 3'b000;
//			end
//		else 											// FOR STATE 2....
//			begin
//					out2 <= 1'b0;
//					dt2Count <= 3'b000;
//			end

		/*else if ((dt2Count == Dt2)&& (mainCounter != 0))
			begin
					out2 <= 1'b1;
					dt2Count <= 0;
			end
		else
			begin
				out2 <= 1'b1;
			end
		*/
endmodule


module dutyCycleConverter (SW_value, clk2, frequency, duty);
input clk2;
input [3:0] frequency;
input [7:0] SW_value;
output reg [9:0] duty;

always@ (posedge clk2)
	case (frequency)
	
		4'b0000: duty = SW_value*8'b00000100;//1000 and freq = 50khz, Don't go beyond ~1000!!!!!!!!!
		4'b0001: duty = SW_value*8'b00000100;//769 and freq = 65khz
		4'b0010: duty = SW_value*8'b00000100;//625 and freq = 80khz
		4'b0011: duty = SW_value*8'b00000100;//526 and freq = 95khz
		4'b0100: duty = SW_value*8'b00000010;//455 and freq = 110khz
		4'b0101: duty = SW_value*8'b00000010;//400 and freq = 125khz
		4'b0110: duty = SW_value*8'b00000010;//357 and freq = 140khz, Don't go beyond ~160!!!!!!!!!
		4'b0111: duty = SW_value*8'b00000010;//322 and freq = 155khz
		4'b1000: duty = SW_value*8'b00000010;//294 and freq = 170khz
		4'b1001: duty = SW_value*8'b00000010; //270 and freq = 185khz
		4'b1010: duty = SW_value; //250 and freq = 200khz, Don't go beyond ~64!!!!!!!!!
		
		default: duty = SW_value*8'b00000100;
	endcase	
endmodule 