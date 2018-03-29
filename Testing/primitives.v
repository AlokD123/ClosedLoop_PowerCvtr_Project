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


