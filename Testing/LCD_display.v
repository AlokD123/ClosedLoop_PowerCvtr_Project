//To display on LCD

`include "LCD.v"

module main(
  input CLOCK_50,    //    50 MHz clock
  input [3:0] KEY,      //    Pushbutton[3:0]
  output [6:0]    HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,HEX6,HEX7,  // Seven Segment Digits
  input [3:0]	VinH,VinL,VoutH,VoutL,IoutH,IoutL,TempH,TempL;
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
assign LEDR = SW[1:0];

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