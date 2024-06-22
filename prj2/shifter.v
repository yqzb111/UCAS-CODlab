`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module shifter (
	input  [`DATA_WIDTH - 1:0] A,
	input  [              4:0] B,
	input  [              1:0] Shiftop,
	output [`DATA_WIDTH - 1:0] Result
);
	// TODO: Please add your logic code here
	wire LS,ARS,LRS;				//左移，算数右移，逻辑右移
	wire signed [`DATA_WIDTH - 1:0] arithmetic_shift;	//算数移位
	assign arithmetic_shift[31:0] = A;
	assign LS = (Shiftop == 2'b00);
	assign ARS = (Shiftop == 2'b11);
	assign LRS = (Shiftop == 2'b10);
	assign Result = ({32{LS}} & A << B) |
		      	({32{ARS}} & $signed(arithmetic_shift >>> B)) |
		      	({32{LRS}} & A >> B);

endmodule
