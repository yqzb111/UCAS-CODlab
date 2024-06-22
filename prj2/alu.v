`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	// TODO: Please add your logic design here
	wire AND,OR,ADD,SUB,SLT,XOR,NOR,SLTU;		//设置选择器选择的路线
	wire [`DATA_WIDTH - 1:0] re_ADDSUB;	//加法器/减法器的结果
	wire [`DATA_WIDTH - 1:0] re_SLT;	//比较器的结果
	wire re_CarryOut;	//加法进位和减法进位
	wire [`DATA_WIDTH - 1:0] re_SLUT;		//无符号比较器结果

	//根据ALUop，选择合适的模式
	assign AND = (ALUop == 3'b000);			
	assign OR = (ALUop == 3'b001);
	assign ADD = (ALUop == 3'b010);
	assign SUB = (ALUop == 3'b110);
	assign SLT = (ALUop == 3'b111);
	assign XOR = (ALUop == 3'b100);
	assign NOR = (ALUop == 3'b101);
	assign SLTU = (ALUop == 3'b011);

	//求得加法/减法的结果以及进位
	assign {re_CarryOut,re_ADDSUB} = {1'b0, A} + {1'b0, ({32{(SUB | SLT | SLTU)}} & ~B) | 
					 ({32{~(SUB | SLT | SLTU)}} & B)} + {31'b0, (ALUop[2] | ALUop[0])};		
	//求得比较的结果：若最高位的值与符号位相同，则A>=B，否则A<B	
	assign re_SLT = {31'b0,(re_ADDSUB[31] ^ Overflow)};	
	//无符号数比较：若小于，则置1，否则置0
	assign re_SLUT = {31'b0,CarryOut};
	/*判断四个输出的结果
	**CarryOut的结果，若为加法，则等于加法器的进位；若为减法，则若B非0，进位与减法器（实际为补码的加法）进位相反；若为其他，则取0
	**Overflow的结果，若为有符号加法，则A，B最高位相同且与结果相反，则溢出；有符号减法，转化为补码的加法，同样相反溢出；其他则取0
	**Result的结果，根据选择的模式，通过相应的运算得出
	**Zero的结果，若Result全为0，则Zero为1，否则为0
	*/
	assign CarryOut = (ADD & re_CarryOut) | ((SUB | SLTU) & (B!=32'b0) & ~re_CarryOut);
	assign Overflow = (ADD & (((A[31] & B[31]) & ~re_ADDSUB[31]) | 
			  ((~A[31] & ~B[31]) & re_ADDSUB[31]))) |
			  (((SUB | SLT) & (B != 32'h80000000)) & (((A[31] & ~B[31]) & ~re_ADDSUB[31]) | 
			  ((~A[31] & B[31]) & re_ADDSUB[31]))) |
			  (((SUB | SLT) & (B == 32'h80000000)) & ~A[31]);
	assign Result = (({32{ADD}} | {32{SUB}}) & re_ADDSUB) | ({32{AND}} & (A & B)) | 
			({32{OR}} & (A | B)) | ({32{SLT}} & re_SLT) | ({32{XOR}} & (A ^ B)) |
			({32{NOR}} & ~(A | B)) | ({32{SLTU}} & (re_SLUT));
	assign Zero = ((Result == 32'b0) & 1) | (!(Result == 32'b0) & 0);
endmodule
