`timescale 10ns / 1ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module simple_cpu(
	input             clk,			//处理器工作时钟
	input             rst,			//与处理器工作时钟同步的高电平复位信号

	output reg [31:0] PC,			//程序计数器, 复位后初值为32’d0(时序电路),跳转目标地址使用一个单独加法器计算
	input  [31:0]     Instruction,		//从内存（Memory）中读取至处理器的指令

	output [31:0]     Address,		//数据访存指令使用的内存地址
	output            MemWrite,		//内存访问的写使能信号（高电平有效）
	output [31:0]     Write_data,		//内存写操作数据
	output [ 3:0]     Write_strb,		//内存写操作字节有效信号（支持32/16/8-bit内存写）	
						//Write_strb[i] == 1表示
						//Write_data[8×(i+1)-1 : 8×i]位会被写入内存的对应地址

	input  [31:0]     Read_data,		//从内存中读取的数据
	output            MemRead		//内存访问的读使能信号（高电平有效）
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	// TODO: PLEASE ADD YOUR CODE BELOW
	wire [`DATA_WIDTH - 1:0]	RF_rdata1;
	wire [`DATA_WIDTH - 1:0]	RF_rdata2;

	wire [`ADDR_WIDTH:0]		opcode;
	wire [`ADDR_WIDTH - 1:0]	rs;
	wire [`ADDR_WIDTH - 1:0]	rt;
	wire [`ADDR_WIDTH - 1:0]	rd;
	wire [`ADDR_WIDTH - 1:0]	sa;
	wire [`ADDR_WIDTH:0]		func;

	wire [`DATA_WIDTH - 1:0]	Zero_extend;			//0扩展
	wire [`DATA_WIDTH - 1:0]	Arithmetic_extend;		//符号位扩展
	wire [`DATA_WIDTH - 1:0]	Shift_arithmetic_extend;	//移位扩展

	wire [2:0]			ALUop;				//ALU控制信号，传输运算指令
	wire [`DATA_WIDTH - 1:0]	ALU_result;			//计算结果
	wire [`DATA_WIDTH - 1:0]	ALU_A;				//计算输入1
	wire [`DATA_WIDTH - 1:0]	ALU_B;				//计算输入2
	wire				Zero;				//零判断

	wire [1:0]			Shiftop;			//移位选择信号
	wire [`DATA_WIDTH - 1:0]	Shift_result;			//移位操作结果
	wire [`ADDR_WIDTH - 1:0]	Shifter_B;			//移位输入

	wire				Jump;				//跳转
	wire [`DATA_WIDTH - 1:0]	Jump_addr;			//跳转地址

	wire				Branch;				//跳转
	wire [`DATA_WIDTH - 1:0]	Branch_addr;			//跳转地址

	wire [`DATA_WIDTH - 1:0]	PC_next;			//PC值

	wire [`DATA_WIDTH - 1:0]	Read_reg_data;			//寄存器读数据
	wire [7:0]			Byte_data;			//长度1byte的读数据
	wire [15:0]			Half_data;			//长度2byte的读数据
	wire [`DATA_WIDTH - 1:0]	LWL_data;			//I-Type LWL操作的读数据
	wire [`DATA_WIDTH - 1:0]	LWR_data;			//I-Type LWR操作的读数据

	//R-Type指令码
	assign	opcode = Instruction[31:26];				//opcode占Instruction的高6位
	assign	rs     = Instruction[25:21];				//rs为Instruction的21-25位
	assign	rt     = Instruction[20:16];				//rt为Instruction的16-20位
	assign	rd     = Instruction[15:11];				//rd为Instruction的11-15位
	assign	sa     = Instruction[10:6];				//sa为Instruction的6-10位
	assign	func   = Instruction[5:0];				//func为Instruction的低6位

	//操作数扩展
	assign	Zero_extend         	= {16'b0, Instruction[15:0]};
	assign	Arithmetic_extend	= Instruction[15] ? {{16{1'b1}},Instruction[15:0]} : {{16{1'b0}},Instruction[15:0]};
	assign	Shift_arithmetic_extend = Instruction[15] ? {{14{1'b1}},Instruction[15:0], 2'b0} : {{14{1'b0}},Instruction[15:0], 2'b0};

	//ALU操作
	assign ALUop = (opcode == 6'b0 && func[3:2] == 2'b0) ? {func[1],2'b10}						//R-Type ADD/SUB操作
		     : (opcode == 6'b0 && func[3:2] == 2'b01) ? {func[1],1'b0,func[0]}	 				//R-Type AND/OR/XOR/NOR操作
		     : (opcode == 6'b0 && func[3:2] == 2'b10) ? {~func[0],2'b11}					//R-Type SLT/SLTU操作
	   	     : (opcode[5:3] == 3'b001 && opcode[2:1] == 2'b0) ? 3'b010						//I-Type ADDIU操作
		     : (opcode[5:3] == 3'b001 && opcode[2] == 1'b1 && opcode[1:0] != 2'b11) ? {opcode[1],1'b0,opcode[0]}//I-Type ANDI/XORI/ORI操作
		     : (opcode[5:3] == 3'b001 && opcode[2:1] == 2'b01) ? {~opcode[0],2'b11}				//I-Type SLTI/SLTIU操作
		     : (opcode == 6'b000001 || opcode[5:1] == 5'b00011) ? 3'b111					//REGIMM BLTZ/BGEZ操作；I-Type BLEZ/BGTZ操作
		     : (opcode[5:1] == 5'b00010) ? 3'b110								//I-Type BEQ/BNE操作
		     : (opcode[5] == 1'b1) ? 3'b010										//I-Type 内存读写指令
		     : 3'bX;												//其他操作
	assign ALU_A = (opcode[5:1] == 5'b00011) ? 32'b0 : RF_rdata1;							//若为BLEZ/BGTZ操作，则输入为0，否则正常输入
	assign ALU_B = (opcode[5:1] == 5'b00011) ? RF_rdata1								//I-Type BLEZ/BGTZ操作
		     : (opcode == 6'b000001) ? 32'b0									//REGIMM BLTZ/BGEZ操作
		     : (opcode[5:3] == 3'b001 && opcode != 6'b001001) ? Zero_extend					//除ADDIU以外的I-Type计算指令
		     : (opcode == 6'b001001 || opcode[5] == 1) ? Arithmetic_extend					//ADDIU/I-Type内存读写指令
		     : RF_rdata2;											//其他操作

	//Shifter操作
	assign Shiftop = (opcode == 6'b0 && func[5:3] == 3'b0) ? func[1:0] : 2'bX;					//R-Type移位指令
	assign Shifter_B = (func[2] == 0) ? sa : RF_rdata1[4:0];							//若为SLL/SRA/SRL，则移位位数由sa决定；若为SLLV/SRAV/SRLV，则由rs低五位决定

	//Jump操作
	assign Jump = ((opcode == 6'b0 && func[5:3] == 3'b001 && func[1] == 1'b0) || opcode[5:1] == 5'b00001) ? 1 : 0;	//R-Type JR/JALR操作；J-Type J/JAL操作
	assign Jump_addr = (opcode == 6'b0 && func[5:3] == 3'b001 && func[1] == 1'b0) ? RF_rdata1			//若为R-Type JR/JALR操作，则跳转地址为rs的值
			 : {PC_next[31:28],Instruction[25:0],2'b0};							//若为J-Type J/JAL操作，则跳转地址为新地址低28位为instr_index左移两位，
															//高4位为跳转指令后指令地址的高4位
	
	//Branch操作
	assign Branch = ((opcode == 6'b000001 && (rt[0] ^ ALU_result[0]))						//REGIMM BLTZ/BGEZ指令
		     || (opcode[5:2] == 4'b0001 && (opcode[0] ^ Zero)))							//I-Type BLEZ/BGTZ/BEQ/BNE指令
		      ? 1 : 0;
	assign Branch_addr = Shift_arithmetic_extend + PC_next;								//跳转地址为{14'b(offset[15]),(offset<<2),00} + (PC+4)

	//Read_reg_data实现
	assign Read_reg_data = (opcode == 6'b100000) ? (Byte_data[7] ? {{24{1'b1}},Byte_data} : {{24{1'b0}},Byte_data})	//I-Type LB操作
			     : (opcode == 6'b100001) ? (Half_data[15] ? {{16{1'b1}},Half_data} : {{16{1'b0}},Half_data})//I-Type LH操作
			     : (opcode == 6'b100100) ? {{24{1'b0}},Byte_data}						//I-Type LBU操作
			     : (opcode == 6'b100101) ? {{16{1'b0}},Half_data}						//I-Type LHU操作
			     : (opcode == 6'b100010) ? LWL_data								//I-Type LWL操作
			     : (opcode == 6'b100110) ? LWR_data								//I-Type LWR操作
			     : Read_data;										//I-Type LW操作
	assign Byte_data = (ALU_result[1:0] == 2'b11) ? Read_data[31:24]
			 : (ALU_result[1:0] == 2'b10) ? Read_data[23:16]
			 : (ALU_result[1:0] == 2'b01) ? Read_data[15:8]
			 : Read_data[7:0];
	assign Half_data = (ALU_result[1:0] == 2'b00) ? Read_data[15:0] : Read_data[31:16];
	assign LWL_data	= (ALU_result[1:0] == 2'b11) ? Read_data[31:0]
			: (ALU_result[1:0] == 2'b10) ? {Read_data[23:0], RF_rdata2[7:0]}
			: (ALU_result[1:0] == 2'b01) ? {Read_data[15:0], RF_rdata2[15:0]}
			: {Read_data[7:0], RF_rdata2[23:0]};
	assign LWR_data	= (ALU_result[1:0] == 2'b11) ? {RF_rdata2[31:8],Read_data[31:24]}
			: (ALU_result[1:0] == 2'b10) ? {RF_rdata2[31:16],Read_data[31:16]}
			: (ALU_result[1:0] == 2'b01) ? {RF_rdata2[31:24],Read_data[31:8]}
			: Read_data[31:0];

	//寄存器读写操作
	assign RF_wen = (opcode == 6'b000001 || opcode[5:2] == 4'b0001 || (opcode[5] && opcode[3]) 
		     || (opcode == 6'b0 && func == 6'b001000)) ? 0							//REGIMM指令/I-Type分支指令/I-Type内存写指令/R-Type JR指令
		      : (opcode[5:1] == 5'b00001) ? opcode[0]								//J-Type指令
		      : (opcode == 6'b0 && func[5:1] == 5'b00101) ? func[0] ^ (RF_rdata2 == 32'b0)			//R-Type MOVZ/MOVN指令
	              : 1;
	assign RF_waddr = ((opcode == 6'b0 && func == 6'b001001 && rd == 5'b0) || opcode[5:1] == 5'b00001) ? 31		//J-Type指令/R-Type JALR指令(rd为0时)
			: (opcode[5:3] == 3'b001 || opcode[5:3] == 3'b100) ? rt						//I-Type计算指令/I-Type内存读指令
		     	: rd;
	assign RF_wdata = (opcode == 6'b0 && ((func == 6'b001010 && RF_rdata2 == 32'b0) 
		       || (func == 6'b001011 && RF_rdata2 != 32'b0))) ? RF_rdata1					//R-Type MOVZ/MOVN指令
		     	: (opcode == 6'b001111) ? {Instruction[15:0],16'b0}						//I-Type LUI指令
			: ((opcode == 6'b0 && func[5] == 1'b1) || opcode[5:3] == 3'b001) ? ALU_result			//R-Type计算指令/I-Type计算指令
			//注意上两行的条件判断有重叠的地方，顺序交换会导致错误，必须先把条件强的放前面
		     	: ((opcode == 6'b0 && func[5:1] == 5'b00100) || opcode[5:1] == 5'b00001) ? PC+8			//R-Type跳转指令/J-Type指令
			: (opcode == 6'b0 && func[5:3] == 3'b0) ? Shift_result						//R-Type移位指令
		     	: (opcode[5:3] == 3'b100) ? Read_reg_data							//I-Type 内存读指令
		     	: 32'bX;									

	//内存读写操作
	assign MemRead = (opcode[5:3] == 3'b100) ? 1 : 0;								//I-Type内存读指令
	assign MemWrite = (opcode[5:3] == 3'b101) ? 1 : 0;								//I-Type内存写指令
	assign Address = {ALU_result[31:2],2'b0};									//内存地址,低二位为0
	assign Write_strb = (opcode[1:0] == 2'b0) ? (4'b1000 >> (~ALU_result[1:0]))					//I-Type SB
			  : (opcode[1:0] == 2'b01) ? {{2{ALU_result[1]}},{2{~ALU_result[1]}}}				//I-Type SH
			  : (opcode[1:0] == 2'b11) ? 4'b1111								//I-Type SW
			  : (opcode[2:0] == 3'b010) ? {ALU_result[1] & ALU_result[0],			
			    	ALU_result[1],ALU_result[1] | ALU_result[0],1'b1}					//I-Type SWL
			  : {1'b1,~ALU_result[1] | ~ALU_result[0],~ALU_result[1],			
			     	~ALU_result[1] & ~ALU_result[0]};							//I-Type SWR
	assign Write_data = (opcode == 6'b101000) ? (Write_strb[3] ? {RF_rdata2[7:0],24'b0}
				: Write_strb[2] ? {8'b0,RF_rdata2[7:0],16'b0} 
				: Write_strb[1] ? {16'b0,RF_rdata2[7:0],8'b0}
				: {24'b0,RF_rdata2[7:0]})								//I-Type SB操作
			  : (opcode == 6'b101001) ? ((Write_strb[3] && Write_strb[2]) ? {RF_rdata2[15:0],16'b0}
				: {16'b0,RF_rdata2[15:0]})								//I-Type SH操作
			  : (opcode == 6'b101010) ? (Write_strb[3] ? RF_rdata2
				: Write_strb[2] ? {8'b0,RF_rdata2[31:8]} 
				: Write_strb[1] ? {16'b0,RF_rdata2[31:16]}
				: {24'b0,RF_rdata2[31:24]})								//I-Type SWL操作				
			  : (opcode == 6'b101110) ? (Write_strb[0] ? RF_rdata2	
				: Write_strb[1] ? {RF_rdata2[23:0],8'b0} 
				: Write_strb[2] ? {RF_rdata2[15:0],16'b0}
				: {RF_rdata2[7:0],24'b0})								//I-Type SWR操作
			  : RF_rdata2;											//I-Type SW操作
	
	//实例化
	reg_file reg_file_module(
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(rs),
		.raddr2(rt),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(RF_rdata1),
		.rdata2(RF_rdata2)
	);
	alu alu_module(
		.A(ALU_A),
		.B(ALU_B),
		.ALUop(ALUop),
		.Result(ALU_result),
		.Overflow(),
		.CarryOut(),
		.Zero(Zero)
	);
	shifter shifter_module(
		.A(RF_rdata2),
		.B(Shifter_B),
		.Shiftop(Shiftop),
		.Result(Shift_result)
	);

	//PC_next
	assign PC_next 	= PC + 4;

	//时序逻辑电路
	always@(posedge clk) begin
		if (rst) begin 
			PC <= 32'b0;							//rst同步的高电平复位信号
		end
		else begin
			PC <= Jump ? Jump_addr : (Branch ? Branch_addr : PC_next);	//根据操作不同，选择相应的下一条指令的PC地址
		end
	end

endmodule
