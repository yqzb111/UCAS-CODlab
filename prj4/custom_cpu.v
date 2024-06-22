`timescale 10ns / 1ns

//宏定义两个常见长度
`define DATA_WIDTH 32
`define ADDR_WIDTH 5

//宏定义各类指令类型
`define R 3'b000
`define I 3'b001
`define S 3'b010
`define B 3'b011
`define U 3'b100
`define J 3'b101

//宏定义ALU的各种操作
`define AND	 3'b000
`define OR	 3'b001
`define ADD	 3'b010
`define SUB	 3'b110
`define SLT	 3'b111
`define XOR	 3'b100
`define SLTU 3'b011

module custom_cpu(
	input         clk,
	input         rst,

	//Valid:高电平表示发送方发出的请求或应答内容有效
	//Ready:高电平表示接收方可以接收发送方的请求或应答

	//Instruction request channel
	output reg [31:0] PC,			//程序计数器
	output        Inst_Req_Valid,	
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,		//从内存（Memory）中读取至处理器的指令
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,			//数据访存指令使用的内存地址
	output        MemWrite,			//内存访问的写使能信号（高电平有效）
	output [31:0] Write_data,		//内存写操作数据
	output [ 3:0] Write_strb,		//内存写操作字节有效信号（支持32/16/8-bit内存写）	
									//Write_strb[i] == 1表示
									//Write_data[8×(i+1)-1 : 8×i]位会被写入内存的对应地址
	output        MemRead,			//内存访问的读使能信号（高电平有效）
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,		//从内存中读取的数据
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,				//实验5中使用

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
// TODO: Please add your custom CPU code here

	//定义各个状态的独热码
	localparam 	INIT = 9'b000000001,
				IF = 9'b000000010,
				IW = 9'b000000100,
				ID = 9'b000001000,
				EX = 9'b000010000,
				ST = 9'b000100000,
				WB = 9'b001000000,
				LD = 9'b010000000,
				RDW = 9'b100000000;

	reg [8:0]	current_state;			//当前状态
	reg [8:0]	next_state;				//下一状态
	reg [31:0]	PC_current;				//当前PC值
	reg [31:0]	Instruction_current;	//当前指令
	reg [31:0]	Read_data_current;		//当前读数据
	reg [31:0]	cycle_cnt;				//周期计数器
	reg [31:0]	inst_cnt;				//指令数计数器

	wire						RF_wen;
	wire [`ADDR_WIDTH - 1:0]	RF_waddr;
	wire [`DATA_WIDTH - 1:0]	RF_wdata;
	wire [`DATA_WIDTH - 1:0]	RF_rdata1;
	wire [`DATA_WIDTH - 1:0]	RF_rdata2;

	//定义指令码中各段代表变量的名称
	wire [6:0]					opcode;
	wire [`ADDR_WIDTH - 1:0]	rd;
	wire [2:0]					funct3;
	wire [`ADDR_WIDTH - 1:0]	rs1;
	wire [`ADDR_WIDTH - 1:0]	rs2;
	wire [6:0]					funct7;
	
	//各类操作的立即数
	wire [`DATA_WIDTH - 1:0]	I_extend;
	wire [`DATA_WIDTH - 1:0]	S_extend;
	wire [`DATA_WIDTH - 1:0]	B_extend;
	wire [`DATA_WIDTH - 1:0]	U_extend;
	wire [`DATA_WIDTH - 1:0]	J_extend;

	wire [2:0]					ALUop;				//ALU控制信号，传输运算指令
	wire [`DATA_WIDTH - 1:0]	ALU_result;			//计算结果
	wire [`DATA_WIDTH - 1:0]	ALU_A;				//计算输入1
	wire [`DATA_WIDTH - 1:0]	ALU_B;				//计算输入2
	wire						Zero;				//零判断

	wire [1:0]					Shiftop;			//移位选择信号
	wire [`DATA_WIDTH - 1:0]	Shift_result;		//移位操作结果
	wire [`ADDR_WIDTH - 1:0]	Shifter_B;			//移位输入

	wire						Jump;				//跳转
	wire [`DATA_WIDTH - 1:0]	JALR_addr;			//JALR跳转地址			
	wire [`DATA_WIDTH - 1:0]	Jump_addr;			//跳转地址

	wire						Branch;				//跳转
	wire [`DATA_WIDTH - 1:0]	Branch_addr;		//跳转地址

	wire [`DATA_WIDTH - 1:0]	PC_next;			//PC值

	wire [`DATA_WIDTH - 1:0]	Read_reg_data;		//寄存器读数据
	wire [7:0]					Byte_data;			//长度1byte的读数据
	wire [15:0]					Half_data;			//长度2byte的读数据

	wire [2:0]					ins;				//指令类型编码

	//指令码
	assign	opcode  = Instruction_current[6:0];				//opcode占Instruction_current的低7位
	assign	rd      = Instruction_current[11:7];			//rd为Instruction_current的7-11位
	assign	funct3  = Instruction_current[14:12];			//funct3为Instruction_current的12-14位
	assign	rs1     = Instruction_current[19:15];			//rs1为Instruction_current的15-19位
	assign 	rs2	    = Instruction_current[24:20];			//rs2为Instruction_current的24-20位
	assign	funct7  = Instruction_current[31:25];			//funct7为Instruction_current的高7位

	//各类指令编码
	assign ins = (opcode == 7'b0110011) ? `R
			   : (opcode == 7'b0100011) ? `S
			   : (opcode == 7'b1100011) ? `B
			   : (~opcode[6] && opcode[2]) ? `U
			   : (opcode == 7'b1101111) ? `J
			   : `I;

	//各类操作数扩展
	assign	I_extend	= {{21{Instruction_current[31]}},Instruction_current[30:20]};
	assign	S_extend	= {{21{Instruction_current[31]}},Instruction_current[30:25],Instruction_current[11:7]};
	assign	B_extend	= {{20{Instruction_current[31]}},Instruction_current[7],Instruction_current[30:25],Instruction_current[11:8],1'b0};
	assign	U_extend	= {Instruction_current[31:12],12'b0};
	assign	J_extend	= {{12{Instruction_current[31]}},Instruction_current[19:12],Instruction_current[20],Instruction_current[30:21],1'b0};

	//ALU操作
	assign ALUop = ({3{(ins == `I || ins == `R) && funct3 == 3'b111}} & `AND) |		//AND/ANDI操作
					({3{(ins == `I || ins == `R) && funct3 == 3'b110}} & `OR) |		//OR/ORI操作
					({3{(ins == `R && funct3 == 3'b0 && funct7[5] == 1'b0) ||		//ADD操作
					(ins == `I && funct3 == 3'b0) ||								//ADDI/LB/JALR操作
					(ins == `S) || (opcode == 7'b0000011) ||						//Store/Load操作
					(ins == `U && opcode[6:4] == 3'b001) || (ins == `J)}} & `ADD) |	//AUIPC/JAL操作
					({3{(ins == `R && funct3 == 3'b0 && funct7[5]) ||				//SUB操作
					(ins == `B && funct3[2:1] == 2'b0)}} & `SUB) |					//BEQ/BNE操作
					({3{(ins == `R && funct3 == 3'b010) || (ins == `I && funct3 == 3'b010 && opcode[4]) ||			//SLT/SLTI操作
					(ins == `B && funct3[2:1] == 2'b10)}} & `SLT) |					//BLT/BGE操作
					({3{(ins == `R && funct3 == 3'b100) || (ins == `I && funct3 == 3'b100 && opcode[4])}} & `XOR) |	//XOR/XORI操作
					({3{(ins == `R && funct3 == 3'b011) || (ins == `I && funct3 == 3'b011 && opcode[4]) ||			//SLTU/SLTIU操作
					(ins == `B && funct3[2:1] == 2'b11)}} & `SLTU);					//BLTU/BGEU操作


	assign ALU_A = (ins == `J || (ins == `U && opcode[5] == 1'b0)) ? PC_current 
				: RF_rdata1;									//若为JAL/AUIPC操作，则输入为PC_current，否则正常输入
	assign ALU_B = (ins == `R || ins == `B) ? RF_rdata2			//R-Type/B-Type操作
		     	: (ins == `I) ? I_extend						//I-Type操作
		     	: (ins == `S) ? S_extend						//S-Type操作
		     	: (ins == `U && opcode[5] == 1'b0) ? U_extend	//AUIPC操作
			 	: (ins == `J) ? J_extend						//J-Type操作
		     	: 32'b0;										//其他操作

	//Shifter操作
	assign Shiftop = ((ins == `R || ins == `I) && funct3 == 3'b001 && opcode[4]) ? 2'b0				//SLL/SLLI操作
				   : ((ins == `R || ins == `I) && funct3 == 3'b101 && funct7[5] == 1'b0) ? 2'b10	//SRL/SRLI操作
				   : ((ins == `R || ins == `I) && funct3 == 3'b101 && funct7[5] == 1'b1) ? 2'b11	//SRA/SRAI操作
				   : 2'b01;
	assign Shifter_B = (ins == `R) ? RF_rdata2[4:0] : I_extend[4:0];					//若为SLL/SRA/SRL，则移位位数由rs2决定；若为SLLI/SRAI/SRLI，则由imm低五位决定

	//Jump操作
	assign Jump = (ins == `J || (ins == `I && opcode == 7'b1100111));					//J-Type操作/I-Type JALR操作
	assign JALR_addr = RF_rdata1+I_extend;
	assign Jump_addr = (ins == `I && opcode[6]) ? {JALR_addr[31:1],1'b0}				//若为I-Type JALR操作，则JALR操作跳转地址为rs1+imm的值
			 		: (PC_current + J_extend);											//若为J-Type JAL操作，则跳转地址为PC_current+imm的值
	
	//Branch操作
	assign Branch = (ins == `B && ((Zero && funct3 == 3'b0) || (!Zero && funct3 == 3'b001) ||
				    (ALU_result && (funct3 == 3'b110 || funct3 == 3'b100)) || 
					(!ALU_result && (funct3 == 3'b101 || funct3 == 3'b111))));			//B-Type指令
	assign Branch_addr = B_extend + PC_current;											//跳转地址为PC_current+imm的值

	//Read_reg_data实现
	assign Read_reg_data = (ins == `I && funct3 == 3'b000 && opcode[5:3] == 3'b0) ? (Byte_data[7] ? {{24{1'b1}},Byte_data} : {{24{1'b0}},Byte_data})	//I-Type LB操作
			     		 : (ins == `I && funct3 == 3'b001 && opcode[5:3] == 3'b0) ? (Half_data[15] ? {{16{1'b1}},Half_data} : {{16{1'b0}},Half_data})//I-Type LH操作
			     		 : (ins == `I && funct3 == 3'b100 && opcode[5:3] == 3'b0) ? {{24{1'b0}},Byte_data}		//I-Type LBU操作
			     		 : (ins == `I && funct3 == 3'b101 && opcode[5:3] == 3'b0) ? {{16{1'b0}},Half_data}		//I-Type LHU操作
			     		 : Read_data_current;																	//I-Type LW操作
	assign Byte_data = (ALU_result[1:0] == 2'b11) ? Read_data_current[31:24]
			 		 : (ALU_result[1:0] == 2'b10) ? Read_data_current[23:16]
			 		 : (ALU_result[1:0] == 2'b01) ? Read_data_current[15:8]
					 : Read_data_current[7:0];
	assign Half_data = (ALU_result[1:0] == 2'b00) ? Read_data_current[15:0] : Read_data_current[31:16];

	//内存读写操作
	assign MemRead = (current_state == LD) ? 1 : 0;						//I-Type内存读指令
	assign MemWrite = (current_state == ST) ? 1 : 0;					//I-Type内存写指令
	assign Address = {ALU_result[31:2],2'b0};							//内存地址,低二位为0
	assign Write_strb = (ins == `S && funct3 == 3'b000) ? (4'b1000 >> (~ALU_result[1:0]))			//I-Type SB操作
			  		  : (ins == `S && funct3 == 3'b001) ? {{2{ALU_result[1]}},{2{~ALU_result[1]}}}	//I-Type SH操作
			  		  : (ins == `S && funct3 == 3'b010) ? 4'b1111									//I-Type SW操作
			  		  : 4'b0;
	assign Write_data = (ins == `S && funct3 == 3'b000) ? (Write_strb[3] ? {RF_rdata2[7:0],24'b0}
					  : Write_strb[2] ? {8'b0,RF_rdata2[7:0],16'b0} 
					  : Write_strb[1] ? {16'b0,RF_rdata2[7:0],8'b0}
					  : {24'b0,RF_rdata2[7:0]})							//I-Type SB操作
			  		  : (ins == `S && funct3 == 3'b001) ? ((Write_strb[3] && Write_strb[2]) ? {RF_rdata2[15:0],16'b0}
					  : {16'b0,RF_rdata2[15:0]})						//I-Type SH操作
			  		  : RF_rdata2;										//I-Type SW操作

	//寄存器读写操作
	assign RF_wen = (current_state == WB) ? 1 : 0;									//只有在WB状态下才会向将内容写回
	assign RF_waddr = rd;															//riscv32指令中所有写地址均写到rd寄存器中
	assign RF_wdata = (ins == `U && opcode[5]) ? U_extend							//U-Type LUI指令
					: (ins == `I && opcode[6:4] == 3'b0) ? Read_reg_data			//I-Type Load操作
		     		: (ins == `J || (ins == `I && opcode[6])) ? (PC_current+4)		//J-Type操作/I-Type JALR操作
					: ((ins == `R || (ins == `I && opcode[4])) && funct3[1:0] == 2'b01) ? Shift_result	//SLL/SRL/SRA/SLLI/SRLI/SRAI操作
		     		: ((ins == `R || (ins == `I && opcode[4]) || (ins == `U && opcode[5] == 1'b0)) 
					  && funct3[1:0] != 2'b01 && opcode[6:4] != 3'b0) ? ALU_result	//R-Type/I-Type中除了移位操作、Load操作的其他操作，AUIPC操作
					: 32'b0;															
	
	//实例化
	reg_file reg_file_module(
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(rs1),
		.raddr2(rs2),
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
		.A(RF_rdata1),
		.B(Shifter_B),
		.Shiftop(Shiftop),
		.Result(Shift_result)
	);

	//第一段：描述状态寄存器的同步跳转状态
	always@(posedge clk)
	begin
		if(rst)
			current_state <= INIT;
		else
			current_state <= next_state;
	end

	//第二段：根据状态机当前状态和输入信号，描述下一状态的计算逻辑
	always@(*)
	begin
		case(current_state)
			INIT:	next_state = IF;
			IF: begin
				if(Inst_Req_Ready)
					next_state = IW;
				else
					next_state = IF;
			end
			IW: begin
				if(Inst_Valid)
					next_state = ID;
				else
					next_state = IW;
			end
			ID: begin
					next_state = EX;
			end
			EX: begin
				if(ins == `B)		//B-Type指令
					next_state = IF;
				else if(ins == `R || ins == `U || ins == `J || (ins == `I && opcode[6:4] != 3'b0))	//R-Type指令,I-Type运算指令,JAL指令
					next_state = WB;
				else if(ins == `S)	//S-Type指令
					next_state = ST;
				else if(ins == `I && opcode[6:4] == 3'b0)	//I-Type Load指令
					next_state = LD;
				else
					next_state = IF;
			end
			LD: begin
				if(Mem_Req_Ready)
					next_state = RDW;
				else
					next_state = LD;
			end
			ST: begin
				if(Mem_Req_Ready)
					next_state = IF;
				else
					next_state = ST;
			end
			RDW: begin
				if(Read_data_Valid)
					next_state = WB;
				else
					next_state = RDW;
			end
			WB: 	next_state = IF;
			default:
				next_state = current_state;
		endcase
	end

	//第三段：根据状态机当前状态，描述不同输出寄存器的同步变化
	//用current_state处理输出变量的组合逻辑
	assign PC_next 	= PC + 4;
	assign Inst_Req_Valid = (current_state == IF) ? 1 : 0;
	assign Inst_Ready = (current_state == IW || current_state == INIT) ? 1 : 0;
	assign Read_data_Ready = (current_state == INIT || current_state == RDW) ? 1 : 0;

	//PC寄存器的时序逻辑电路
	always@(posedge clk) 
	begin
		if (rst) 						//rst同步的高电平复位信号
		begin
			PC <= 32'b0;
		end									   			
		else if(current_state == EX)	//根据操作不同，选择相应的下一条指令的PC地址
		begin
			PC <= ({32{Jump}} & Jump_addr) | ({32{Branch}} & Branch_addr) | ({32{~Jump}} & {32{~Branch}} & PC_next);
		end
	end
	//判断是否更新PC值的时序逻辑电路
	always@(posedge clk)
	begin
		if (rst)
		begin
			PC_current <= 32'b0;
		end
		else if(current_state == IF)
			PC_current <= PC;
	end
	//判断是否更新Instruction_current值的时序逻辑电路
	always@(posedge clk)
	begin
		if(rst)
		begin
			Instruction_current <= 32'b0;
		end
		else if(Inst_Ready && Inst_Valid)
			Instruction_current <= Instruction;
	end
	//判断是否更新读数据值的时序逻辑电路
	always@(posedge clk)
	begin
		if(rst)
		begin
			Read_data_current <= 32'b0;
		end
		else if(Read_data_Ready && Read_data_Valid)
			Read_data_current <= Read_data;
	end

	//周期计数器
	always@(posedge clk)
	begin
		if(rst)
			cycle_cnt <= 32'b0;
		else
			cycle_cnt <= cycle_cnt + 32'b1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;		//将周期数存储到寄存器cnt_0中

	//指令数计数器
	always@(posedge clk)
	begin
		if(rst)
			inst_cnt <= 32'b0;
		else if(Inst_Ready && Inst_Valid)
			inst_cnt <= inst_cnt + 32'b1;
	end
	assign cpu_perf_cnt_1 = inst_cnt;		//将指令个数存储到寄存器cnt_1中

endmodule
