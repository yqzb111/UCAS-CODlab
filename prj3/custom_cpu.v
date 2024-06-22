`timescale 10ns / 1ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module custom_cpu(
	input         clk,
	input         rst,

	//Valid:高电平表示发送方发出的请求或应答内容有效
	//Ready:高电平表示接收方可以接收发送方的请求或应答

	//Instruction request channel
	output reg [31:0] PC,		//程序计数器
	output        Inst_Req_Valid,	
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,	//从内存（Memory）中读取至处理器的指令
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,		//数据访存指令使用的内存地址
	output        MemWrite,		//内存访问的写使能信号（高电平有效）
	output [31:0] Write_data,	//内存写操作数据
	output [ 3:0] Write_strb,	//内存写操作字节有效信号（支持32/16/8-bit内存写）	
								//Write_strb[i] == 1表示
								//Write_data[8×(i+1)-1 : 8×i]位会被写入内存的对应地址
	output        MemRead,		//内存访问的读使能信号（高电平有效）
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,	//从内存中读取的数据
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,		//实验5中使用

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

	wire			RF_wen;
	wire [`ADDR_WIDTH - 1:0]	RF_waddr;
	wire [`DATA_WIDTH - 1:0]	RF_wdata;
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
	assign	opcode = Instruction_current[31:26];				//opcode占Instruction_current的高6位
	assign	rs     = Instruction_current[25:21];				//rs为Instruction_current的21-25位
	assign	rt     = Instruction_current[20:16];				//rt为Instruction_current的16-20位
	assign	rd     = Instruction_current[15:11];				//rd为Instruction_current的11-15位
	assign	sa     = Instruction_current[10:6];				//sa为Instruction_current的6-10位
	assign	func   = Instruction_current[5:0];				//func为Instruction_current的低6位

	//操作数扩展
	assign	Zero_extend         	= {16'b0, Instruction_current[15:0]};
	assign	Arithmetic_extend	= Instruction_current[15] ? {{16{1'b1}},Instruction_current[15:0]} : {{16{1'b0}},Instruction_current[15:0]};
	assign	Shift_arithmetic_extend = Instruction_current[15] ? {{14{1'b1}},Instruction_current[15:0], 2'b0} : {{14{1'b0}},Instruction_current[15:0], 2'b0};

	//ALU操作
	assign ALUop = (opcode == 6'b0 && func[3:2] == 2'b0) ? {func[1],2'b10}						//R-Type ADD/SUB操作
		     : (opcode == 6'b0 && func[3:2] == 2'b01) ? {func[1],1'b0,func[0]}	 				//R-Type AND/OR/XOR/NOR操作
		     : (opcode == 6'b0 && func[3:2] == 2'b10) ? {~func[0],2'b11}					//R-Type SLT/SLTU操作
	   	     : (opcode[5:3] == 3'b001 && opcode[2:1] == 2'b0) ? 3'b010						//I-Type ADDIU操作
		     : (opcode[5:3] == 3'b001 && opcode[2] == 1'b1 && opcode[1:0] != 2'b11) ? {opcode[1],1'b0,opcode[0]}//I-Type ANDI/XORI/ORI操作
		     : (opcode[5:3] == 3'b001 && opcode[2:1] == 2'b01) ? {~opcode[0],2'b11}				//I-Type SLTI/SLTIU操作
		     : (opcode == 6'b000001 || opcode[5:1] == 5'b00011) ? 3'b111					//REGIMM BLTZ/BGEZ操作；I-Type BLEZ/BGTZ操作
		     : (opcode[5:1] == 5'b00010) ? 3'b110								//I-Type BEQ/BNE操作
		     : (opcode[5]) ? 3'b010									//I-Type 内存读写指令
		     : 3'b0;												//其他操作
	assign ALU_A = (opcode[5:1] == 5'b00011) ? 32'b0 : RF_rdata1;							//若为BLEZ/BGTZ操作，则输入为0，否则正常输入
	assign ALU_B = (opcode[5:1] == 5'b00011) ? RF_rdata1								//I-Type BLEZ/BGTZ操作
		     : (opcode == 6'b000001) ? 32'b0									//REGIMM BLTZ/BGEZ操作
		     : (opcode[5:3] == 3'b001 && opcode != 6'b001001) ? Zero_extend					//除ADDIU以外的I-Type计算指令
		     : (opcode == 6'b001001 || opcode[5] == 1) ? Arithmetic_extend					//ADDIU/I-Type内存读写指令
		     : RF_rdata2;											//其他操作

	//Shifter操作
	assign Shiftop = (opcode == 6'b0 && func[5:3] == 3'b0) ? func[1:0] : 2'b0;					//R-Type移位指令
	assign Shifter_B = (func[2] == 0) ? sa : RF_rdata1[4:0];							//若为SLL/SRA/SRL，则移位位数由sa决定；若为SLLV/SRAV/SRLV，则由rs低五位决定

	//Jump操作
	assign Jump = ((opcode == 6'b0 && func[5:3] == 3'b001 && func[1] == 1'b0) || opcode[5:1] == 5'b00001) ? 1 : 0;	//R-Type JR/JALR操作；J-Type J/JAL操作
	assign Jump_addr = (opcode == 6'b0 && func[5:3] == 3'b001 && func[1] == 1'b0) ? RF_rdata1			//若为R-Type JR/JALR操作，则跳转地址为rs的值
			 : {PC_next[31:28],Instruction_current[25:0],2'b0};							//若为J-Type J/JAL操作，则跳转地址为新地址低28位为instr_index左移两位，
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
			     : Read_data_current;										//I-Type LW操作
	assign Byte_data = (ALU_result[1:0] == 2'b11) ? Read_data_current[31:24]
			 : (ALU_result[1:0] == 2'b10) ? Read_data_current[23:16]
			 : (ALU_result[1:0] == 2'b01) ? Read_data_current[15:8]
			 : Read_data_current[7:0];
	assign Half_data = (ALU_result[1:0] == 2'b00) ? Read_data_current[15:0] : Read_data_current[31:16];
	assign LWL_data	= (ALU_result[1:0] == 2'b11) ? Read_data_current[31:0]
			: (ALU_result[1:0] == 2'b10) ? {Read_data_current[23:0], RF_rdata2[7:0]}
			: (ALU_result[1:0] == 2'b01) ? {Read_data_current[15:0], RF_rdata2[15:0]}
			: {Read_data_current[7:0], RF_rdata2[23:0]};
	assign LWR_data	= (ALU_result[1:0] == 2'b11) ? {RF_rdata2[31:8],Read_data_current[31:24]}
			: (ALU_result[1:0] == 2'b10) ? {RF_rdata2[31:16],Read_data_current[31:16]}
			: (ALU_result[1:0] == 2'b01) ? {RF_rdata2[31:24],Read_data_current[31:8]}
			: Read_data_current[31:0];

	//内存读写操作
	assign MemRead = (current_state == LD) ? 1 : 0;								//I-Type内存读指令
	assign MemWrite = (current_state == ST) ? 1 : 0;								//I-Type内存写指令
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

	//寄存器读写操作
	assign RF_wen = (opcode == 6'b000001 || opcode[5:2] == 4'b0001 || (opcode[5] && opcode[3]) 
		     || (opcode == 6'b0 && func == 6'b001000)) || (opcode == 6'b000010) ? 0		//REGIMM指令/I-Type分支指令/I-Type内存写指令/R-Type JR指令/J-Type J指令
		      : (opcode == 6'b0 && func[5:1] == 5'b00101) ? func[0] ^ (RF_rdata2 == 32'b0)			//R-Type MOVZ/MOVN指令
			  : ((opcode == 6'b000011 && current_state == EX) || (current_state == WB)
			 || (opcode == 6'b0 && func == 6'b001001 && current_state == EX)) ? 1					//J-Type JAL指令/R-Type JALR指令
	          : 0;
	assign RF_waddr = ((opcode == 6'b0 && func == 6'b001001 && rd == 5'b0) || opcode[5:1] == 5'b00001) ? 31		//J-Type指令/R-Type JALR指令(rd为0时)
			: (opcode[5:3] == 3'b001 || opcode[5:3] == 3'b100) ? rt						//I-Type计算指令/I-Type内存读指令
		     	: rd;
	assign RF_wdata = (opcode == 6'b0 && ((func == 6'b001010 && RF_rdata2 == 32'b0) 
		       || (func == 6'b001011 && RF_rdata2 != 32'b0))) ? RF_rdata1					//R-Type MOVZ/MOVN指令
		     	: (opcode == 6'b001111) ? {Instruction_current[15:0],16'b0}						//I-Type LUI指令
			: ((opcode == 6'b0 && func[5] == 1'b1) || opcode[5:3] == 3'b001) ? ALU_result			//R-Type计算指令/I-Type计算指令
			//注意上两行的条件判断有重叠的地方，顺序交换会导致错误，必须先把条件强的放前面
		     	: ((opcode == 6'b0 && func[5:1] == 5'b00100) || opcode[5:1] == 5'b00001) ? (PC_current+8)			//R-Type跳转指令/J-Type指令
			: (opcode == 6'b0 && func[5:3] == 3'b0) ? Shift_result						//R-Type移位指令
		     	: (opcode[5:3] == 3'b100) ? Read_reg_data							//I-Type 内存读指令
		     	: 32'b0;															
	
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
				if(Instruction_current != 32'b0)	//非NOP指令
					next_state = EX;
				else
					next_state = IF;
			end
			EX: begin
				if(opcode == 6'b000001 || opcode[5:2] == 4'b0001 || opcode == 6'b000010)	//REGIMM,I-Type跳转指令,J指令
					next_state = IF;
				else if(opcode == 6'b0 || opcode[5:3] == 3'b001 || opcode == 6'b000011)		//R-Type指令,I-Type运算指令,JAL指令
					next_state = WB;
				else if(opcode[5:3] == 3'b101)	//I-Type内存写指令
					next_state = ST;
				else if(opcode[5:3] == 3'b100)	//I-Type内存读指令
					next_state = LD;
				else
					next_state = EX;
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
		if (rst) 			//rst同步的高电平复位信号
		begin
			PC <= 32'b0;
		end									   			
		else if(current_state == EX)	//根据操作不同，选择相应的下一条指令的PC地址
		begin
			PC <= ({32{Jump}} & Jump_addr) | ({32{Branch}} & Branch_addr) | ({32{~Jump}} & {32{~Branch}} & PC_next);
		end
		else if (Instruction == 32'b0 && current_state == IW && Inst_Ready && Inst_Valid)	//当指令为NOP
		begin
			PC <= PC_next;
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
