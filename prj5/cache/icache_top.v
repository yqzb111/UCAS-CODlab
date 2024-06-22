`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here

	reg [2:0] len;					//用于判断是否拉高last
	reg valid_array [`CACHE_WAY-1:0][`CACHE_SET-1:0];	// 有效位
	reg	[31:0] array_data [7:0];	//数据数组，表示8个32bit的数组
	reg [7:0] current_state;
	reg [7:0] next_state;

	wire ReadMiss;				 //读命中
	wire ReadHit;				 //未命中
	wire wen;					 //读使能
	wire [2:0] index;            // 组索引
	wire [4:0] offset;           // 偏移量
	wire [`TAG_LEN-1:0] tag_in;  // 输入的标签
	wire [`TAG_LEN-1:0]	tag_rdata [`CACHE_WAY - 1:0];	//tag_array中读出的数据
	wire [`LINE_LEN-1:0] data_rdata	[`CACHE_WAY - 1:0];	//data_array中读出的数据
	wire hit_tag [`CACHE_WAY - 1:0];	//检验是否和tag_in匹配
	wire [1:0] selected_way;	 //表示选中的片序号
	wire [`LINE_LEN-1:0] selected_data;	//表示选中的32byte数据

	// 固定替换位置
	integer fixed_replace_way = 0;

	// 状态编码
	localparam WAIT = 8'b00000001,
	           TAG_RD = 8'b00000010,
	           CACHE_RD = 8'b00000100,
	           RESP = 8'b00001000,
	           EVICT = 8'b00010000,
	           MEM_RD = 8'b00100000,
	           RECV = 8'b01000000,
	           REFILL = 8'b10000000;

	assign to_cpu_inst_req_ready = (current_state == WAIT);
	assign to_cpu_cache_rsp_valid = (current_state == RESP);
	assign to_mem_rd_req_valid = (current_state == MEM_RD);
	assign to_mem_rd_rsp_ready = ((current_state == RECV) || rst);		//保证复位未释放时接口正确
	assign to_mem_rd_req_addr = {from_cpu_inst_req_addr[31:5], 5'b0};   // 对齐32字节

	// 地址解析
	assign index = from_cpu_inst_req_addr[7:5];  // 地址的组索引部分
	assign offset = from_cpu_inst_req_addr[4:0]; // 地址的块内偏移部分
	assign tag_in = from_cpu_inst_req_addr[31:8];// 地址的标签部分

	//判定是否读命中
	assign ReadHit	= hit_tag[0] | hit_tag[1] | hit_tag[2] | hit_tag[3];
	assign ReadMiss = ~ReadHit;
	assign wen = (current_state == REFILL);
	assign selected_way = {2{hit_tag[0]}} & 2'b00 | {2{hit_tag[1]}} & 2'b01 |
						  {2{hit_tag[2]}} & 2'b10 | {2{hit_tag[3]}} & 2'b11 ;
	assign selected_data = data_rdata[selected_way];
	assign to_cpu_cache_rsp_data = selected_data >> {offset[4:0],3'b0};	// CPU接口信号

	// 状态转移
	always @(posedge clk) begin
		if (rst) begin
			current_state <= WAIT; // 复位时进入等待状态
		end else begin
			current_state <= next_state; // 否则进入下一个状态
		end
	end

	// 状态机
	always @(*) begin
		case (current_state)
			WAIT: begin
				if (from_cpu_inst_req_valid) begin
					next_state = TAG_RD; // 如果收到有效请求，转到TAG_RD状态
				end else begin
					next_state = WAIT; // 否则保持在等待状态
				end
			end
			TAG_RD: begin
				if (ReadMiss) begin
					next_state = EVICT; // 如果未命中，转到EVICT状态
                end else if (ReadHit) begin
                    next_state = CACHE_RD;
                end else begin
					next_state = TAG_RD;
				end
			end
			CACHE_RD: begin
				next_state = RESP; // 转到RESP状态
			end
			RESP: begin
				if (from_cpu_cache_rsp_ready) begin
					next_state = WAIT; // 如果CPU准备好接收，转到WAIT状态
				end else begin
					next_state = RESP; // 否则保持在RESP状态
				end
			end
			EVICT: begin
				next_state = MEM_RD; // 转到MEM_RD状态
			end
			MEM_RD: begin
				if (from_mem_rd_req_ready) begin
					next_state = RECV; // 如果内存准备好接收请求，转到RECV状态
				end else begin
					next_state = MEM_RD; // 否则保持在MEM_RD状态
				end
			end
			RECV: begin
				if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
					next_state = REFILL; // 如果是最后一组数据，转到REFILL状态
				end else begin 
					next_state = RECV;
				end
			end
			REFILL: begin
				next_state = RESP; // 转到RESP状态
			end
			default: begin
				next_state = WAIT; // 默认状态为WAIT
			end
		endcase
	end

	/*创建tag_array和data_array*/
	genvar i_way;	//循环变量
	generate
		for (i_way=0;i_way<`CACHE_WAY;i_way=i_way+1) begin : ways
			tag_array tag_array_module(
				.clk	(clk),
				.waddr	(index),
				.raddr	(index),
				.wen	(wen),
				.wdata	(tag_in),
				.rdata	(tag_rdata[i_way])
			);
			assign hit_tag[i_way] = (valid_array[i_way][index] && (tag_rdata[i_way] == tag_in));
			data_array data_array_module(
				.clk	(clk),
				.waddr	(index),
				.raddr	(index),
				.wen	(wen),
				.wdata	({array_data[7],array_data[6],array_data[5],array_data[4],
						array_data[3],array_data[2],array_data[1],array_data[0]}),
				.rdata	(data_rdata[i_way])
			);
		end
	endgenerate

	integer j_way;	//循环变量
	always @(posedge clk) begin
		if (rst) begin
			for (j_way=0;j_way<`CACHE_SET;j_way=j_way+1) begin
				valid_array[0][j_way] <= 0;
				valid_array[1][j_way] <= 0;
				valid_array[2][j_way] <= 0;
				valid_array[3][j_way] <= 0;
			end
		end else if (!rst && (current_state == EVICT)) begin	//替换算法：每次都是将第0片中对应的位置进行替换
			valid_array[fixed_replace_way][index]<=1'b0;						//将对应的valid置为0
		end else if (!rst && (current_state == MEM_RD)) begin
			len <= 3'b000;
		end else if (!rst && (current_state == RECV) && from_mem_rd_rsp_valid) begin
			array_data[len] <= from_mem_rd_rsp_data;				//接收4-byte from_mem_rd_rsp_data
			len <= len + 1;										//直至from_mem_rd_rsp_last标记的最后一个4-byte数据已接收
		end else if (!rst && (current_state == REFILL)) begin
			valid_array[fixed_replace_way][index] <= 1'b1;
		end
	end

endmodule
