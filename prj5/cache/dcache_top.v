`timescale 10ns / 1ns

`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

  //TODO: Please add your D-Cache code here

	// 状态编码
	localparam WAIT			= 16'b0000_0000_0000_0001;
	localparam TAG_RD		= 16'b0000_0000_0000_0010;
	localparam CACHE_WR		= 16'b0000_0000_0000_0100;
	localparam CACHE_RD		= 16'b0000_0000_0000_1000;
	localparam CACHE_RESP	= 16'b0000_0100_0001_0000;
	localparam REFILL		= 16'b0000_0000_0010_0000;
	localparam EVICT		= 16'b0000_0000_0100_0000;
	localparam MEM_RD		= 16'b0000_0000_1000_0000;
	localparam MEM_WR		= 16'b0000_0001_0000_0000;
	localparam CACHE_WB		= 16'b0000_0010_0000_0000;
	localparam RECV_MEM1	= 16'b0000_0100_0000_0000;	
	localparam BYPASS_RD	= 16'b0000_1000_0000_0000;
	localparam BYPASS_WR	= 16'b0001_0000_0000_0000;	
	localparam RECV_MEM2	= 16'b0010_0000_0000_0000;
	localparam RECV_CPU		= 16'b0100_0000_0000_0000;	
	localparam CPU_WB		= 16'b1000_0000_0000_0000;

	reg  [2:0] len;												// 用于判断是否拉高last
	reg	 valid_array [`CACHE_WAY - 1:0][`CACHE_SET - 1:0];		// 有效位数组
	reg	 dirty_array [`CACHE_WAY - 1:0][`CACHE_SET - 1:0];		// 脏数组
	reg  [31:0] array_data [7:0];								// 数据数组，表示8个32bit的数组
	reg  [31:0]	data_wb;										// 从内存中写回的数据
	reg  [31:0]	data_bypass;									// 旁路写回CPU的数据
	reg  [31:0] Bypass_address_current;							// 旁路读写地址
	reg  [31:0] Write_data_current;								// 旁路写回内存的数据
	reg  [3:0]	Write_strb_current;								// 旁路写回使能
	reg			Mem_req_current;								// 存储WAIT状态时的from_cpu_mem_req值
	reg  [17:0] current_state;									
	reg  [17:0] next_state;

	wire Cacheable;												// Cache使能
	wire Cache_hit;												// Cache命中			
	wire Cache_miss;											// Cache未命中
	wire Cache_dirty;											// Cache脏
	wire [`TAG_LEN-1:0] tag_in;  								// 输入的标签
	wire [2:0] index;											// 组索引
	wire [4:0] offset;											// 偏移量
	wire wen [`CACHE_WAY - 1:0];								// 读写使能
	wire [`TAG_LEN - 1:0] tag_rdata	[`CACHE_WAY - 1:0];			// tag_array中读出的数据
	wire [`LINE_LEN - 1:0] data_rdata [`CACHE_WAY - 1:0];		// data_array中读出的数据
	wire hit_tag [`CACHE_WAY - 1:0];							// 检验是否和tag_in匹配
	wire [1:0] selected_way;									// 表示选中的片序号
	wire [`LINE_LEN - 1:0] selected_data;						// 表示选中的32byte数据
	wire [`LINE_LEN - 1:0] replaced_data;						// 表示替换的数据	

	// 固定替换位置
	integer fixed_replace_way = 0;

	//输出赋值
	assign to_cpu_mem_req_ready		= (current_state == WAIT);
	assign to_cpu_cache_rsp_valid	= (current_state == CACHE_RESP || current_state == RECV_CPU);
	assign to_cpu_cache_rsp_data	= {32{current_state == CACHE_RESP}} & (selected_data[31:0])
									| {32{current_state == RECV_CPU}} & data_bypass;
	assign to_mem_rd_req_valid		= current_state == MEM_RD || current_state == BYPASS_RD;
	assign to_mem_rd_req_addr		= {32{current_state == MEM_RD}} & {tag_in, index, 5'b00000}
									| {32{current_state == BYPASS_RD}} & Bypass_address_current;
	assign to_mem_rd_req_len		= {8{current_state == MEM_RD}} & 8'b111
									| {8{current_state == BYPASS_RD}} & 8'b0;
	assign to_mem_rd_rsp_ready		= (current_state == RECV_MEM1) || (current_state == RECV_MEM2) || rst;
	assign to_mem_wr_req_valid		= current_state == MEM_WR || current_state == BYPASS_WR;
	assign to_mem_wr_req_addr		= {32{current_state == MEM_WR}}	& {tag_rdata[fixed_replace_way], index, 5'b00000}
									| {32{current_state == BYPASS_WR}} & Bypass_address_current;
	assign to_mem_wr_req_len		= {8{current_state == MEM_WR}} & 8'b111
									| {8{current_state == BYPASS_WR}} & 8'b0;
	assign to_mem_wr_data_valid		= (current_state == CACHE_WB) || (current_state == CPU_WB);
	assign to_mem_wr_data			= {32{current_state == CACHE_WB}} & data_wb
									| {32{current_state == CPU_WB}}	& Write_data_current;
	assign to_mem_wr_data_strb		= {4{current_state == CACHE_WB}}
									| {4{current_state == CPU_WB}}	& Write_strb_current;
	assign to_mem_wr_data_last		= (current_state == CACHE_WB && len == 3'b000)
								   || current_state == CPU_WB;

	// 地址解析
	assign index  					= Bypass_address_current[7:5];  	// 地址的组索引部分
	assign offset 					= Bypass_address_current[4:0]; 		// 地址的块内偏移部分
	assign tag_in 					= Bypass_address_current[31:8];		// 地址的标签部分

	// 判定是否可以访问Cache、是否读命中、是否脏
	assign Cacheable 				= (|from_cpu_mem_req_addr[31:5]) && ~from_cpu_mem_req_addr[31] && ~from_cpu_mem_req_addr[30];
	assign Cache_hit 	 			= hit_tag[0] | hit_tag[1] | hit_tag[2] | hit_tag[3];
	assign Cache_miss		 		= ~Cache_hit;	
	assign Cache_dirty 	 			= valid_array[fixed_replace_way][index] && dirty_array[fixed_replace_way][index];

	// 选中的片序号和数据
	assign selected_way 			= {2{hit_tag[0]}} & 2'b00 | {2{hit_tag[1]}} & 2'b01 |
						  			  {2{hit_tag[2]}} & 2'b10 | {2{hit_tag[3]}} & 2'b11 ;
	assign selected_data 			= data_rdata[selected_way] >> {offset[4:0], 3'b000};
	assign replaced_data			= data_rdata[fixed_replace_way] >> {len, 5'b00000};

	//使能判断
	//由于分为从旁路读写数据和从Cache读写数据，因此写使能与Cache的片数有关
	assign wen[0] 					= current_state == REFILL || current_state == CACHE_WR && selected_way == 2'b00;
	assign wen[1] 					= current_state == REFILL || current_state == CACHE_WR && selected_way == 2'b01;
	assign wen[2] 					= current_state == REFILL || current_state == CACHE_WR && selected_way == 2'b10;
	assign wen[3] 					= current_state == REFILL || current_state == CACHE_WR && selected_way == 2'b11;

	// 状态转移
	always @(posedge clk) begin
		if (rst) begin
			current_state <= WAIT; 			// 复位时进入等待状态
		end else begin
			current_state <= next_state; 	// 否则进入下一个状态
		end
	end

	// 状态机
	always @(*) begin
		case (current_state)
			WAIT: begin														//判断地址是否在Cache允许的范围内
				if (from_cpu_mem_req_valid && Cacheable) begin				
					next_state = TAG_RD;
				end else if (from_cpu_mem_req_valid && !Cacheable && from_cpu_mem_req) begin	
					next_state = BYPASS_WR;
				end else if (from_cpu_mem_req_valid && !Cacheable && !from_cpu_mem_req) begin	
					next_state = BYPASS_RD;
				end else begin
					next_state = WAIT;
				end
			end
			TAG_RD: begin													//判断是否命中
				if (Cache_miss) begin													
					next_state = EVICT;
				end else if (Cache_hit && Mem_req_current) begin										
					next_state = CACHE_WR;
				end else if (Cache_hit && !Mem_req_current) begin										
					next_state = CACHE_RD;
				end else begin
					next_state = TAG_RD;
				end
			end
			CACHE_WR: begin													//无条件写回
				next_state = WAIT;
			end
			CACHE_RD: begin													//无条件读取数据
				next_state = CACHE_RESP;
			end
			CACHE_RESP: begin
				if (from_cpu_cache_rsp_ready) begin							//CPU允许接收数据
					next_state = WAIT;
				end else begin
					next_state = CACHE_RESP;
				end
			end
			REFILL: begin													//判断是读操作还是写操作
				if (Mem_req_current) begin											
					next_state = CACHE_WR;
				end else if (!Mem_req_current) begin
					next_state = CACHE_RD;
				end else begin
					next_state = REFILL;
				end
			end
			EVICT: begin													//判断是否为脏数据
				if (Cache_dirty) begin											
					next_state = MEM_WR;
				end else if (!Cache_dirty) begin									
					next_state = MEM_RD;
				end else begin
					next_state = EVICT;
				end
			end	 
			MEM_RD: begin
				if (from_mem_rd_req_ready) begin							//允许接收内存读请求
					next_state = RECV_MEM1;
				end else begin
					next_state = MEM_RD;
				end
			end	
			MEM_WR: begin
				if (from_mem_wr_req_ready) begin							//允许接收内存写请求
					next_state = CACHE_WB;
				end else begin												
					next_state = MEM_WR;
				end
			end
			CACHE_WB: begin
				if (from_mem_wr_data_ready && to_mem_wr_data_last) begin	//允许内存接收数据（32byte突发传输）
					next_state = MEM_RD;
				end else begin	
					next_state = CACHE_WB;
				end
			end	 
			RECV_MEM1: begin
				if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin	//允许从内存将数据读入Cache（突发传输）
					next_state = REFILL;
				end else begin
					next_state = RECV_MEM1;
				end
			end 
			BYPASS_RD: begin
				if (from_mem_rd_req_ready) begin							//允许接收内存读请求
					next_state = RECV_MEM2;
				end else begin
					next_state = BYPASS_RD;
				end
			end
			BYPASS_WR: begin
				if (from_mem_wr_req_ready) begin							//允许接收内存写请求
					next_state = CPU_WB;
				end else begin
					next_state = BYPASS_WR;
				end
			end
			RECV_MEM2: begin		
				if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin	//允许从内存读出数据（突发传输）
					next_state = RECV_CPU;
				end else begin
					next_state = RECV_MEM2;
				end
			end
			RECV_CPU: begin
				if (from_cpu_cache_rsp_ready) begin							//CPU允许接收数据
					next_state = WAIT;	
				end else begin
					next_state = RECV_CPU;
				end
			end
			CPU_WB: begin
				if (from_mem_wr_data_ready && to_mem_wr_data_last) begin	//允许内存接收数据（突发传输）
					next_state = WAIT;
				end else begin
					next_state = CPU_WB;
				end
			end
			default: next_state = WAIT;
		endcase
	end

	//请求数据缓冲
	always @(posedge clk) begin
		if (current_state == WAIT) begin
			Bypass_address_current	<= from_cpu_mem_req_addr;
			Write_data_current 		<= from_cpu_mem_req_wdata;
			Write_strb_current 		<= from_cpu_mem_req_wstrb;
			Mem_req_current			<= from_cpu_mem_req;
		end
	end

	/*创建tag_array和data_array*/
	genvar i_way;	//循环变量
	generate
		for (i_way=0; i_way<`CACHE_WAY; i_way=i_way+1) begin : ways
			tag_array tag_array_module(
				.clk	(clk),
				.waddr	(index),
				.raddr	(index),
				.wen	(wen[i_way]),
				.wdata	(tag_in),
				.rdata	(tag_rdata[i_way])
			);
			assign hit_tag[i_way] = (valid_array[i_way][index] && (tag_rdata[i_way] == tag_in));
			data_array data_array_module(
				.clk	(clk),
				.waddr	(index),
				.raddr	(index),
				.wen	(wen[i_way]),
				.wdata	(current_state == REFILL 
					  ? {array_data[7],array_data[6],array_data[5],array_data[4],
						array_data[3],array_data[2],array_data[1],array_data[0]} 
					  : current_state == CACHE_WR 
					  ? (data_rdata[i_way] & ~({224'b0,{8{Write_strb_current[3]}},{8{Write_strb_current[2]}},
						{8{Write_strb_current[1]}},{8{Write_strb_current[0]}}} << {offset,3'b000}) | 
						({224'b0,Write_data_current & {{8{Write_strb_current[3]}},{8{Write_strb_current[2]}},
						{8{Write_strb_current[1]}},{8{Write_strb_current[0]}}}} << {offset,3'b000})) 
					  : 256'b0),
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
		end else if (current_state == EVICT) begin									// 替换算法：每次都是将第0片中对应的位置进行替换
			valid_array[fixed_replace_way][index] <= 1'b0; 
			len <= 3'b000; 
		end else if (current_state != CACHE_WB && next_state == CACHE_WB) begin		// 将待替换的缓存块写回内存
			data_wb <= replaced_data[31:0];
			len <= len + 1;
		end else if (current_state == CACHE_WB && from_mem_wr_data_ready) begin		// 将待替换的缓存块写回内存
			data_wb <= replaced_data[31:0];
			len <= len + 1;
		end else if (current_state == MEM_RD) begin
			len <= 3'b000;
		end else if (current_state == RECV_MEM1 && from_mem_rd_rsp_valid) begin		// 从内存接收新缓存块的数据
			array_data[len] <= from_mem_rd_rsp_data;
			len <= len + 1;
		end else if (current_state == REFILL) begin									// 重填后修改标记
			valid_array[fixed_replace_way][index] <= 1'b1; 
			dirty_array[fixed_replace_way][index] <= 1'b0;
		end else if (current_state == CACHE_WR) begin								// 将来自CPU的修改写入对应缓存块后标记为脏
			dirty_array[selected_way][index] <= 1'b1;
		end else if (current_state == RECV_MEM2 && from_mem_rd_rsp_valid) begin		//接受内存回复的数据
			data_bypass <= from_mem_rd_rsp_data;
		end
	end

endmodule
