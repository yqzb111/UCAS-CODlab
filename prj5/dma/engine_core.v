`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`define IDLE 4'b0001
`define RD_WR_REQ 4'b0010
`define RD_WR 4'b0100
`define FIFO 4'b1000

module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	//控制/状态寄存器与处理器的互连端口
	output reg [31:0]       src_base,		//DMA读基地址寄存器值
	output reg [31:0]       dest_base,		//DMA写基地址寄存器值
	output reg [31:0]       tail_ptr,		//DMA队列尾指针寄存器值
	output reg [31:0]       head_ptr,		//DMA队列头指针寄存器值
	output reg [31:0]       dma_size,		//DMA读/写数据块大小寄存器值
	output reg [31:0]       ctrl_stat,		//DMA控制/状态寄存器值
	
	input  [31:0]	    reg_wr_data,		//处理器向I/O寄存器发送的写数据
	input  [ 5:0]       reg_wr_en,			//处理器发送的寄存器写请求写使能信号
  
	output              intr,				//向处理器发送的中断请求信号

	//读引擎与内存控制器的互连端口
	output [31:0]       rd_req_addr,		//读地址（只需在每次突发传输时传输一次，与rd_req_valid同时有效，且按32-byte对齐）
	output [ 4:0]       rd_req_len,			//读突发传输长度（与rd_req_valid同时有效）
	output              rd_req_valid,		//读请求有效

	input               rd_req_ready,		//读请求响应
	input  [31:0]       rd_rdata,			//读数据
	input               rd_last,			//读数据last信号（和最后一个有效读数据同时拉高）
	input               rd_valid,			//读数据有效
	output              rd_ready,			//读数据响应
	
	//写引擎与与内存控制器的互连端口
	output [31:0]       wr_req_addr,		//写请求地址（只需在每次突发传输时传输一次，与wr_req_valid同时有效）
	output [ 4:0]       wr_req_len,			//写突发传输长度（与wr_req_valid同时有效）
	output              wr_req_valid,		//写请求有效
	input               wr_req_ready,		//写请求响应
	output [31:0]       wr_data,			//写数据
	output              wr_valid,			//写数据有效
	input               wr_ready,			//写数据响应
	output              wr_last,			//写数据last信号（和最后一个写数据同时拉高）
	
	//读/写引擎与FIFO队列接口信号
	output              fifo_rden,			//读使能（下一时钟上升沿读数据有效）
	output [31:0]       fifo_wdata,			//写数据
	output              fifo_wen,			//写使能（需和写数据同时有效）
	
	input  [31:0]       fifo_rdata,			//读数据（有效一拍）
	input               fifo_is_empty,		//队列空
	input               fifo_is_full		//队列满
);
	// TODO: Please add your logic design here
	reg [3:0] rd_current_state;             //当前读状态
	reg [3:0] rd_next_state;                //下一读状态
	reg [3:0] wr_current_state;             //当前写状态
	reg [3:0] wr_next_state;                //下一写状态

    //突发传输
	wire [2:0] last_burst;                  //最后一次传输长度
	wire [31:0] burst_times;          		//总传输长度

	wire EN;                                //DMA使能位

    //写操作未写满32bit时的临时寄存器和计数器
	reg [31:0] fifo_data;                   //写操作寄存器
    reg [31:0] fifo_data_counter;           //写操作计数器

    //读写计数器
	reg [31:0] rd_counter;
	reg [31:0] wr_counter;
	
    //ctrl_stat寄存器bitmap
    assign intr = ctrl_stat[31];
	assign EN = ctrl_stat[0];

    /*读引擎与内存控制器的互连端口输出赋值*/
	assign rd_req_addr = src_base + tail_ptr + (rd_counter << 5);	//每次传输32bit，所以左移5位
	//传输长度为4*(rd_req_len+1)字节,若传输32字节，则取7，若最后一次传输不为字节，则取last_burst。
	assign rd_req_len = ((rd_counter == burst_times - 1) && |last_burst) ? {2'b0, {3{last_burst - 1'b1}}} : 5'b111;
	assign rd_req_valid = (rd_current_state == `RD_WR_REQ) & ~fifo_is_full & ~(rd_counter == burst_times);
	assign rd_ready = (rd_current_state == `RD_WR);

    /*写引擎与与内存控制器的互连端口输出赋值*/
	assign wr_req_addr = dest_base + tail_ptr + (wr_counter << 5);
	assign wr_req_len = ((wr_counter == burst_times - 1) && |last_burst)? {2'b0, {3{last_burst - 1'b1}}} : 5'b111;
	assign wr_req_valid = (wr_current_state == `RD_WR_REQ) & ~fifo_is_empty;
	assign wr_valid = (wr_current_state == `RD_WR);
	assign wr_data = fifo_data;
	assign wr_last = (fifo_data_counter == wr_req_len[2:0]);

    /*读写引擎与FIFO队列接口信号输出赋值*/
	assign fifo_rden = (wr_next_state == `FIFO);
	assign fifo_wen = rd_ready & rd_valid & ~fifo_is_full;
	assign fifo_wdata = rd_rdata;

    //突发传输次数计算
	assign last_burst = dma_size[4:2] + |dma_size[1:0];           //最后一次传输长度int((N%32)/4) + ( ((N % 32) % 4) != 0
	assign burst_times = {5'b0, dma_size[31:5]} + |dma_size[4:0]; //总共需要传输的次数
        
    //控制/状态寄存器与处理器的互连端口的赋值
	always @ (posedge clk) begin
		if (reg_wr_en[0]) 
            src_base <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[1]) 
            dest_base <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[2]) 
            tail_ptr <= reg_wr_data;
        //读写数据量全部达到dma_size时，本次DMA子缓冲区传输结束，发送中断请求
		else if ((rd_counter == burst_times) && (wr_counter == burst_times) && 
				(rd_current_state == `IDLE) && (wr_current_state == `IDLE))
			tail_ptr <= tail_ptr + dma_size;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[3]) 
            head_ptr <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[4]) 
            dma_size <= reg_wr_data;
	end
	always @ (posedge clk) begin
		if (reg_wr_en[5]) 
            ctrl_stat <= reg_wr_data;
        //读写数据量全部达到dma_size时，本次DMA子缓冲区传输结束，发送中断请求
		else if (EN && (rd_counter == burst_times) && (wr_counter == burst_times) && 
				(rd_current_state == `IDLE) && (wr_current_state == `IDLE))
			ctrl_stat[31] = 1'b1;
	end

    //计数器操作：每次Burst传输完成，计数器加一。每次在完成一个DMA子缓冲区的传输后归零。
    always @ (posedge clk) begin
		if (rst || (rd_current_state == `IDLE) && (wr_current_state == `IDLE) && EN && 
			~(head_ptr == tail_ptr) && (rd_counter == burst_times) && (wr_counter == burst_times))
			rd_counter <= 32'b0;
		else if ((rd_current_state == `RD_WR) && rd_valid && rd_last)
			rd_counter <= rd_counter + 1;
	end
    always @ (posedge clk) begin
		if (rst || (rd_current_state == `IDLE) && (wr_current_state == `IDLE) && EN && 
			~(head_ptr == tail_ptr) && ~intr && (rd_counter == burst_times) && (wr_counter == burst_times))
			wr_counter <= 32'b0;
		else if ((wr_current_state == `RD_WR) && wr_ready && wr_last)
			wr_counter <= wr_counter + 1;
	end

    /*读写引擎状态机*/
    //第一段
	always @ (posedge clk) begin
		if (rst) 
            rd_current_state <= `IDLE;
		else 
            rd_current_state <= rd_next_state;
	end
    always @ (posedge clk) begin
		if (rst) 
            wr_current_state <= `IDLE;
		else 
            wr_current_state <= wr_next_state;
	end
    //第二段
	always @ (*) begin
		case (rd_current_state)
			`IDLE: begin
				if (EN && (wr_current_state == `IDLE) && ~(head_ptr == tail_ptr) && ~(rd_counter == burst_times) && fifo_is_empty)
					rd_next_state = `RD_WR_REQ;
				else 
                    rd_next_state = `IDLE;
			end
			`RD_WR_REQ: begin
				if (fifo_is_full || (rd_counter == burst_times)) 
                    rd_next_state = `IDLE;
				else if (rd_req_ready) 
                    rd_next_state = `RD_WR;
				else 
                    rd_next_state = `RD_WR_REQ;
			end
			`RD_WR: begin
				if (rd_valid && rd_last && ~fifo_is_full) 
                    rd_next_state = `RD_WR_REQ;
				else 
                    rd_next_state = `RD_WR;
			end
			default: rd_next_state = `IDLE;
		endcase
	end	
	always @ (*) begin
		case (wr_current_state)
			`IDLE: begin
				if (EN && (rd_current_state == `IDLE) && ~(head_ptr == tail_ptr) && ~(wr_counter == burst_times) && fifo_is_full)
					wr_next_state = `RD_WR_REQ;
				else 
                    wr_next_state = `IDLE;
			end
			`RD_WR_REQ: begin
				if ((wr_counter == burst_times) || fifo_is_empty) 
                    wr_next_state = `IDLE;
				else if (wr_req_ready) 
                    wr_next_state = `FIFO;
				else 
                    wr_next_state = `RD_WR_REQ;
			end
			`FIFO: begin
				wr_next_state = `RD_WR;
			end
			`RD_WR: begin
				if (wr_ready && wr_last || fifo_is_empty) 
                    wr_next_state = `RD_WR_REQ;
				else if (wr_ready && ~fifo_is_empty) 
                    wr_next_state = `FIFO;
				else 
                    wr_next_state = `RD_WR;
			end
			default: wr_next_state = `IDLE;
		endcase
	end
        
    //第三段
	always @ (posedge clk) begin
		if ((wr_current_state == `FIFO)) 
            fifo_data <= fifo_rdata;
	end
	always @ (posedge clk) begin
		if (rst || (wr_current_state == `RD_WR_REQ)) 
            fifo_data_counter <= 3'b0;
		else if ((wr_current_state == `RD_WR) && wr_ready) 
            fifo_data_counter <= fifo_data_counter + 1;
	end
  
endmodule
