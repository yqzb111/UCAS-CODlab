`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,    //时钟
	input  [`ADDR_WIDTH - 1:0]  waddr,  //写口地址	
	input  [`ADDR_WIDTH - 1:0]  raddr1, //读口地址1
	input  [`ADDR_WIDTH - 1:0]  raddr2, //读口地址2
	input                       wen,    //写使能
	input  [`DATA_WIDTH - 1:0]  wdata,  //写数据
	output [`DATA_WIDTH - 1:0]  rdata1, //读口1数据
	output [`DATA_WIDTH - 1:0]  rdata2  //读口2数据
);

	// TODO: Please add your logic design here
	
	reg [`DATA_WIDTH-1:0] regfile [`DATA_WIDTH-1:0]; // 声明32个32位寄存器

    always @(posedge clk) begin
        if (wen && waddr != 0) begin // 当写使能wen为1且waddr非0时写入数据
            regfile[waddr] <= wdata;
        end
    end

    // 异步读数据1
    assign rdata1 = ({32{raddr1 == 0}} & `DATA_WIDTH'b0) | ({32{~(raddr1 == 0)}} & regfile[raddr1]);
    // 异步读数据2
    assign rdata2 = ({32{raddr2 == 0}} & `DATA_WIDTH'b0) | ({32{~(raddr2 == 0)}} & regfile[raddr2]);
    
endmodule