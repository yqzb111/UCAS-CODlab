.data
last_tail_ptr:
        .word 0

.globl intr_handler
	.align  2
	.type intr_handler, @function
	.section .exception

intr_handler:
	# TODO: Please add your own interrupt handler for DMA engine 
/*------------------------
	base: 0x60020000
	offset:
	tail_ptr: 0x0008
	dma_size: 0x0010
	ctrl_stat: 0x0014
------------------------*/
	#获取tail_ptr的值存到k0寄存器, 计算前后差值
	lui		$k0, 0x6002
	lw		$k0, 0x8 ($k0)
	sub		$k1, $k0, $k1

	/*通过循环计算子缓冲区数并更新dam_buf_stat值*/
LOOP:
	#每次dam_buf_stat减一
	lw		$k0, 0x10($0)
	addi	$k0, $k0, 0xffff
	sw		$k0, 0x10($0)
	#每次减去一个子缓冲区数
	lui		$k0, 0x6002
	lw		$k0, 0x10($k0)
	sub		$k1, $k1, $k0
	bgtz $k1, LOOP
	#将ctrl_stat的INTR标志位清0
	lui		$k1, 0x6002
	lw		$k0, 0x14 ($k1)
	andi	$k0, $k0, 0x1
	sw		$k0, 0x14 ($k1)
	#将tail_ptr传到k1
	lw		$k1, 0x8 ($k1)
	#退出中断
	eret

/*--------------------------------------------------------------------------------
#include <stdint.h>

// 定义基地址和偏移量
#define DMA_BASE 0x60020000
#define TAIL_PTR_OFFSET 0x0008
#define DMA_SIZE_OFFSET 0x0010
#define CTRL_STAT_OFFSET 0x0014

// 内存映射I/O函数
volatile uint32_t* get_address(uint32_t base, uint32_t offset) {
    return (volatile uint32_t*)(base + offset);
}

void intr_handler() {
    // 定义指向内存映射寄存器的指针
    volatile uint32_t* tail_ptr = get_address(DMA_BASE, TAIL_PTR_OFFSET);
    volatile uint32_t* dma_size = get_address(0, DMA_SIZE_OFFSET);
    volatile uint32_t* ctrl_stat = get_address(DMA_BASE, CTRL_STAT_OFFSET);

    // 加载tail_ptr的值
    uint32_t k0 = *tail_ptr;
    uint32_t k1 = 0;

    // 计算差值
    k1 = k0 - k1;

    // 循环更新dam_buf_stat
    do {
        // 减少dam_buf_stat
        uint32_t buf_stat = *dma_size;
        buf_stat -= 1;
        *dma_size = buf_stat;

        // 从k1中减去一个子缓冲区大小
        k0 = *get_address(DMA_BASE, DMA_SIZE_OFFSET);
        k1 -= k0;
    } while (k1 > 0);

    // 清除ctrl_stat寄存器中的INTR标志位
    uint32_t ctrl_stat_val = *ctrl_stat;
    ctrl_stat_val &= 0xFFFFFFFE; // 清除INTR标志
    *ctrl_stat = ctrl_stat_val;

    // 记录新的tail_ptr值
    k1 = *tail_ptr;
}
--------------------------------------------------------------------------------*/