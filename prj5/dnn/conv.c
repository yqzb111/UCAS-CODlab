#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"

#define FRAC_BIT 10			//用整数表示浮点数中低十位表示小数部分

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

//输出特征图的尺寸，d1表示通道数；d2表示特征图高度；d3表示特征图宽度
struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution() {
	short *in = (short *)addr.rd_addr;			// 输入图像数组首地址指针
	short *weight = (short *)addr.weight_addr;	// 卷积核数组首地址指针
	short *out = (short *)addr.wr_addr;			// 输出特征图数组首地址指针

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	// 计算输出特征图的宽度和高度
	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;	//步幅

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;
	conv_size.d3 = conv_out_w;

	//TODO: Please add your implementation here

	/*定义输入图像、卷积核、输出特征图数组*/
	//Input[ic][ih][iw],ic表示输入图像的通道数，ih表示输入图像的高度,iw表示输入图像的宽度
	typedef short (*IN)[rd_size.d1][rd_size.d2][rd_size.d3];	
	//Filter[oc][ic][1+k×k],oc表示输出特征图的通道数，ic表示输入图像的通道数（行下标），k表示卷积核尺寸（列下标）
	typedef short (*WEIGHT)[weight_size.d0][weight_size.d1][mul(weight_size.d2, weight_size.d3) + 1];
	//Output[oc][oh][ow],oc表示输出特征的通道数，oh表示输出图像的高度（行下标），ow表示输出图像的宽度（列下标）
	typedef short (*OUT)[conv_size.d1][conv_size.d2][conv_size.d3];
	IN in_array = (IN)(in + input_offset);
	WEIGHT weight_array = (WEIGHT)weight;
	OUT out_array = (OUT)(out + output_offset);

	//卷积
	for(int no=0;no<weight_size.d0;no++){		//输出特征图数量
		for(int ni=0;ni<rd_size.d1;ni++){		//输入特征图数量
			for(int y=0;y<conv_size.d2;y++){	//输出特征图尺寸
				int ybase = mul(y, stride) - pad;		//乘法外移，减少乘法的运算次数
				for(int x=0;x<conv_size.d3;x++){
					int xbase = mul(x, stride) - pad;	//乘法外移
					if(ni==0){
						(*out_array)[no][y][x] = (*weight_array)[no][0][0];
					}
					int unshifted = 0;	//
					for(int ky=0;ky<WEIGHT_SIZE_D2;ky++){
						int ih = ky + ybase;
						int yoffset = mul(ky, weight_size.d3);//乘法外移，且由于二维数组的存储，应当按照y*每行元素个数来表示ky
						for(int kx=0;kx<WEIGHT_SIZE_D3;kx++){
							int iw = kx + xbase;
							if(iw>=0 && iw<input_fm_w && ih>=0 && ih<input_fm_h){	//条件判断，当前坐标是否在输入范围内
								unshifted += mul((*in_array)[ni][ih][iw],(*weight_array)[no][ni][yoffset + kx + 1]);
							}
						}
					}
					(*out_array)[no][y][x] += unshifted >> FRAC_BIT;	//用整数结果来计算最终的浮点数结果
				}
			}
		}
	}
}

void pooling()
{
	short *in = (short *)addr.wr_addr;	// 输入图像数组首地址指针
	short *out = (short *)addr.wr_addr;	// 输出特征图数组首地址指针

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	//TODO: Please add your implementation here

	/*Max Pooling算法，对N×N的特征图取最大值，作为采样后的样本值*/

	/*定义输入数组、输出数组*/
	//池化中的输入数组即为卷积的输出数组Output[oc][oh][ow]
	typedef short (*IN_TYPE)[conv_size.d1][conv_size.d2][conv_size.d3];
	//池化的输出数组是取完最大值并更新后的结果
	typedef short (*OUT_TYPE)[conv_size.d1][pool_out_h][pool_out_w];
	IN_TYPE in_array = (IN_TYPE)(in + input_offset);
	OUT_TYPE out_array = (OUT_TYPE)(out + output_offset);
	
	//类似卷积的for循环
	for(int no=0;no<conv_size.d1;no++){				//输出特征图数量
		for(int y=0;y<pool_out_h;y++){				//输出特征图的行数
			int ybase = mul(y, stride) - pad;		//乘法外移
			for(int x=0;x<pool_out_w;x++){			//输出特征图的列数
				int xbase = mul(x, stride) - pad;	//乘法外移
				short max = 0x8000;					//short类型表示的最小值
				for(int ky=0;ky<KERN_ATTR_POOL_KERN_SIZE;ky++){
					int ih = ky + ybase;
					for(int kx=0;kx<KERN_ATTR_POOL_KERN_SIZE;kx++){
						int iw = kx + xbase;
						if(iw>=0 && iw<input_fm_w && ih>=0 && ih<input_fm_h){	//条件判断
							if(max < (*in_array)[no][ih][iw]){					//大小判断
								max = (*in_array)[no][ih][iw];
							}
						}
					}
				}
				(*out_array)[no][y][x] = max;		//将其中的元素值替换为最大值
			}
		}
	}
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here
	// 启动硬件加速器
	*gpio_start = 1;
	// 等待硬件加速器完成
	while (*gpio_done == 0);
	// 硬件加速器完成
	*gpio_start = 0;
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{
	Result res;
	res.msec = 0;
	res.InstructionCount = 0;
	bench_prepare(&res);

#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif

	int result = comparing();
	bench_done(&res);
	printf("Cycle Count:      	%u\n", res.msec);
    printf("Instruction Count:  %u\n", res.InstructionCount);
	printf("benchmark finished\n");

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}

	return 0;
}
