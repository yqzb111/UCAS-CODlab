
#ifndef __PERF_CNT__
#define __PERF_CNT__

#ifdef __cplusplus
extern "C" {
#endif

#define cpu_perf_cnt_0 0x60010000
#define cpu_perf_cnt_1 0x60010008

typedef struct Result {
	int pass;
	unsigned long msec;
	unsigned long InstructionCount;
} Result;

void bench_prepare(Result *res);
void bench_done(Result *res);

#endif
