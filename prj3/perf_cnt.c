#include "perf_cnt.h"

unsigned long _uptime() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
  volatile unsigned long *cycle_cnt = (unsigned long*)cpu_perf_cnt_0;   //将第0个性能计数器转换为cycle_cnt的类型并赋值给cycle_cnt
  return *cycle_cnt;
}

unsigned long _upinst() {
  volatile unsigned long *inst_cnt = (unsigned long*)cpu_perf_cnt_1;    //将第1个性能计数器同样转换类型并赋值给inst_cnt
  return *inst_cnt;
}

void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  //调用函数，将性能计数器与对应的接口连接
  res->msec = _uptime();              
  res->InstructionCount = _upinst();
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  //在统计时，要获取当前计数器的值，则应当用总值减去初值
  res->msec = _uptime() - res->msec;
  res->InstructionCount = _upinst() - res->InstructionCount;
}

