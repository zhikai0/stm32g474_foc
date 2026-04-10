#include "board_support.hpp"  // HAL 外设头文件 + print/sleep/print_memory_usage/sys_init
#include "utils.hpp"             /* 数学常量,our_arm_sin_f32, our_arm_cos_f32, fast_atan2 */
#include "timer.hpp"             /* TaskTimer, LogicTimer, MEASURE_TIME */
#include "low_level.hpp"
#include "phase_control_law.hpp"
#include "test_m.hpp"

/* TaskTimer::enabled */
bool TaskTimer::enabled = false;

extern "C" int main(void)
{

  board_init();
  // MX_CORDIC_Init(); // 硬件三角函数
  sleep(5);
  can_start();
  print_memory_usage();
  timing_init(); /* 启动 DWT CYCCNT */

  TaskTimer sin_timer;
  TaskTimer::enabled = true;   /* 打开计时开关 */
  volatile float acc = 0;
  float test_x = -2.4f;
  int iters = 10000;
  MEASURE_TIME(sin_timer) {
    for (int i = 0; i < iters; i++) {acc += our_arm_sin_f32(test_x); test_x += 0.0001f; }
  }
  TaskTimer::enabled = false;
  printf("cordic 10000x: total=%lu cyc  avg=%lu cyc / %.3f us\n",
    (unsigned long)sin_timer.last_cycles,
    (unsigned long)(sin_timer.last_cycles / iters),
    sin_timer.last_us() / iters);


  DemoController demo;
  PhaseControlLaw<3>* ctrl = &demo;  /* 多态：通过父类指针操作 */

  ctrl->reset();

  /* 模拟 ADC 采样完成（三相电流 + 母线电压）*/
  std::array<float, 3> currents = {1.0f, -0.5f, -0.5f};  /* Ia+Ib+Ic=0 */
  ctrl->on_measurement(24.0f, currents, 0);

  /* 模拟 PWM 更新时刻，获取三相占空比 */
  float pwm[3] = {0};
  std::optional<float> ibus;
  int err = ctrl->get_output(0, pwm, &ibus);

  print("DemoController: err=%d  tA=%.3f  tB=%.3f  tC=%.3f",err, pwm[0], pwm[1], pwm[2]);
  /* 期望输出：tA<tB<tC，三个值均在[0,1]内，err=0 */
  spi_test_once();
  for(;;)
  {  
    EXECUTE_EVERY_N_MS(100,can_test();); // can测试，100ms=10hz
    // EXECUTE_EVERY_N_MS(10,spi_test_once(););   
    EXECUTE_EVERY_N_MS(100,adc2_pa7_thermistor_test_once(););   // 热敏电阻
  }
  for(;;);
}


