# ODrive FOC 学习计划

参考 ODrive 源码，逐模块手写复现，理解无刷电机 FOC 控制全链路。

---

## 已完成

- **SVM（空间矢量调制）**：`phase_control_law.cpp`
  - Clark/Park 逆变换 → SVM 占空比计算 → 三相输出
  - `fast_math`：`sin/cos` 查表加速

---

## 当前阶段：FOC 核心模块

**目标**：理解并实现 `foc.hpp / foc.cpp` 中的完整电流环。

### 学习步骤

1. **读懂 `foc.hpp` 接口**
   - `Foc` 类结构：成员变量（Id/Iq 目标、PI 状态、坐标系变量）
   - 与 `interfaces.hpp` 中 `ICurrentSensor`、`IPositionSensor` 的依赖关系

2. **实现 ADC 采样 → Clark 变换**
   - 三相电流 → αβ 坐标（`ia, ib, ic → i_alpha, i_beta`）

3. **实现 Park 变换**
   - αβ → dq 坐标（需要转子角 θ）

4. **实现 dq 轴 PI 控制器**
   - d 轴：控制励磁电流（通常目标 = 0）
   - q 轴：控制转矩电流
   - 参考 ODrive `current_control_` 结构中的积分限幅与前馈

5. **实现反Park变换 + 接入 SVM**
   - dq → αβ → SVM 占空比
   - 完整闭环：`Foc::update()` 串联以上所有步骤

6. **限幅与解耦（可选深入）**
   - 电压限幅圆
   - dq 轴交叉解耦项

---

## 后续模块（FOC 完成后）

| 顺序 | 模块 | 核心内容 |
|------|------|---------|
| 1 | **Motor** | 电机参数、相电阻/电感辨识 |
| 2 | **Axis** | 状态机：IDLE / CALIBRATION / CLOSED_LOOP |
| 3 | **Low Level** | TIM PWM、ADC 中断、DMA 触发时序 |

---

## 关键文件索引

```
motor_control/
├── include/
│   ├── foc.hpp          ← 当前重点
│   ├── interfaces.hpp   ← 传感器/执行器抽象接口
│   ├── component.hpp    ← 组件基类
│   ├── timer.hpp        ← PWM 定时器封装
│   └── utils.hpp        ← 通用工具
└── src/
    ├── phase_control_law.cpp  ← SVM 已完成
    ├── fast_math.c            ← sin/cos 查表
    ├── main.cpp               ← 调度入口
    └── foc.cpp                ← 待实现
```
