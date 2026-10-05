# DMMotor

达妙（DM）电机 CAN 驱动模块，支持 DM4310 与 DM8009 / CAN driver Module for Damiao (DM) motors, supporting the DM4310 and DM8009

## 1. 模块作用 / Purpose

DMMotor 封装达妙 CAN 协议并实现 `Motor` 抽象接口。构造时在 CAN 总线上注册一个标准帧回调，接收 ID 为 `0x10 + can_id` 的反馈帧；反馈帧存入 LibXR 的 `LatestSnapshot`，只保留最新一帧。

`Update()` 取出并解码反馈：`position`（rad）、`omega`（rad/s）、`velocity`（rpm，由 `omega` 换算）、`torque`（N·m）、`temp`（帧内两路温度的较大值）、`error_id`（帧首字节低 4 位）与 `state`（帧首字节高 4 位）；`abs_angle` 取 `position`。`reverse = true` 时，反馈的 `position`、`velocity`、`omega`、`torque` 取反，下发的位置、速度与力矩也取反。

所有控制帧以 `can_id` 为 ID 发送，`Control()` 按 `MotorCmd::mode` 处理：

| `MotorCmd::mode` | 行为 |
| --- | --- |
| `MODE_MIT` | MIT 帧，使用 `position`、`velocity`、`kp`、`kd`、`torque` |
| `MODE_TORQUE` | MIT 帧，仅 `torque` 有值，位置、速度、`kp`、`kd` 为 0 |
| `MODE_POSITION` | 位置速度帧，使用 `position`、`velocity`（float） |
| `MODE_VELOCITY` | 速度帧，使用 `velocity`（float） |
| `MODE_CURRENT` | 忽略 |

下发的位置、速度与力矩先按型号量程限幅，`reduction_ratio` 不参与计算。`Enable()`、`Disable()`、`ClearError()`、`SaveZeroPoint()` 分别发送达妙的使能（`0xFC`）、失能（`0xFD`）、清错（`0xFB`）与保存零点（`0xFE`）特殊帧，`Relax()` 与 `Disable()` 相同。

MIT 与位置模式下，反馈温度超过 90 ℃ 时发送失能帧并输出 `XR_LOG_WARN`；速度模式的阈值为 85 ℃。该次控制帧随后照常发送。

除 `Motor` 接口外，公共接口还有 `MITControl(pos, vel, kp, kd, tor)`、`GetAngle()`、`GetTor()` 与 `GetOmega()`。

DMMotor implements the `Motor` interface on top of the Damiao CAN protocol. Upon construction it registers a standard-frame callback on the CAN bus that receives feedback frames with ID `0x10 + can_id`. Feedback frames are stored in LibXR's `LatestSnapshot`, which keeps only the latest frame.

`Update()` pops and decodes the feedback: `position` (rad), `omega` (rad/s), `velocity` (rpm, converted from `omega`), `torque` (N·m), `temp` (the larger of the two temperatures in the frame), `error_id` (low 4 bits of the first byte) and `state` (high 4 bits of the first byte); `abs_angle` takes the value of `position`. With `reverse = true`, the feedback `position`, `velocity`, `omega` and `torque` are negated, as are the position, velocity and torque that are sent.

All control frames are sent with `can_id` as the ID, and `Control()` handles `MotorCmd::mode` as follows:

| `MotorCmd::mode` | Behavior |
| --- | --- |
| `MODE_MIT` | MIT frame using `position`, `velocity`, `kp`, `kd` and `torque` |
| `MODE_TORQUE` | MIT frame with only `torque` set; position, velocity, `kp` and `kd` are 0 |
| `MODE_POSITION` | Position-velocity frame using `position` and `velocity` (float) |
| `MODE_VELOCITY` | Velocity frame using `velocity` (float) |
| `MODE_CURRENT` | Ignored |

The position, velocity and torque that are sent are first clamped to the model range, and `reduction_ratio` is not used in the calculation. `Enable()`, `Disable()`, `ClearError()` and `SaveZeroPoint()` send the Damiao special frames for enable (`0xFC`), disable (`0xFD`), clear error (`0xFB`) and save zero point (`0xFE`) respectively; `Relax()` is the same as `Disable()`.

In MIT and position modes, a feedback temperature above 90 ℃ makes the Module send the disable frame and emit `XR_LOG_WARN`; the threshold in velocity mode is 85 ℃. The control frame of that call is then sent as usual.

Besides the `Motor` interface, the public interface includes `MITControl(pos, vel, kp, kd, tor)`, `GetAngle()`, `GetTor()` and `GetOmega()`.

## 2. 型号量程 / Model Ranges

`Param::model` 选择量程，对应源码中的宏：

| 型号 | P_MAX (rad) | V_MAX (rad/s) | T_MAX (N·m) | KP | KD |
| --- | --- | --- | --- | --- | --- |
| `MOTOR_DM4310` | 6.283185 | 30 | 10 | 0 – 500 | 0 – 5 |
| `MOTOR_DM8009` | 12.56637 | 45 | 54 | 0 – 500 | 0 – 5 |

`MOTOR_NONE` 的各项量程均为 0。

`Param::model` selects the range, which corresponds to the macros in the source:

| Model | P_MAX (rad) | V_MAX (rad/s) | T_MAX (N·m) | KP | KD |
| --- | --- | --- | --- | --- | --- |
| `MOTOR_DM4310` | 6.283185 | 30 | 10 | 0 – 500 | 0 – 5 |
| `MOTOR_DM8009` | 12.56637 | 45 | 54 | 0 – 500 | 0 – 5 |

All ranges of `MOTOR_NONE` are 0.

## 3. 构造接口 / Constructor

```cpp
DMMotor(LibXR::CAN& can_bus,
        const Param& param = {.model = DMMotor::Model::MOTOR_DM4310,
                              .reverse = false,
                              .can_id = 1});
```

依赖：

- `can_bus`：`LibXR::CAN`，电机所在的 CAN 总线，取自 BSP 的硬件注册（`XR_REGISTER`）。

配置参数（`Param`）：

- `model`：电机型号，`DMMotor::Model::MOTOR_DM4310`、`MOTOR_DM8009` 或 `MOTOR_NONE`，默认 `MOTOR_DM4310`。
- `reverse`：是否反向，默认 `false`。
- `can_id`：电机控制 ID，默认 1；反馈 ID 为 `0x10 + can_id`。

Dependencies:

- `can_bus`: the `LibXR::CAN` bus the motor is attached to, taken from the BSP's Registration (`XR_REGISTER`).

Configuration parameters (`Param`):

- `model`: motor model, `DMMotor::Model::MOTOR_DM4310`, `MOTOR_DM8009` or `MOTOR_NONE`, default `MOTOR_DM4310`.
- `reverse`: whether the direction is reversed, default `false`.
- `can_id`: motor control ID, default 1; the feedback ID is `0x10 + can_id`.

## 4. Topic

无 / None

## 5. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/DMMotor` 写入的实例，`can_bus` 填写为 BSP 中注册的 CAN 名称，`model`、`reverse`、`can_id` 按电机填写：

An instance written by `xrobot instance add QDU-Robomaster/DMMotor`, with `can_bus` set to a CAN name registered by the BSP, and `model`, `reverse` and `can_id` set according to the motor:

```yaml
modules:
  - module: QDU-Robomaster/DMMotor
    id: motor_pit
    args:
      - can_bus: can1
      - param:
          model: DMMotor::Model::MOTOR_DM4310
          reverse: true
          can_id: 1
```

其他模块（例如 `QDU-Robomaster/Gimbal`）的 `Motor&` 参数填写本实例的 id（此处为 `motor_pit`），本实例须在它们之前列出。

Other Modules (for example `QDU-Robomaster/Gimbal`) take the id of this instance (here `motor_pit`) for their `Motor&` parameter; this instance is listed before them.

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/Motor`：本模块实现的电机抽象接口。
- LibXR。

硬件：挂在 CAN 总线上的达妙电机（DM4310 或 DM8009），总线对象由 BSP 通过 `XR_REGISTER` 注册；电机的控制 ID 为 `can_id`，反馈帧 ID 为 `0x10 + can_id`。

Dependencies:

- `QDU-Robomaster/Motor`: the motor abstraction interface implemented by this Module.
- LibXR.

Hardware: a Damiao motor (DM4310 or DM8009) on a CAN bus whose object the BSP registers with `XR_REGISTER`; the motor control ID is `can_id` and the feedback frame ID is `0x10 + can_id`.
