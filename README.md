# DMMotor

达妙（DM）电机驱动模块。封装达妙 CAN 协议并实现 `Motor` 抽象接口，支持 DM4310 与 DM8009。

- 构造时在 CAN 总线上注册标准帧回调，接收 ID 为 `0x10 + can_id` 的反馈帧（固定映射）。
  接收队列深度为 1，只保留最新一帧。
- `Update()` 取出并解码反馈：`position`（rad，按型号量程 ±P_MAX）、`omega`（rad/s）、
  `velocity`（rpm，由 `omega` 换算）、`torque`（N·m）、`temp`（取帧内两路温度的较大值）、
  `error_id` / `state`（帧首字节低 / 高 4 位）；`abs_angle` 取 `position`。
- 所有控制帧都以 `can_id` 为 ID 发送。`Control()` 的模式映射：

  | `MotorCmd::mode` | 行为 |
  | --- | --- |
  | `MODE_MIT` | MIT 帧：`position`、`velocity`、`kp`、`kd`、`torque` |
  | `MODE_TORQUE` | MIT 帧，只给 `torque`（位置、速度、kp、kd 为 0） |
  | `MODE_POSITION` | 位置速度帧：`position`、`velocity`（float） |
  | `MODE_VELOCITY` | 速度帧：`velocity`（float） |
  | `MODE_CURRENT` | 不处理 |

  下发值先按型号量程限幅；`reduction_ratio` 不参与计算。
- `Enable()` / `Disable()` / `ClearError()` / `SaveZeroPoint()` 分别发送达妙的使能（`0xFC`）、
  失能（`0xFD`）、清错（`0xFB`）、保存零点（`0xFE`）特殊帧；`Relax()` 等同 `Disable()`。
- `reverse = true` 时反馈的位置、速度、角速度、扭矩取反，下发的位置、速度、扭矩也取反。
- 温度保护：MIT / 位置模式下反馈温度超过 90 ℃、速度模式下超过 85 ℃ 时发送失能帧并输出
  `XR_LOG_WARN`（随后本次控制帧仍会发出）。
- 额外公共接口：`MITControl(pos, vel, kp, kd, tor)`、`GetAngle()`、`GetTor()`、`GetOmega()`。

型号量程（代码中的宏）：

| 型号 | P_MAX (rad) | V_MAX (rad/s) | T_MAX (N·m) | KP | KD |
| --- | --- | --- | --- | --- | --- |
| `MOTOR_DM4310` | 6.283185 | 30 | 10 | 0–500 | 0–5 |
| `MOTOR_DM8009` | 12.56637 | 45 | 54 | 0–500 | 0–5 |

## 依赖

- `QDU-Robomaster/Motor`：本模块实现的电机抽象接口（库）。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
DMMotor(LibXR::CAN& can_bus,
        const Param& param = {
            .model = DMMotor::Model::MOTOR_DM4310, .reverse = false, .can_id = 1});
```

依赖：

- `can_bus`：`LibXR::CAN`，电机所在的 CAN 总线。

配置（`Param`）：

- `model`：电机型号，`DMMotor::Model::MOTOR_DM4310` / `MOTOR_DM8009` / `MOTOR_NONE`，
  默认 `MOTOR_DM4310`（`MOTOR_NONE` 的量程全为 0）。
- `reverse`：是否反向，默认 `false`。
- `can_id`：电机控制 ID，默认 1；反馈 ID 固定为 `0x10 + can_id`。

## 使用

```sh
xrobot module add QDU-Robomaster/DMMotor
xrobot setup
xrobot instance add QDU-Robomaster/DMMotor
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把 `can_bus` 填为 BSP 中用 `XR_REGISTER` 注册的 CAN 对象名：

```yaml
modules:
  - module: QDU-Robomaster/DMMotor
    id: dmmotor_0
    args:
      - can_bus: can1
      - param:
          model: DMMotor::Model::MOTOR_DM4310
          reverse: 'false'
          can_id: '1'
```

BSP 侧：

```cpp
XR_REGISTER(can1, LibXR::CAN);
```

其他模块（如 `Gimbal`）的 `Motor&` 参数直接填写本实例的 id（此处为 `dmmotor_0`），
本实例须在它们之前列出。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/DMMotor`
（在 BSP 中）打印当前的构造函数。
