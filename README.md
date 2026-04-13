# DMMotor

## 1. 模块作用
达妙电机驱动模块。封装 CAN 协议并实现 Motor 抽象接口。

## 2. 主要函数说明
1. Decode: 解析电机反馈帧。
2. MITControl / PosControl / SpdControl: 三类控制模式下发。
3. Update / GetFeedback: 刷新并读取反馈。
4. Enable / Disable / Relax / Control: 标准电机控制入口。

## 3. 接入步骤
1. 添加模块并配置 model、reverse、can_id、can_bus_name。
2. 上电后先检查反馈在线，再启用控制。
3. 上层业务通过 Motor 接口调用本模块。

标准命令流程：
    xrobot_add_mod DMMotor --instance-id dmmotor
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: DMMotor
entry_header: Modules/DMMotor/DMMotor.hpp
constructor_args:
  - param:
      model: DMMotor::Model::MOTOR_DM4310
      reverse: false
      can_id: 1
      can_bus_name: can1
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
[]

## 6. 代码入口
Modules/DMMotor/DMMotor.hpp
