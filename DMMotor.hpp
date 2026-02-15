#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - param:
      model: DMMotor::Model::MOTOR_DM4310
      reverse: false
      can_id: 1
      can_bus_name: can1
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <math.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Motor.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "thread.hpp"

#define DM4310_PMAX (12.5f)
#define DM4310_VMAX (30.0f)
#define DM4310_TMAX (10.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)

#define DM8009_PMAX (12.5f)
#define DM8009_VMAX (45.0f)
#define DM8009_TMAX (54.0f)
#define DM8009_KP_MIN (0.0f)
#define DM8009_KP_MAX (500.0f)
#define DM8009_KD_MIN (0.0f)
#define DM8009_KD_MAX (5.0f)

class DMMotor : public LibXR::Application, public Motor {
 public:
  /*电机型号*/
  enum class Model : uint8_t {
    MOTOR_NONE = 0,
    MOTOR_DM4310,
    MOTOR_DM8009,
  };

  /*电机参数*/
  struct Param {
    Model model;
    bool reverse;
    uint16_t can_id;
    const char* can_bus_name;
  };

  /*量程*/
  struct LSB {
    float P_MAX;
    float V_MAX;
    float T_MAX;
    float KD_MIN;
    float KD_MAX;
    float KP_MIN;
    float KP_MAX;
  };

  /**
   * @brief DMMotor 的构造函数
   * @param hw
   * @param app
   * @param param 电机参数 (电机型号 是否反转 CANID CanBusName)
   */
  DMMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          const Param& param)
      : param_(param),
        feedback_{},
        can_(hw.template FindOrExit<LibXR::CAN>({param_.can_bus_name})) {
    UNUSED(app);

    switch (param_.model) {
      case Model::MOTOR_DM4310:
        lsb_.P_MAX = DM4310_PMAX;
        lsb_.V_MAX = DM4310_VMAX;
        lsb_.T_MAX = DM4310_TMAX;
        lsb_.KD_MIN = DM4310_KD_MIN;
        lsb_.KD_MAX = DM4310_KD_MAX;
        lsb_.KP_MIN = DM4310_KP_MIN;
        lsb_.KP_MAX = DM4310_KP_MAX;
        break;
      case Model::MOTOR_DM8009:
        lsb_.P_MAX = DM8009_PMAX;
        lsb_.V_MAX = DM8009_VMAX;
        lsb_.T_MAX = DM8009_TMAX;
        lsb_.KD_MIN = DM8009_KD_MIN;
        lsb_.KD_MAX = DM8009_KD_MAX;
        lsb_.KP_MIN = DM8009_KP_MIN;
        lsb_.KP_MAX = DM8009_KP_MAX;
        break;
      case Model::MOTOR_NONE:
        lsb_.P_MAX = 0;
        lsb_.V_MAX = 0;
        lsb_.T_MAX = 0;
        lsb_.KD_MIN = 0;
        lsb_.KD_MAX = 0;
        lsb_.KP_MIN = 0;
        lsb_.KP_MAX = 0;
        break;
    }
    /* 强制规定达妙电机反馈id=自身id+10 */
    uint16_t feedback_id_to_register = 0x10 + param_.can_id;

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, DMMotor* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);
    /* 注册can */
    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, feedback_id_to_register,
                   feedback_id_to_register);
  }

  /*使能*/
  void Enable() override {
    /*使能can包*/
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /*失能*/
  void Disable() override {
    /*失能can包*/
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void Relax() override {}

  ErrorCode Update() override {
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == ErrorCode::OK) {
      this->Decode(pack);
      last_online_time_ = LibXR::Timebase::GetMicroseconds();
    }
    return ErrorCode::OK;
  }

  const Feedback& GetFeedback() override { return feedback_; }

  void Control(const MotorCmd& cmd) override {
    switch (cmd.mode) {
      case ControlMode::MODE_POSITION:
        PosControl(cmd.position, cmd.velocity);
        break;
      case ControlMode::MODE_VELOCITY:
        SpdControl(cmd.velocity);
        break;
      case ControlMode::MODE_TORQUE:
        MITControl(0.0f, 0.0f, 0.0f, 0.0f, cmd.torque);
        break;
      case ControlMode::MODE_MIT:
        MITControl(cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
        break;
      default:
        break;
    }
  }

  /*重置错误状态*/
  void ClearError() override {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /*将当前位置设成零点*/
  void SaveZeroPoint() override {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void OnMonitor() override {}

 private:
  uint64_t last_online_time_; /* 方便查看电机是否在线 */
  Param param_;
  LSB lsb_;
  Motor::Feedback feedback_;
  LibXR::CAN* can_;
  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};

  /*---------------------工具函数---------------------------------------------*/
  int FloatToUint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return static_cast<int>((x - offset) *
                            (static_cast<float>((1 << bits) - 1)) / span);
  }

  float UintToFloat(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (static_cast<float>(x_int)) * span /
               (static_cast<float>((1 << bits) - 1)) +
           offset;
  }

  /**
   * @brief CAN 接收回调的静态包装函数
   * @details
   * 将接收到的CAN数据包推入无锁队列中，供后续处理。如果队列已满，则丢弃最旧的数据包。
   * @param in_isr 指示是否在中断服务程序中调用
   * @param self 用户提供的参数，这里是 RMMotorContainer 实例的指针
   * @param pack 接收到的 CAN 数据包
   */
  static void RxCallback(bool in_isr, DMMotor* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }

  void Decode(LibXR::CAN::ClassicPack& pack) {
    feedback_.error_id = (pack.data[0]) & 0x0F;
    feedback_.state = (pack.data[0]) >> 4;
    feedback_.position =
        UintToFloat(static_cast<int16_t>((pack.data[1] << 8) | pack.data[2]),
                    -lsb_.P_MAX, lsb_.P_MAX, 16);
    feedback_.abs_angle = LibXR::CycleValue<float>(feedback_.position);
    feedback_.velocity = feedback_.omega * 60.0f / static_cast<float>(M_2PI);
    feedback_.omega = UintToFloat(
        static_cast<int16_t>((pack.data[3] << 4) | (pack.data[4] >> 4)),
        -lsb_.V_MAX, lsb_.V_MAX, 12);
    feedback_.torque = UintToFloat(
        static_cast<int16_t>(((pack.data[4] & 0xF) << 8) | pack.data[5]),
        -lsb_.T_MAX, lsb_.T_MAX, 12);
    feedback_.temp = static_cast<float>(
        pack.data[6] > pack.data[7] ? pack.data[6] : pack.data[7]);
  }

  void MITControl(float pos, float vel, float kp, float kd, float tor) {
    if (this->feedback_.temp > 85.0f) {
      Disable();
      XR_LOG_WARN("motor %d high temperature detected", param_.can_id);
    }
    pos = std::clamp(pos, -lsb_.P_MAX, lsb_.P_MAX);
    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);
    tor = std::clamp(tor, -lsb_.T_MAX, lsb_.T_MAX);

    float send_pos = param_.reverse ? -pos : pos;
    float send_vel = param_.reverse ? -vel : vel;
    float send_tor = param_.reverse ? -tor : tor;

    uint16_t pos_u = FloatToUint(send_pos, -lsb_.P_MAX, lsb_.P_MAX, 16);
    uint16_t vel_u = FloatToUint(send_vel, -lsb_.V_MAX, lsb_.V_MAX, 12);
    uint16_t tor_u = FloatToUint(send_tor, -lsb_.T_MAX, lsb_.T_MAX, 12);
    uint16_t kp_u = FloatToUint(kp, lsb_.KP_MIN, lsb_.KP_MAX, 12);
    uint16_t kd_u = FloatToUint(kd, lsb_.KD_MIN, lsb_.KD_MAX, 12);

    uint8_t data[8];
    data[0] = (pos_u >> 8) & 0xFF;
    data[1] = pos_u & 0xFF;
    data[2] = (vel_u >> 4) & 0xFF;
    data[3] = ((vel_u & 0xF) << 4) | ((kp_u >> 8) & 0xF);
    data[4] = kp_u & 0xFF;
    data[5] = (kd_u >> 4) & 0xFF;
    data[6] = ((kd_u & 0xF) << 4) | ((tor_u >> 8) & 0xF);
    data[7] = tor_u & 0xFF;

    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void PosControl(float pos, float vel) {
    if (this->feedback_.temp > 85.0f) {
      XR_LOG_WARN("motor %d high temperature detected", param_.can_id);
      Disable();
    }
    pos = std::clamp(pos, -lsb_.P_MAX, lsb_.P_MAX);
    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);

    float send_pos = param_.reverse ? -pos : pos;
    float send_vel = param_.reverse ? -vel : vel;

    uint8_t data[8];
    uint8_t* pbuf = reinterpret_cast<uint8_t*>(&send_pos);
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&send_vel);

    for (int i = 0; i < 4; ++i) {
      data[i] = pbuf[i];
      data[i + 4] = vbuf[i];
    }

    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void SpdControl(float vel) {
    if (this->feedback_.temp > 85.0f) {
      Disable();
      XR_LOG_WARN("motor %d high temperature detected", param_.can_id);
    }

    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);
    float send_vel = param_.reverse ? -vel : vel;

    uint8_t data[4];
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&send_vel);

    for (int i = 0; i < 4; ++i) {
      data[i] = vbuf[i];
    }

    uint32_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;

    memcpy(tx_pack.data, data, 4);
    can_->AddMessage(tx_pack);
  }
};
