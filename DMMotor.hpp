#pragma once
// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 达妙（DM）电机 CAN 驱动模块，支持 DM4310 与 DM8009 / CAN driver Module for Damiao (DM) motors, supporting the DM4310 and DM8009
depends:
- id: QDU-Robomaster/Motor
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>

#include "Motor.hpp"
#include "can.hpp"
#include "latest_snapshot.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"

// DM4310 量程：位置 (rad)、速度 (rad/s)、力矩 (N·m)、KP、KD
// DM4310 ranges: position (rad), velocity (rad/s), torque (N·m), KP, KD
#define DM4310_PMAX (6.283185f)
#define DM4310_VMAX (30.0f)
#define DM4310_TMAX (10.0f)
#define DM4310_KP_MIN (0.0f)
#define DM4310_KP_MAX (500.0f)
#define DM4310_KD_MIN (0.0f)
#define DM4310_KD_MAX (5.0f)

// DM8009 量程：位置 (rad)、速度 (rad/s)、力矩 (N·m)、KP、KD
// DM8009 ranges: position (rad), velocity (rad/s), torque (N·m), KP, KD
#define DM8009_PMAX (12.56637f)
#define DM8009_VMAX (45.0f)
#define DM8009_TMAX (54.0f)
#define DM8009_KP_MIN (0.0f)
#define DM8009_KP_MAX (500.0f)
#define DM8009_KD_MIN (0.0f)
#define DM8009_KD_MAX (5.0f)

/**
 * @brief 达妙（DM）电机 CAN 驱动，实现 Motor 接口。
 *        CAN driver for Damiao (DM) motors, implementing the Motor interface.
 */
class DMMotor : public Motor
{
 public:
  /**
   * @brief 电机型号。
   *        Motor model.
   */
  enum class Model : uint8_t
  {
    MOTOR_NONE = 0,  ///< 未指定，各项量程为 0
                     ///< Unspecified, all ranges are 0
    MOTOR_DM4310,    ///< DM4310
    MOTOR_DM8009,    ///< DM8009
  };

  /**
   * @brief 电机配置参数。
   *        Motor configuration parameters.
   */
  struct Param
  {
    Model model;      ///< 电机型号
                      ///< Motor model
    bool reverse;     ///< 反向：反馈与下发的位置、速度、力矩取反
                      ///< Reverse: negates the position, velocity and torque of the
                      ///< feedback and of the commands
    uint16_t can_id;  ///< 电机控制 ID，反馈 ID 为 0x10 + can_id
                      ///< Motor control ID; the feedback ID is 0x10 + can_id
  };

  /**
   * @brief 型号量程。
   *        Model ranges.
   */
  struct LSB
  {
    float P_MAX;   ///< 位置量程 (rad)
                   ///< Position range (rad)
    float V_MAX;   ///< 速度量程 (rad/s)
                   ///< Velocity range (rad/s)
    float T_MAX;   ///< 力矩量程 (N·m)
                   ///< Torque range (N·m)
    float KD_MIN;  ///< KD 下限
                   ///< KD lower bound
    float KD_MAX;  ///< KD 上限
                   ///< KD upper bound
    float KP_MIN;  ///< KP 下限
                   ///< KP lower bound
    float KP_MAX;  ///< KP 上限
                   ///< KP upper bound
  };

  /**
   * @brief 构造 DMMotor，并在 CAN 总线上注册反馈帧接收回调。
   *        Construct DMMotor and register the feedback-frame receive callback on the CAN
   *        bus.
   *
   * @param can_bus 电机所在的 CAN 总线。
   *                CAN bus the motor is attached to.
   * @param param 电机配置参数。
   *              Motor configuration parameters.
   */
  DMMotor(LibXR::CAN& can_bus,
          const Param& param = {.model = DMMotor::Model::MOTOR_DM4310,
                                .reverse = false,
                                .can_id = 1})
      : param_(param), feedback_{}, can_(std::addressof(can_bus))
  {
    switch (param_.model)
    {
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
    // 反馈帧 ID 为 0x10 + can_id
    uint16_t feedback_id_to_register = 0x10 + param_.can_id;

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, DMMotor* self, const LibXR::CAN::ClassicPack& pack)
        { RxCallback(in_isr, self, pack); }, this);
    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, feedback_id_to_register,
                   feedback_id_to_register);
  }

  /**
   * @brief 发送使能帧 (0xFC)。
   *        Send the enable frame (0xFC).
   */
  void Enable() override
  {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /**
   * @brief 发送失能帧 (0xFD)。
   *        Send the disable frame (0xFD).
   */
  void Disable() override
  {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /**
   * @brief 松开电机，等同于 Disable()。
   *        Relax the motor; the same as Disable().
   */
  void Relax() override { Disable(); }

  /**
   * @brief 取出最新的反馈帧并解码到反馈数据。
   *        Take the latest feedback frame and decode it into the feedback data.
   *
   * @return 始终为 ErrorCode::OK。
   *         Always ErrorCode::OK.
   */
  LibXR::ErrorCode Update() override
  {
    LibXR::CAN::ClassicPack pack;
    if (feedback_frame_.LoadLatest(pack))
    {
      this->Decode(pack);
      last_online_time_ = LibXR::Timebase::GetMicroseconds();
    }
    return LibXR::ErrorCode::OK;
  }

  /**
   * @brief 获取最近一次解码的反馈。
   *        Get the most recently decoded feedback.
   *
   * @return 反馈数据的引用。
   *         Reference to the feedback data.
   */
  const Feedback& GetFeedback() override { return feedback_; }

  /**
   * @brief 按控制模式下发控制帧。
   *        Send a control frame according to the control mode.
   *
   * @param cmd 控制命令。MODE_MIT 使用 position、velocity、kp、kd、torque；MODE_TORQUE
   *            仅使用 torque；MODE_POSITION 使用 position、velocity；MODE_VELOCITY
   *            使用 velocity；其他模式忽略。
   *            Control command. MODE_MIT uses position, velocity, kp, kd and torque;
   *            MODE_TORQUE uses only torque; MODE_POSITION uses position and velocity;
   *            MODE_VELOCITY uses velocity; other modes are ignored.
   */
  void Control(const MotorCmd& cmd) override
  {
    switch (cmd.mode)
    {
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

  /**
   * @brief 发送清错帧 (0xFB)。
   *        Send the clear-error frame (0xFB).
   */
  void ClearError() override
  {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  /**
   * @brief 发送保存零点帧 (0xFE)，将当前位置设为零点。
   *        Send the save-zero-point frame (0xFE), setting the current position as the
   *        zero point.
   */
  void SaveZeroPoint() override
  {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

 private:
  uint64_t last_online_time_;
  Param param_;
  LSB lsb_;
  Motor::Feedback feedback_;
  LibXR::CAN* can_;
  LibXR::LatestSnapshot<LibXR::CAN::ClassicPack> feedback_frame_{LibXR::CAN::ClassicPack{}};

  int FloatToUint(float x, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    float offset = x_min;
    return static_cast<int>((x - offset) * (static_cast<float>((1 << bits) - 1)) / span);
  }

  float UintToFloat(int x_int, float x_min, float x_max, int bits)
  {
    float span = x_max - x_min;
    float offset = x_min;
    return (static_cast<float>(x_int)) * span / (static_cast<float>((1 << bits) - 1)) +
           offset;
  }

  /**
   * @brief CAN 接收回调：保存最新的反馈帧，覆盖尚未取走的旧帧。
   *        CAN receive callback: store the latest feedback frame, replacing an older
   *        frame that has not been taken.
   *
   * @param in_isr 是否在中断上下文中调用。
   *               Whether called from interrupt context.
   * @param self DMMotor 实例指针。
   *             Pointer to the DMMotor instance.
   * @param pack 接收到的 CAN 数据包。
   *             Received CAN packet.
   */
  static void RxCallback(bool in_isr, DMMotor* self, const LibXR::CAN::ClassicPack& pack)
  {
    UNUSED(in_isr);
    self->feedback_frame_.Store(pack);
  }

  void Decode(LibXR::CAN::ClassicPack& pack)
  {
    feedback_.error_id = (pack.data[0]) & 0x0F;
    feedback_.state = (pack.data[0]) >> 4;
    feedback_.position =
        UintToFloat(static_cast<int16_t>((pack.data[1] << 8) | pack.data[2]), -lsb_.P_MAX,
                    lsb_.P_MAX, 16);

    feedback_.omega =
        UintToFloat(static_cast<int16_t>((pack.data[3] << 4) | (pack.data[4] >> 4)),
                    -lsb_.V_MAX, lsb_.V_MAX, 12);
    feedback_.velocity = feedback_.omega * 60.0f / static_cast<float>(LibXR::TWO_PI);
    feedback_.torque =
        UintToFloat(static_cast<int16_t>(((pack.data[4] & 0xF) << 8) | pack.data[5]),
                    -lsb_.T_MAX, lsb_.T_MAX, 12);
    feedback_.temp =
        static_cast<float>(pack.data[6] > pack.data[7] ? pack.data[6] : pack.data[7]);

    if (param_.reverse)
    {
      feedback_.position = -feedback_.position;
      feedback_.velocity = -feedback_.velocity;
      feedback_.torque = -feedback_.torque;
      feedback_.omega = -feedback_.omega;
    }
    feedback_.abs_angle = feedback_.position;
  }

 public:
  /**
   * @brief 获取反馈位置。
   *        Get the feedback position.
   *
   * @return 位置，单位 rad。
   *         Position in rad.
   */
  float GetAngle() const { return feedback_.position; }

  /**
   * @brief 获取反馈力矩。
   *        Get the feedback torque.
   *
   * @return 力矩，单位 N·m。
   *         Torque in N·m.
   */
  float GetTor() const { return feedback_.torque; }

  /**
   * @brief 获取反馈角速度。
   *        Get the feedback angular velocity.
   *
   * @return 角速度，单位 rad/s。
   *         Angular velocity in rad/s.
   */
  float GetOmega() const { return feedback_.omega; }

  /**
   * @brief 发送 MIT 控制帧。
   *        Send an MIT control frame.
   *
   * 反馈温度超过 90 ℃ 时先发送失能帧并输出警告日志，随后仍发送本帧。pos、vel、tor
   * 先按型号量程限幅。
   * When the feedback temperature exceeds 90 ℃, the disable frame is sent and a warning
   * is logged first, and this frame is then sent as usual. pos, vel and tor are first
   * clamped to the model range.
   *
   * @param pos 目标位置，单位 rad。
   *            Target position in rad.
   * @param vel 目标速度，单位 rad/s。
   *            Target velocity in rad/s.
   * @param kp 位置刚度。
   *           Position stiffness.
   * @param kd 速度阻尼。
   *           Velocity damping.
   * @param tor 前馈力矩，单位 N·m。
   *            Feedforward torque in N·m.
   */
  void MITControl(float pos, float vel, float kp, float kd, float tor)
  {
    if (this->feedback_.temp > 90.0f)
    {
      Disable();
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.can_id));
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
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

 private:
  void PosControl(float pos, float vel)
  {
    if (this->feedback_.temp > 90.0f)
    {
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.can_id));
      Disable();
    }
    pos = std::clamp(pos, -lsb_.P_MAX, lsb_.P_MAX);
    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);

    float send_pos = param_.reverse ? -pos : pos;
    float send_vel = param_.reverse ? -vel : vel;

    uint8_t data[8];
    uint8_t* pbuf = reinterpret_cast<uint8_t*>(&send_pos);
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&send_vel);

    for (int i = 0; i < 4; ++i)
    {
      data[i] = pbuf[i];
      data[i + 4] = vbuf[i];
    }

    uint16_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 8);
    can_->AddMessage(tx_pack);
  }

  void SpdControl(float vel)
  {
    if (this->feedback_.temp > 85.0f)
    {
      Disable();
      XR_LOG_WARN("motor %u high temperature detected",
                  static_cast<unsigned>(param_.can_id));
    }

    vel = std::clamp(vel, -lsb_.V_MAX, lsb_.V_MAX);
    float send_vel = param_.reverse ? -vel : vel;

    uint8_t data[4];
    uint8_t* vbuf = reinterpret_cast<uint8_t*>(&send_vel);

    for (int i = 0; i < 4; ++i)
    {
      data[i] = vbuf[i];
    }

    uint32_t id = param_.can_id;
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = id;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = 8;
    memcpy(tx_pack.data, data, 4);
    can_->AddMessage(tx_pack);
  }
};
