#ifndef _RM_SERIAL_DRIVER_PACKET_HPP
#define _RM_SERIAL_DRIVER_PACKET_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rm::serial
{
constexpr std::size_t kArmJointCount = 6;
constexpr uint8_t kSendHeader = 0xA5;
constexpr uint8_t kReceiveHeader = 0x5A;
constexpr uint8_t kLegacyEndFrame = 0x4A;
constexpr uint8_t kArmExtensionHeader = 0xB5;
constexpr uint8_t kArmExtensionEndFrame = 0x4B;
constexpr std::size_t kLegacyReceiveFrameSize = 2;
constexpr std::size_t kStatusReceiveFrameSize = 3;

struct ArmJointPacket
{
  float position[kArmJointCount] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
} __attribute__((packed));

struct ReceivePacket
{
  uint8_t header = kReceiveHeader;
  uint8_t end_frame = kLegacyEndFrame;
  uint8_t arm_header = kArmExtensionHeader;
  uint8_t arm_state_valid = 0;
  uint32_t arm_sequence = 0;
  ArmJointPacket arm;
  uint8_t arm_feedback_reserved = 0;
  uint8_t arm_end_frame = kArmExtensionEndFrame;
} __attribute__((packed));

struct NavPacket
{
  float linear_x = 0.0;
  float linear_y = 0.0;
  float angular_z = 0.0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = kSendHeader;
  NavPacket nav;
  uint8_t end_frame = kLegacyEndFrame;
  uint8_t arm_header = kArmExtensionHeader;
  uint8_t arm_command_enabled = 0;
  uint32_t arm_sequence = 0;
  ArmJointPacket arm;
  // Two-state gripper command: 1=closed, -1=open.
  int8_t gripper_command = 1;
  uint8_t arm_end_frame = kArmExtensionEndFrame;
} __attribute__((packed));

ReceivePacket FromVector(const std::vector<uint8_t>& data);
ReceivePacket FromVectorWithArmOffset(const std::vector<uint8_t>& data, std::size_t arm_offset);
std::vector<uint8_t> ToVector(const SendPacket& data, bool include_arm_extension = true);
std::size_t SendPacketSize(bool include_arm_extension);
std::size_t LegacySendPacketSize();
std::size_t ReceiveArmExtensionOffset(const std::vector<uint8_t>& data, std::size_t size);
std::size_t ReceivePacketSize(std::size_t arm_offset);
bool HasValidArmReceiveExtension(
  const ReceivePacket& packet,
  std::size_t size,
  std::size_t arm_offset = kLegacyReceiveFrameSize);
}

#endif
