#include <rm_serial_driver/packet.hpp>

#include <algorithm>
#include <cstring>

namespace rm::serial
{
namespace
{
constexpr std::size_t kPacketArmOffset = offsetof(ReceivePacket, arm_header);
constexpr std::size_t kReceiveBytesBeforeOptionalReserved =
  offsetof(ReceivePacket, arm_feedback_reserved) - kPacketArmOffset;
constexpr std::size_t kReceiveArmExtensionMinSize =
  kReceiveBytesBeforeOptionalReserved + sizeof(ReceivePacket::arm_end_frame);
constexpr std::size_t kReceiveArmExtensionFullSize =
  kReceiveBytesBeforeOptionalReserved + sizeof(ReceivePacket::arm_feedback_reserved) +
  sizeof(ReceivePacket::arm_end_frame);
}

ReceivePacket FromVector(const std::vector<uint8_t>& data)
{
  return FromVectorWithArmOffset(data, kLegacyReceiveFrameSize);
}

ReceivePacket FromVectorWithArmOffset(const std::vector<uint8_t>& data, std::size_t arm_offset)
{
  ReceivePacket packet;
  if (data.empty())
  {
    return packet;
  }

  packet.header = data[0];
  if (arm_offset > 0 && data.size() >= arm_offset)
  {
    packet.end_frame = data[arm_offset - 1];
  }
  else if (data.size() >= kLegacyReceiveFrameSize)
  {
    packet.end_frame = data[1];
  }

  if (data.size() > arm_offset)
  {
    const std::size_t bytes_to_copy = std::min(
      data.size() - arm_offset,
      kReceiveBytesBeforeOptionalReserved);
    std::memcpy(
      reinterpret_cast<uint8_t *>(&packet) + kPacketArmOffset,
      data.data() + arm_offset,
      bytes_to_copy);

    const std::size_t arm_payload_size = data.size() - arm_offset;
    if (arm_payload_size >= kReceiveArmExtensionFullSize)
    {
      packet.arm_feedback_reserved =
        data[arm_offset + kReceiveBytesBeforeOptionalReserved];
      packet.arm_end_frame =
        data[arm_offset + kReceiveBytesBeforeOptionalReserved + 1];
    }
    else if (arm_payload_size >= kReceiveArmExtensionMinSize)
    {
      packet.arm_feedback_reserved = 0;
      packet.arm_end_frame =
        data[arm_offset + kReceiveBytesBeforeOptionalReserved];
    }
  }
  return packet;
}

std::vector<uint8_t> ToVector(const SendPacket& data, bool include_arm_extension)
{
  std::vector<uint8_t> packet(SendPacketSize(include_arm_extension));
  std::memcpy(packet.data(), &data, packet.size());
  return packet;
}

std::size_t SendPacketSize(bool include_arm_extension)
{
  return include_arm_extension ? sizeof(SendPacket) : LegacySendPacketSize();
}

std::size_t LegacySendPacketSize()
{
  return sizeof(uint8_t) + sizeof(NavPacket) + sizeof(uint8_t);
}

std::size_t ReceiveArmExtensionOffset(const std::vector<uint8_t>& data, std::size_t size)
{
  if (size >= kLegacyReceiveFrameSize &&
      data[0] == kReceiveHeader &&
      data[1] == kLegacyEndFrame)
  {
    return kLegacyReceiveFrameSize;
  }
  if (size >= kStatusReceiveFrameSize &&
      data[0] == kReceiveHeader &&
      data[2] == kLegacyEndFrame)
  {
    return kStatusReceiveFrameSize;
  }
  return 0;
}

std::size_t ReceivePacketSize(std::size_t arm_offset)
{
  return arm_offset + kReceiveArmExtensionMinSize;
}

bool HasValidArmReceiveExtension(
  const ReceivePacket& packet,
  std::size_t size,
  std::size_t arm_offset)
{
  return size >= ReceivePacketSize(arm_offset) &&
         packet.header == kReceiveHeader &&
         packet.end_frame == kLegacyEndFrame &&
         packet.arm_header == kArmExtensionHeader &&
         packet.arm_end_frame == kArmExtensionEndFrame &&
         packet.arm_state_valid != 0;
}
} 
