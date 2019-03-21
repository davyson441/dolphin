// Copyright 2019 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/WiimoteEmu/MotionPlus.h"

#include <zlib.h>

#include "Common/BitUtils.h"
#include "Common/ChunkFile.h"
#include "Common/Logging/Log.h"
#include "Common/MathUtil.h"
#include "Common/MsgHandler.h"
#include "Common/Swap.h"

#include "Core/HW/Wiimote.h"
#include "Core/HW/WiimoteEmu/Dynamics.h"

namespace WiimoteEmu
{
MotionPlus::MotionPlus() : Extension("MotionPlus")
{
}

void MotionPlus::Reset()
{
  m_reg_data = {};

  m_progress_timer = {};

  // Seeing as we allow disconnection of the M+, we'll say we're not integrated.
  // (0x00 or 0x01)
  constexpr u8 IS_INTEGRATED = 0x00;

  // FYI: This ID changes on activation/deactivation
  constexpr std::array<u8, 6> initial_id = {IS_INTEGRATED, 0x00, 0xA6, 0x20, 0x00, 0x05};
  m_reg_data.ext_identifier = initial_id;

  // Build calibration data.

  // Matching signedness of my real Wiimote+.
  // This also results in all values following the "right-hand rule".
  constexpr u16 YAW_SCALE = CALIBRATION_ZERO - CALIBRATION_SCALE_OFFSET;
  constexpr u16 ROLL_SCALE = CALIBRATION_ZERO + CALIBRATION_SCALE_OFFSET;
  constexpr u16 PITCH_SCALE = CALIBRATION_ZERO - CALIBRATION_SCALE_OFFSET;

#pragma pack(push, 1)
  struct CalibrationBlock
  {
    u16 yaw_zero = Common::swap16(CALIBRATION_ZERO);
    u16 roll_zero = Common::swap16(CALIBRATION_ZERO);
    u16 pitch_zero = Common::swap16(CALIBRATION_ZERO);
    u16 yaw_scale = Common::swap16(YAW_SCALE);
    u16 roll_scale = Common::swap16(ROLL_SCALE);
    u16 pitch_scale = Common::swap16(PITCH_SCALE);
    u8 degrees_div_6;
  };

  struct CalibrationData
  {
    CalibrationBlock fast;
    u8 uid_1;
    Common::BigEndianValue<u16> crc32_msb;
    CalibrationBlock slow;
    u8 uid_2;
    Common::BigEndianValue<u16> crc32_lsb;
  };
#pragma pack(pop)

  static_assert(sizeof(CalibrationData) == 0x20, "Bad size.");

  static_assert(CALIBRATION_FAST_SCALE_DEGREES % 6 == 0, "Value should be divisible by 6.");
  static_assert(CALIBRATION_SLOW_SCALE_DEGREES % 6 == 0, "Value should be divisible by 6.");

  CalibrationData calibration;
  calibration.fast.degrees_div_6 = CALIBRATION_FAST_SCALE_DEGREES / 6;
  calibration.slow.degrees_div_6 = CALIBRATION_SLOW_SCALE_DEGREES / 6;

  // From what I can tell, this value is only used to compare against a previously made copy.
  // If the value matches that of the last connected wiimote which passed the "challenge",
  // then it seems the "challenge" is not performed a second time.
  calibration.uid_1 = 0x0b;
  calibration.uid_2 = 0xe9;

  // Update checksum (crc32 of all data other than the checksum itself):
  const auto crc_result = crc32(crc32(0, reinterpret_cast<const u8*>(&calibration), 0xe),
                                reinterpret_cast<const u8*>(&calibration) + 0x10, 0xe);

  calibration.crc32_lsb = u16(crc_result);
  calibration.crc32_msb = u16(crc_result >> 16);

  Common::BitCastPtr<CalibrationData>(m_reg_data.calibration_data.data()) = calibration;
}

void MotionPlus::DoState(PointerWrap& p)
{
  p.Do(m_reg_data);
  p.Do(m_progress_timer);
}

MotionPlus::ActivationStatus MotionPlus::GetActivationStatus() const
{
  if ((ACTIVE_DEVICE_ADDR << 1) == m_reg_data.ext_identifier[2])
  {
    if (ChallengeState::Activating == m_reg_data.challenge_state)
      return ActivationStatus::Activating;
    else
      return ActivationStatus::Active;
  }
  else
  {
    if (m_progress_timer != 0)
      return ActivationStatus::Deactivating;
    else
      return ActivationStatus::Inactive;
  }
}

MotionPlus::PassthroughMode MotionPlus::GetPassthroughMode() const
{
  return static_cast<PassthroughMode>(m_reg_data.ext_identifier[4]);
}

ExtensionPort& MotionPlus::GetExtPort()
{
  return m_extension_port;
}

int MotionPlus::BusRead(u8 slave_addr, u8 addr, int count, u8* data_out)
{
  switch (GetActivationStatus())
  {
  case ActivationStatus::Inactive:
    if (INACTIVE_DEVICE_ADDR != slave_addr)
    {
      // Passthrough to the connected extension. (if any)
      return m_i2c_bus.BusRead(slave_addr, addr, count, data_out);
    }

    // Perform a normal read of the M+ register.
    return RawRead(&m_reg_data, addr, count, data_out);

  case ActivationStatus::Active:
    // FYI: Motion plus does not respond to 0x53 when activated.
    if (ACTIVE_DEVICE_ADDR != slave_addr)
    {
      // No i2c passthrough when activated.
      return 0;
    }

    // Perform a normal read of the M+ register.
    return RawRead(&m_reg_data, addr, count, data_out);

  default:
  case ActivationStatus::Activating:
  case ActivationStatus::Deactivating:
    // The extension port is completely unresponsive here.
    return 0;
  }
}

int MotionPlus::BusWrite(u8 slave_addr, u8 addr, int count, const u8* data_in)
{
  switch (GetActivationStatus())
  {
  case ActivationStatus::Inactive:
  {
    if (INACTIVE_DEVICE_ADDR != slave_addr)
    {
      // Passthrough to the connected extension. (if any)
      return m_i2c_bus.BusWrite(slave_addr, addr, count, data_in);
    }

    DEBUG_LOG(WIIMOTE, "Inactive M+ write 0x%x : %s", addr, ArrayToString(data_in, count).c_str());

    auto const result = RawWrite(&m_reg_data, addr, count, data_in);

    if (PASSTHROUGH_MODE_OFFSET == addr)
    {
      OnPassthroughModeWrite();
    }

    return result;
  }

  case ActivationStatus::Active:
  {
    // FYI: Motion plus does not respond to 0x53 when activated.
    if (ACTIVE_DEVICE_ADDR != slave_addr)
    {
      // No i2c passthrough when activated.
      return 0;
    }

    DEBUG_LOG(WIIMOTE, "Active M+ write 0x%x : %s", addr, ArrayToString(data_in, count).c_str());

    auto const result = RawWrite(&m_reg_data, addr, count, data_in);

    switch (addr)
    {
    case offsetof(Register, init_trigger):
      // It seems a write of any value here triggers deactivation on a real M+.
      Deactivate();

      // Passthrough the write to the attached extension.
      // The M+ deactivation signal is cleverly the same as EXT initialization.
      m_i2c_bus.BusWrite(slave_addr, addr, count, data_in);
      break;

    case offsetof(Register, challenge_type):
      if (ChallengeState::ParameterXReady == m_reg_data.challenge_state)
      {
        DEBUG_LOG(WIIMOTE, "M+ challenge: 0x%x", m_reg_data.challenge_type);

        // After games read parameter x they write here to request y0 or y1.
        if (0 == m_reg_data.challenge_type)
        {
          // Preparing y0 on the real M+ is almost instant (30ms maybe).
          constexpr int PREPARE_Y0_MS = 30;
          m_progress_timer = ::Wiimote::UPDATE_FREQ * PREPARE_Y0_MS / 1000;
        }
        else
        {
          // A real M+ takes about 1200ms to prepare y1.
          // Games seem to not care that we don't take that long.
          constexpr int PREPARE_Y1_MS = 500;
          m_progress_timer = ::Wiimote::UPDATE_FREQ * PREPARE_Y1_MS / 1000;
        }

        // Games give the M+ a bit of time to compute the value.
        // y0 gets about half a second.
        // y1 gets at about 9.5 seconds.
        // After this the M+ will fail the "challenge".

        m_reg_data.challenge_state = ChallengeState::PreparingY;
      }
      break;

    case offsetof(Register, calibration_trigger):
      // Games seem to invoke this to start and stop calibration. Exact consequences unknown.
      DEBUG_LOG(WIIMOTE, "M+ calibration trigger: 0x%x", m_reg_data.calibration_trigger);
      break;

    case PASSTHROUGH_MODE_OFFSET:
      // Games sometimes (not often) write zero here to deactivate the M+.
      OnPassthroughModeWrite();
      break;
    }

    return result;
  }

  default:
  case ActivationStatus::Activating:
  case ActivationStatus::Deactivating:
    // The extension port is completely unresponsive here.
    return 0;
  }
}

void MotionPlus::OnPassthroughModeWrite()
{
  const auto status = GetActivationStatus();

  switch (GetPassthroughMode())
  {
  case PassthroughMode::Disabled:
  case PassthroughMode::Nunchuk:
  case PassthroughMode::Classic:
    if (ActivationStatus::Active != status)
      Activate();
    break;

  default:
    if (ActivationStatus::Inactive != status)
      Deactivate();
    break;
  }
}

void MotionPlus::Activate()
{
  DEBUG_LOG(WIIMOTE, "M+ has been activated.");

  m_reg_data.ext_identifier[2] = ACTIVE_DEVICE_ADDR << 1;

  // We must do this to reset our extension_connected and is_mp_data flags:
  m_reg_data.controller_data = {};

  m_reg_data.challenge_state = ChallengeState::Activating;

  // M+ takes a bit of time to activate. During which it is completely unresponsive.
  // This also affects the device detect pin which results in wiimote status reports.
  constexpr int ACTIVATION_MS = 20;
  m_progress_timer = ::Wiimote::UPDATE_FREQ * ACTIVATION_MS / 1000;
}

void MotionPlus::Deactivate()
{
  DEBUG_LOG(WIIMOTE, "M+ has been deactivated.");

  m_reg_data.ext_identifier[2] = INACTIVE_DEVICE_ADDR << 1;

  // M+ takes a bit of time to deactivate. During which it is completely unresponsive.
  // This also affects the device detect pin which results in wiimote status reports.
  constexpr int DEACTIVATION_MS = 20;
  m_progress_timer = ::Wiimote::UPDATE_FREQ * DEACTIVATION_MS / 1000;
}

bool MotionPlus::ReadDeviceDetectPin() const
{
  switch (GetActivationStatus())
  {
  case ActivationStatus::Inactive:
    // When inactive the device detect pin reads from the ext port:
    return m_extension_port.IsDeviceConnected();

  case ActivationStatus::Active:
    return true;

  default:
  case ActivationStatus::Activating:
  case ActivationStatus::Deactivating:
    return false;
  }
}

bool MotionPlus::IsButtonPressed() const
{
  return false;
}

void MotionPlus::Update()
{
  if (m_progress_timer)
    --m_progress_timer;

  if (!m_progress_timer && ActivationStatus::Activating == GetActivationStatus())
  {
    // M+ is active now that the timer is up.
    m_reg_data.challenge_state = ChallengeState::PreparingX;

    // Games give the M+ about a minute to prepare x before failure.
    // A real M+ can take about 1500ms.
    // The SDK seems to have a race condition that fails if a non-ready value is not read.
    // A necessary delay preventing challenge failure is not inserted if x is immediately ready.
    // So we must use at least a small delay.
    // Note: This does not delay game start. The challenge takes place in the background.
    constexpr int PREPARE_X_MS = 500;
    m_progress_timer = ::Wiimote::UPDATE_FREQ * PREPARE_X_MS / 1000;
  }

  if (ActivationStatus::Active != GetActivationStatus())
    return;

  u8* const data = m_reg_data.controller_data.data();
  DataFormat mplus_data = Common::BitCastPtr<DataFormat>(data);

  const bool is_ext_connected = m_extension_port.IsDeviceConnected();

  // Check for extension change:
  if (is_ext_connected != mplus_data.extension_connected)
  {
    if (is_ext_connected)
    {
      DEBUG_LOG(WIIMOTE, "M+ initializing new extension.");

      // The M+ automatically initializes an extension when attached.

      // What we do here does not exactly match a real M+,
      // but it's close enough for our emulated extensions which are not very picky.

      // Disable encryption
      {
        constexpr u8 INIT_OFFSET = offsetof(Register, init_trigger);
        std::array<u8, 1> enc_data = {0x55};
        m_i2c_bus.BusWrite(ACTIVE_DEVICE_ADDR, INIT_OFFSET, int(enc_data.size()), enc_data.data());
      }

      // Read identifier
      {
        constexpr u8 ID_OFFSET = offsetof(Register, ext_identifier);
        std::array<u8, 6> id_data = {};
        m_i2c_bus.BusRead(ACTIVE_DEVICE_ADDR, ID_OFFSET, int(id_data.size()), id_data.data());
        m_reg_data.passthrough_ext_id_0 = id_data[0];
        m_reg_data.passthrough_ext_id_4 = id_data[4];
        m_reg_data.passthrough_ext_id_5 = id_data[5];
      }

      // Read calibration data
      {
        constexpr u8 CAL_OFFSET = offsetof(Register, calibration_data);
        m_i2c_bus.BusRead(ACTIVE_DEVICE_ADDR, CAL_OFFSET,
                          int(m_reg_data.passthrough_ext_calib.size()),
                          m_reg_data.passthrough_ext_calib.data());
      }
    }

    // Update flag in register:
    mplus_data.extension_connected = is_ext_connected;
    Common::BitCastPtr<DataFormat>(data) = mplus_data;
  }

  // Only perform any of the following challenge logic if our timer is up.
  if (m_progress_timer)
    return;

  switch (m_reg_data.challenge_state)
  {
  case ChallengeState::PreparingX:
    // Big-int little endian parameter x.
    m_reg_data.challenge_data = {
        0x99, 0x1a, 0x07, 0x1b, 0x97, 0xf1, 0x11, 0x78, 0x0c, 0x42, 0x2b, 0x68, 0xdf,
        0x44, 0x38, 0x0d, 0x2b, 0x7e, 0xd6, 0x84, 0x84, 0x58, 0x65, 0xc9, 0xf2, 0x95,
        0xd9, 0xaf, 0xb6, 0xc4, 0x87, 0xd5, 0x18, 0xdb, 0x67, 0x3a, 0xc0, 0x71, 0xec,
        0x3e, 0xf4, 0xe6, 0x7e, 0x35, 0xa3, 0x29, 0xf8, 0x1f, 0xc5, 0x7c, 0x3d, 0xb9,
        0x56, 0x22, 0x95, 0x98, 0x8f, 0xfb, 0x66, 0x3e, 0x9a, 0xdd, 0xeb, 0x7e,
    };

    DEBUG_LOG(WIIMOTE, "M+ parameter x ready.");
    m_reg_data.challenge_state = ChallengeState::ParameterXReady;
    break;

  case ChallengeState::PreparingY:
    if (0 == m_reg_data.challenge_type)
    {
      // Big-int little endian parameter y0.
      m_reg_data.challenge_data = {
          0xe8, 0x77, 0x5a, 0x67, 0xb8, 0x86, 0x4b, 0x41, 0xa1, 0x2a, 0x4e, 0xb1, 0x8d,
          0xdb, 0x64, 0x5b, 0x44, 0x30, 0xdc, 0x68, 0x90, 0x36, 0x9f, 0xeb, 0x29, 0x00,
          0xa6, 0xad, 0xbc, 0x16, 0x8e, 0x06, 0x12, 0xf3, 0xe1, 0x75, 0x26, 0x81, 0xc7,
          0x69, 0xb7, 0x24, 0x23, 0xb5, 0xfe, 0x7b, 0x6e, 0x78, 0x05, 0x93, 0x4d, 0x91,
          0x96, 0xa0, 0x15, 0x00, 0x63, 0x4e, 0x6b, 0x3e, 0xed, 0x11, 0x45, 0x71,
      };
    }
    else
    {
      // Big-int little endian parameter y1.
      m_reg_data.challenge_data = {
          0xa5, 0x84, 0x1f, 0xd6, 0xbd, 0xdc, 0x7a, 0x4c, 0xf3, 0xc0, 0x24, 0xe0, 0x92,
          0xef, 0x19, 0x28, 0x65, 0xe0, 0x62, 0x7c, 0x9b, 0x41, 0x6f, 0x12, 0xc3, 0xac,
          0x78, 0xe4, 0xfc, 0x6b, 0x7b, 0x0a, 0xb4, 0x50, 0xd6, 0xf2, 0x45, 0xf7, 0x93,
          0x04, 0xaf, 0xf2, 0xb7, 0x26, 0x94, 0xee, 0xad, 0x92, 0x05, 0x6d, 0xe5, 0xc6,
          0xd6, 0x36, 0xdc, 0xa5, 0x69, 0x0f, 0xc8, 0x99, 0xf2, 0x1c, 0x4e, 0x0d,
      };
    }

    DEBUG_LOG(WIIMOTE, "M+ parameter y ready.");
    m_reg_data.challenge_state = ChallengeState::ParameterYReady;
    break;

  default:
    break;
  }
}

// This is something that is triggered by a read of 0x00 on real hardware.
// But we do it here for determinism reasons.
void MotionPlus::PrepareInput(const Common::Vec3& angular_velocity)
{
  if (GetActivationStatus() != ActivationStatus::Active)
    return;

  u8* const data = m_reg_data.controller_data.data();

  // FYI: A real M+ seems to always send some garbage/mystery data for the first report,
  // followed by a normal M+ data report, and then finally passhrough data (if enabled).
  // Things seem to work without doing that so we'll just send normal M+ data right away.
  DataFormat mplus_data = Common::BitCastPtr<DataFormat>(data);

  // Maintain the current state of this bit rather than reading from the port.
  // We update this bit elsewhere and performs some tasks on change.
  const bool is_ext_connected = mplus_data.extension_connected;

  // After the first "garbage" report a real M+ alternates between M+ and EXT data.
  // Failure to read from the extension results in a fallback to M+ data.
  mplus_data.is_mp_data ^= true;

  // If the last frame had M+ data try to send some non-M+ data:
  if (!mplus_data.is_mp_data)
  {
    // Real M+ only ever reads 6 bytes from the extension which is triggered by a read at 0x00.
    // Data after 6 bytes seems to be zero-filled.
    // After reading from the EXT, the real M+ uses that data for the next frame.
    // But we are going to use it for the current frame, because we can.
    constexpr int EXT_AMT = 6;
    // Always read from 0x52 @ 0x00:
    constexpr u8 EXT_SLAVE = ExtensionPort::REPORT_I2C_SLAVE;
    constexpr u8 EXT_ADDR = ExtensionPort::REPORT_I2C_ADDR;

    switch (GetPassthroughMode())
    {
    case PassthroughMode::Disabled:
    {
      // Passthrough disabled, always send M+ data:
      mplus_data.is_mp_data = true;
      break;
    }
    case PassthroughMode::Nunchuk:
    {
      if (EXT_AMT == m_i2c_bus.BusRead(EXT_SLAVE, EXT_ADDR, EXT_AMT, data))
      {
        // Passthrough data modifications via wiibrew.org
        // Verified on real hardware via a test of every bit.
        // Data passing through drops the least significant bit of the three accelerometer values.
        // Bit 7 of byte 5 is moved to bit 6 of byte 5, overwriting it
        Common::SetBit(data[5], 6, Common::ExtractBit(data[5], 7));
        // Bit 0 of byte 4 is moved to bit 7 of byte 5
        Common::SetBit(data[5], 7, Common::ExtractBit(data[4], 0));
        // Bit 3 of byte 5 is moved to  bit 4 of byte 5, overwriting it
        Common::SetBit(data[5], 4, Common::ExtractBit(data[5], 3));
        // Bit 1 of byte 5 is moved to bit 3 of byte 5
        Common::SetBit(data[5], 3, Common::ExtractBit(data[5], 1));
        // Bit 0 of byte 5 is moved to bit 2 of byte 5, overwriting it
        Common::SetBit(data[5], 2, Common::ExtractBit(data[5], 0));

        mplus_data = Common::BitCastPtr<DataFormat>(data);

        // Bit 0 and 1 of byte 5 contain a M+ flag and a zero bit which is set below.
        mplus_data.is_mp_data = false;
      }
      else
      {
        // Read failed (extension unplugged), Send M+ data instead
        mplus_data.is_mp_data = true;
      }
      break;
    }
    case PassthroughMode::Classic:
    {
      if (EXT_AMT == m_i2c_bus.BusRead(EXT_SLAVE, EXT_ADDR, EXT_AMT, data))
      {
        // Passthrough data modifications via wiibrew.org
        // Verified on real hardware via a test of every bit.
        // Data passing through drops the least significant bit of the axes of the left (or only)
        // joystick Bit 0 of Byte 4 is overwritten [by the 'extension_connected' flag] Bits 0 and
        // 1 of Byte 5 are moved to bit 0 of Bytes 0 and 1, overwriting what was there before.
        Common::SetBit(data[0], 0, Common::ExtractBit(data[5], 0));
        Common::SetBit(data[1], 0, Common::ExtractBit(data[5], 1));

        mplus_data = Common::BitCastPtr<DataFormat>(data);

        // Bit 0 and 1 of byte 5 contain a M+ flag and a zero bit which is set below.
        mplus_data.is_mp_data = false;
      }
      else
      {
        // Read failed (extension unplugged), Send M+ data instead
        mplus_data.is_mp_data = true;
      }
      break;
    }
    default:
      // This really shouldn't happen as the M+ deactivates on an invalid mode write.
      ERROR_LOG(WIIMOTE, "M+ unknown passthrough-mode %d", int(GetPassthroughMode()));
      mplus_data.is_mp_data = true;
      break;
    }
  }

  // If the above logic determined this should be M+ data, update it here.
  if (mplus_data.is_mp_data)
  {
    constexpr int BITS_OF_PRECISION = 14;

    // Conversion from radians to the calibrated values in degrees.
    constexpr float VALUE_SCALE =
        (CALIBRATION_SCALE_OFFSET >> (CALIBRATION_BITS - BITS_OF_PRECISION)) /
        float(MathUtil::TAU) * 360;

    constexpr float SLOW_SCALE = VALUE_SCALE / CALIBRATION_SLOW_SCALE_DEGREES;
    constexpr float FAST_SCALE = VALUE_SCALE / CALIBRATION_FAST_SCALE_DEGREES;

    constexpr s32 ZERO_VALUE = CALIBRATION_ZERO >> (CALIBRATION_BITS - BITS_OF_PRECISION);
    constexpr s32 MAX_VALUE = (1 << BITS_OF_PRECISION) - 1;

    static_assert(ZERO_VALUE == 1 << (BITS_OF_PRECISION - 1),
                  "SLOW_MAX_RAD_PER_SEC assumes calibrated zero is at center of sensor values.");

    constexpr u16 SENSOR_RANGE = 1 << (BITS_OF_PRECISION - 1);
    constexpr float SLOW_MAX_RAD_PER_SEC = SENSOR_RANGE / SLOW_SCALE;

    // Slow (high precision) scaling can be used if it fits in the sensor range.
    const float yaw = angular_velocity.z;
    mplus_data.yaw_slow = (std::abs(yaw) < SLOW_MAX_RAD_PER_SEC);
    s32 yaw_value = yaw * (mplus_data.yaw_slow ? SLOW_SCALE : FAST_SCALE);

    const float roll = angular_velocity.y;
    mplus_data.roll_slow = (std::abs(roll) < SLOW_MAX_RAD_PER_SEC);
    s32 roll_value = roll * (mplus_data.roll_slow ? SLOW_SCALE : FAST_SCALE);

    const float pitch = angular_velocity.x;
    mplus_data.pitch_slow = (std::abs(pitch) < SLOW_MAX_RAD_PER_SEC);
    s32 pitch_value = pitch * (mplus_data.pitch_slow ? SLOW_SCALE : FAST_SCALE);

    yaw_value = MathUtil::Clamp(yaw_value + ZERO_VALUE, 0, MAX_VALUE);
    roll_value = MathUtil::Clamp(roll_value + ZERO_VALUE, 0, MAX_VALUE);
    pitch_value = MathUtil::Clamp(pitch_value + ZERO_VALUE, 0, MAX_VALUE);

    // TODO: Remove before merge.
    // INFO_LOG(WIIMOTE, "M+ YAW: 0x%x slow:%d", yaw_value, mplus_data.yaw_slow);
    // INFO_LOG(WIIMOTE, "M+ ROL: 0x%x slow:%d", roll_value, mplus_data.roll_slow);
    // INFO_LOG(WIIMOTE, "M+ PIT: 0x%x slow:%d", pitch_value, mplus_data.pitch_slow);

    // Bits 0-7
    mplus_data.yaw1 = u8(yaw_value);
    mplus_data.roll1 = u8(roll_value);
    mplus_data.pitch1 = u8(pitch_value);

    // Bits 8-13
    mplus_data.yaw2 = u8(yaw_value >> 8);
    mplus_data.roll2 = u8(roll_value >> 8);
    mplus_data.pitch2 = u8(pitch_value >> 8);
  }

  mplus_data.extension_connected = is_ext_connected;
  mplus_data.zero = 0;

  Common::BitCastPtr<DataFormat>(data) = mplus_data;
}

}  // namespace WiimoteEmu
