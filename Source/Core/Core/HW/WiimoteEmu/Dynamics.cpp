// Copyright 2019 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/WiimoteEmu/Dynamics.h"

#include <cmath>

#include "Common/MathUtil.h"
#include "Core/Config/WiimoteInputSettings.h"
#include "Core/HW/Wiimote.h"
#include "Core/HW/WiimoteEmu/WiimoteEmu.h"
#include "InputCommon/ControllerEmu/ControlGroup/Buttons.h"
#include "InputCommon/ControllerEmu/ControlGroup/Cursor.h"
#include "InputCommon/ControllerEmu/ControlGroup/Force.h"
#include "InputCommon/ControllerEmu/ControlGroup/Tilt.h"

namespace
{
constexpr int SHAKE_FREQ = 6;
// Frame count of one up/down shake
// < 9 no shake detection in "Wario Land: Shake It"
constexpr int SHAKE_STEP_MAX = ::Wiimote::UPDATE_FREQ / SHAKE_FREQ;

// Given a velocity, acceleration, and maximum jerk value,
// calculate change in position after a stop in the shortest possible time.
// Used to smoothly adjust acceleration and come to complete stops at precise positions.
// Based on equations for motion with constant jerk.
// s = s0 + v0 t + a0 t^2 / 2 + j t^3 / 6
double CalculateStopDistance(double velocity, double acceleration, double max_jerk)
{
  // Math below expects velocity to be non-negative.
  const auto velocity_flip = (velocity < 0 ? -1 : 1);

  const auto v_0 = velocity * velocity_flip;
  const auto a_0 = acceleration * velocity_flip;
  const auto j = max_jerk;

  // Time to reach zero acceleration.
  const auto t_0 = a_0 / j;

  // Distance to reach zero acceleration.
  const auto d_0 = std::pow(a_0, 3) / (3 * j * j) + (a_0 * v_0) / j;

  // Velocity at zero acceleration.
  const auto v_1 = v_0 + a_0 * std::abs(t_0) - std::copysign(j * t_0 * t_0 / 2, t_0);

  // Distance to complete stop.
  const auto d_1 = std::copysign(std::pow(std::abs(v_1), 3.0 / 2), v_1) / std::sqrt(j);

  return (d_0 + d_1) * velocity_flip;
}

double CalculateStopDistance(double velocity, double max_accel)
{
  return velocity * velocity / (2 * std::copysign(max_accel, velocity));
}

}  // namespace

namespace WiimoteEmu
{
Common::Vec3 EmulateShake(ControllerEmu::Buttons* const buttons_group, const double intensity,
                          u8* const shake_step)
{
  // shake is a bitfield of X,Y,Z shake button states
  static const unsigned int btns[] = {0x01, 0x02, 0x04};
  unsigned int shake = 0;
  buttons_group->GetState(&shake, btns);

  Common::Vec3 accel;

  for (std::size_t i = 0; i != accel.data.size(); ++i)
  {
    if (shake & (1 << i))
    {
      accel.data[i] = std::sin(MathUtil::TAU * shake_step[i] / SHAKE_STEP_MAX) * intensity *
                      GRAVITY_ACCELERATION;
      shake_step[i] = (shake_step[i] + 1) % SHAKE_STEP_MAX;
    }
    else
    {
      shake_step[i] = 0;
    }
  }

  return accel;
}

Common::Vec3 EmulateDynamicShake(DynamicData& dynamic_data,
                                 ControllerEmu::Buttons* const buttons_group,
                                 const DynamicConfiguration& config, u8* const shake_step)
{
  // shake is a bitfield of X,Y,Z shake button states
  static const unsigned int btns[] = {0x01, 0x02, 0x04};
  unsigned int shake = 0;
  buttons_group->GetState(&shake, btns);

  Common::Vec3 accel;

  for (std::size_t i = 0; i != accel.data.size(); ++i)
  {
    if ((shake & (1 << i)) && dynamic_data.executing_frames_left[i] == 0)
    {
      dynamic_data.timing[i]++;
    }
    else if (dynamic_data.executing_frames_left[i] > 0)
    {
      accel.data[i] = std::sin(MathUtil::TAU * shake_step[i] / SHAKE_STEP_MAX) *
                      dynamic_data.intensity[i] * GRAVITY_ACCELERATION;
      shake_step[i] = (shake_step[i] + 1) % SHAKE_STEP_MAX;
      dynamic_data.executing_frames_left[i]--;
    }
    else if (shake == 0 && dynamic_data.timing[i] > 0)
    {
      if (dynamic_data.timing[i] > config.frames_needed_for_high_intensity)
      {
        dynamic_data.intensity[i] = config.high_intensity;
      }
      else if (dynamic_data.timing[i] < config.frames_needed_for_low_intensity)
      {
        dynamic_data.intensity[i] = config.low_intensity;
      }
      else
      {
        dynamic_data.intensity[i] = config.med_intensity;
      }
      dynamic_data.timing[i] = 0;
      dynamic_data.executing_frames_left[i] = config.frames_to_execute;
    }
    else
    {
      shake_step[i] = 0;
    }
  }

  return accel;
}

void EmulateTilt(RotationalState* state, ControllerEmu::Tilt* const tilt_group, float time_elapsed)
{
  const auto target = tilt_group->GetState();

  // 180 degrees is currently the max tilt value.
  const ControlState roll = target.x * MathUtil::PI;
  const ControlState pitch = target.y * MathUtil::PI;

  // TODO: expose this setting in UI:
  constexpr auto MAX_ACCEL = float(MathUtil::TAU * 50);

  ApproachAngleWithAccel(state, Common::Vec3(pitch, -roll, 0), MAX_ACCEL, time_elapsed);
}

void EmulateSwing(MotionState* state, ControllerEmu::Force* swing_group, float time_elapsed)
{
  const auto target = swing_group->GetState();

  // Note. Y/Z swapped because X/Y axis to the swing_group is X/Z to the wiimote.
  // X is negated because Wiimote X+ is to the left.
  ApproachPositionWithJerk(state, {-target.x, -target.z, target.y}, swing_group->GetMaxJerk(),
                           time_elapsed);

  const auto angle = state->position / swing_group->GetMaxDistance() * swing_group->GetTwistAngle();

  // TODO: expose this setting in UI or calculate something sensible from jerk.
  constexpr auto MAX_ACCEL = float(MathUtil::TAU * 50);

  ApproachAngleWithAccel(state, {-angle.z, 0, angle.x}, MAX_ACCEL, time_elapsed);
}

WiimoteCommon::DataReportBuilder::AccelData ConvertAccelData(const Common::Vec3& accel, u16 zero_g,
                                                             u16 one_g)
{
  const auto scaled_accel = accel * (one_g - zero_g) / float(GRAVITY_ACCELERATION);

  // 10-bit integers.
  constexpr long MAX_VALUE = (1 << 10) - 1;

  return {u16(MathUtil::Clamp(std::lround(scaled_accel.x + zero_g), 0l, MAX_VALUE)),
          u16(MathUtil::Clamp(std::lround(scaled_accel.y + zero_g), 0l, MAX_VALUE)),
          u16(MathUtil::Clamp(std::lround(scaled_accel.z + zero_g), 0l, MAX_VALUE))};
}

void EmulateCursor(MotionState* state, ControllerEmu::Cursor* ir_group, float time_elapsed)
{
  const auto cursor = ir_group->GetState(true);

  using Common::Matrix33;
  using Common::Matrix44;

  // Values are optimized for default settings in "Super Mario Galaxy 2"
  // This seems to be acceptable for a good number of games.
  // constexpr float YAW_ANGLE = 0.1472f;
  // constexpr float PITCH_ANGLE = 0.121f;
  constexpr float YAW_ANGLE = 0.3f / 5 * 1.435f * 3;
  constexpr float PITCH_ANGLE = 0.3f / 5 * 1.34f * 3;

  // Nintendo recommends a distance of 1-3 meters.
  constexpr float NEUTRAL_DISTANCE = 3.f;
  constexpr float MOVE_DISTANCE = 1.f;

  // TODO: Move state out of ControllerEmu::Cursor
  // TODO: Use ApproachPositionWithJerk
  // TODO: Move forward/backward after rotation.
  const auto new_position = Common::Vec3{0, NEUTRAL_DISTANCE - MOVE_DISTANCE * float(cursor.z), 0};
  state->acceleration = new_position - state->position;
  state->position = new_position;

  // TODO: expose this setting in UI:
  constexpr auto MAX_ACCEL = float(MathUtil::TAU * 100);

  ApproachAngleWithAccel(state, Common::Vec3(PITCH_ANGLE * -cursor.y, 0, YAW_ANGLE * -cursor.x),
                         MAX_ACCEL, time_elapsed);
}

void ApproachAngleWithAccel(RotationalState* state, const Common::Vec3& angle_target,
                            float max_accel, float time_elapsed)
{
  const auto stop_distance =
      Common::Vec3(CalculateStopDistance(state->angular_velocity.x, max_accel),
                   CalculateStopDistance(state->angular_velocity.y, max_accel),
                   CalculateStopDistance(state->angular_velocity.z, max_accel));

  const auto offset = angle_target - state->angle;
  const auto stop_offset = offset - stop_distance;

  const Common::Vec3 accel{std::copysign(max_accel, stop_offset.x),
                           std::copysign(max_accel, stop_offset.y),
                           std::copysign(max_accel, stop_offset.z)};

  state->angular_velocity += accel * time_elapsed;

  const auto change_in_angle =
      state->angular_velocity * time_elapsed + accel * time_elapsed * time_elapsed / 2;

  for (std::size_t i = 0; i != offset.data.size(); ++i)
  {
    // If new velocity will overshoot assume we would have stopped right on target.
    // TODO: Improve check to see if less accel would have caused undershoot.
    if ((change_in_angle.data[i] / offset.data[i]) > 1.0)
    {
      state->angular_velocity.data[i] = 0;
      state->angle.data[i] = angle_target.data[i];
    }
    else
    {
      state->angle.data[i] += change_in_angle.data[i];
    }
  }
}

void ApproachPositionWithJerk(PositionalState* state, const Common::Vec3& position_target,
                              float max_jerk, float time_elapsed)
{
  const auto stop_distance =
      Common::Vec3(CalculateStopDistance(state->velocity.x, state->acceleration.x, max_jerk),
                   CalculateStopDistance(state->velocity.y, state->acceleration.y, max_jerk),
                   CalculateStopDistance(state->velocity.z, state->acceleration.z, max_jerk));

  const auto offset = position_target - state->position;
  const auto stop_offset = offset - stop_distance;

  const Common::Vec3 jerk{std::copysign(max_jerk, stop_offset.x),
                          std::copysign(max_jerk, stop_offset.y),
                          std::copysign(max_jerk, stop_offset.z)};

  state->acceleration += jerk * time_elapsed;

  state->velocity += state->acceleration * time_elapsed + jerk * time_elapsed * time_elapsed / 2;

  const auto change_in_position = state->velocity * time_elapsed +
                                  state->acceleration * time_elapsed * time_elapsed / 2 +
                                  jerk * time_elapsed * time_elapsed * time_elapsed / 6;

  for (std::size_t i = 0; i != offset.data.size(); ++i)
  {
    // If new velocity will overshoot assume we would have stopped right on target.
    // TODO: Improve check to see if less jerk would have caused undershoot.
    if ((change_in_position.data[i] / offset.data[i]) > 1.0)
    {
      state->acceleration.data[i] = 0;
      state->velocity.data[i] = 0;
      state->position.data[i] = position_target.data[i];
    }
    else
    {
      state->position.data[i] += change_in_position.data[i];
    }
  }
}

Common::Matrix33 GetRotationalMatrix(const Common::Vec3& angle)
{
  return Common::Matrix33::RotateZ(angle.z) * Common::Matrix33::RotateY(angle.y) *
         Common::Matrix33::RotateX(angle.x);
}

}  // namespace WiimoteEmu
