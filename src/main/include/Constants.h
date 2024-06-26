// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

#pragma once

namespace DriveConstants {
constexpr int kFrontLeftDriveMotorPort = 10;
constexpr int kRearLeftDriveMotorPort = 20;
constexpr int kRearRightDriveMotorPort = 30;
constexpr int kFrontRightDriveMotorPort = 40;

constexpr int kFrontLeftTurningMotorPort = 11;
constexpr int kRearLeftTurningMotorPort = 21;
constexpr int kRearRightTurningMotorPort = 31;
constexpr int kFrontRightTurningMotorPort = 41;

constexpr int kFrontLeftTurningEncoderPorts = 1;
constexpr int kRearLeftTurningEncoderPorts = 2;
constexpr int kRearRightTurningEncoderPorts = 3;
constexpr int kFrontRightTurningEncoderPorts = 0;

constexpr double kFrontLeftEncoderOffset = 0.3229283664585561;
constexpr double kRearLeftEncoderOffset = 0.8181793961686633;
constexpr double kFrontRightEncoderOffset = 0.6411730390524463;
constexpr double kRearRightEncoderOffset = 0.005668068613012625;

constexpr bool flipFL = false;
constexpr bool flipFR = false;
constexpr bool flipBL = false;
constexpr bool flipBR = false;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr double kWheelDiameterMeters = 0.1016;
constexpr double kWheelCircumference = kWheelDiameterMeters * std::numbers::pi;
constexpr double driveGearRatio = 6.0;
constexpr double angleGearRatio = 12.0;

}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 5.0_mps;
constexpr auto kMaxAngularSpeed = 15_rad_per_s;
constexpr auto kMaxAngularAcceleration = 30_rad_per_s_sq;

constexpr double kPXController = 1.0;
constexpr double kPThetaController = 1.0;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace DriveConstants {
constexpr auto kMaxSpeed = 5.0_mps;
constexpr auto kMaxAngularSpeed = 15_rad_per_s;
constexpr auto kMaxAngularAcceleration = 30_rad_per_s_sq;
}  // namespace DriveConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kSecondaryControllerPort = 1;
}  // namespace OIConstants