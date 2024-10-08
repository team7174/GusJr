// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "Constants.h"
#include "HardwareConfig.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               int turningEncoderPorts, double encoderOffset,
               bool flipDrive);

  frc::SwerveModuleState GetState();

  frc::SwerveModulePosition GetPosition();

  double getTurnEncoderDistance();

  double getDriveEncoderRate();

  double getDriveEncoderDistance();

  void SetDesiredState(const frc::SwerveModuleState &state);

  void ResetEncoders();

  units::angle::turn_t DegreesToFalcon(units::degree_t angle);
  units::angle::degree_t FalconToDegrees(double turn);

  HardwareConfig motorConfigs;

  ctre::phoenix6::controls::VelocityVoltage m_driveRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps};
  //    ctre::phoenix6::controls::MotionMagicVelocityVoltage m_driveRequest{0_tps};
  ctre::phoenix6::controls::PositionVoltage m_turnRequest = ctre::phoenix6::controls::PositionVoltage{0_tr};

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

  ctre::phoenix6::hardware::TalonFX m_driveMotor;
  ctre::phoenix6::hardware::TalonFX m_turningMotor;
  HardwareConfig m_Settings;
  frc::AnalogEncoder m_turningEncoder;

  double absEnc;
  
  frc::PIDController steerPID{0.007, 0, 0};
};