// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem(VisionSubsystem *passedVisionSubsystem)
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftTurningEncoderPorts,
                  kFrontLeftEncoderOffset,
                  flipFL},

      m_rearLeft{kRearLeftDriveMotorPort,
                 kRearLeftTurningMotorPort,
                 kRearLeftTurningEncoderPorts,
                 kRearLeftEncoderOffset,
                 flipBL},

      m_frontRight{kFrontRightDriveMotorPort,
                   kFrontRightTurningMotorPort,
                   kFrontRightTurningEncoderPorts,
                   kFrontRightEncoderOffset,
                   flipFR},

      m_rearRight{kRearRightDriveMotorPort,
                  kRearRightTurningMotorPort,
                  kRearRightTurningEncoderPorts,
                  kRearRightEncoderOffset,
                  flipBR},

      m_odometry{kDriveKinematics,
                 GetHeading(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {

  m_visionSubsystem = passedVisionSubsystem;

  // Configure the AutoBuilder last
  pathplanner::AutoBuilder::configureHolonomic(
      [this]() { return this->GetPose(); },                                                          // Robot pose supplier
      [this](frc::Pose2d pose) { this->ResetOdometry(pose); },                                       // Method to reset odometry (will be called if your auto has a starting pose)
      [this]() { return this->kDriveKinematics.ToChassisSpeeds(this->GetModuleStates()); },          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds) { this->Drive(speeds.vx, speeds.vy, speeds.omega, true); },  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      pathplanner::HolonomicPathFollowerConfig(                                                      // HolonomicPathFollowerConfig, this should likely live in your Constants class
          pathplanner::PIDConstants(AutoConstants::kPXController, 0.0, 0.0),
          pathplanner::PIDConstants(AutoConstants::kPThetaController, 0.0, 0.0),  // Rotation PID constants
          AutoConstants::kMaxSpeed,                                               // Max module speed, in m/s
          this->kModuleRadius,                                                    // Drive base radius in meters. Distance from robot center to furthest module.
          pathplanner::ReplanningConfig(true, true, 1.0_m, 0.25_m)                // Default path replanning config. See the API for the options here
          ),
      []() {
        auto ally = frc::DriverStation::GetAlliance();
        return ally.value() == frc::DriverStation::Alliance::kRed;
      },
      this  // Reference to this subsystem to set requirements
  );

  frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Field Y", GetPose().Y().value());
  frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetYaw());

  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(GetHeading(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});

  m_visionSubsystem->SetPoseLL3(&m_odometry);
  // m_visionSubsystem->SetPoseLL2(&m_odometry);

  m_field.SetRobotPose(GetPose());

  if (auto ally = frc::DriverStation::GetAlliance()) {
    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      allianceColorBlue = false;
    } else {
      allianceColorBlue = true;
    }
  }
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  frc::SmartDashboard::PutNumber("Drive X", xSpeed.value());
  frc::SmartDashboard::PutNumber("Drive Y", ySpeed.value());
  frc::SmartDashboard::PutNumber("Drive Z", rot.value());

  if (allianceColorBlue) {
    xSpeed = -xSpeed;
    ySpeed = -ySpeed;
  }

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetPose().Rotation())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

wpi::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() {
  return wpi::array{
      m_frontLeft.GetState(),
      m_frontRight.GetState(),
      m_rearLeft.GetState(),
      m_rearRight.GetState()};
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() {
  if (allianceColorBlue) {
    return m_gyro.GetRotation2d().Degrees();
  } else {
    return (m_gyro.GetRotation2d().Degrees() + 180_deg);
  }
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return m_gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  frc::Pose2d pose = m_odometry.GetEstimatedPosition();
  return pose;
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

frc2::CommandPtr DriveSubsystem::GeneratedPath(frc::Pose2d targetPose) {
  // Create the constraints to use while pathfinding
  pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
      5.0_mps, 4.5_mps_sq,
      1050_deg_per_s, 1000_deg_per_s_sq);

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  frc2::CommandPtr pathfindingCommand = pathplanner::AutoBuilder::pathfindToPose(
      targetPose,
      constraints,
      0.0_mps,  // Goal end velocity in meters/sec
      0.0_m     // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  return pathfindingCommand;
}