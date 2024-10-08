// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() : m_drive(&m_visionSubsystem) {
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftY(), 0.1) * DriveConstants::kMaxSpeed.value()},
            units::meters_per_second_t{frc::ApplyDeadband(-m_driverController.GetLeftX(), 0.1) * DriveConstants::kMaxSpeed.value()},
            units::radians_per_second_t{frc::ApplyDeadband(-m_driverController.GetRightX(), 0.1) * DriveConstants::kMaxAngularSpeed.value()},
            true);
      },
      {&m_drive}));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::Trigger{[this]() { return m_driverController.GetYButtonPressed(); }}.OnTrue(frc2::cmd::RunOnce([this] { m_drive.ZeroHeading(); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto(pathPlannerChooser.GetSelected()).ToPtr();
}