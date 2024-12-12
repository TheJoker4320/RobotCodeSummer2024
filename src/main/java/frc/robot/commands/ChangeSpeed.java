// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
public class ChangeSpeed extends InstantCommand {
  private DriveSubsystem m_robotDrive;
  private double m_speedMultiplier;
  public ChangeSpeed(DriveSubsystem robotDrive, double speedMultiplier) {
    m_robotDrive = robotDrive;
    m_speedMultiplier = speedMultiplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.setInputMultiplier(m_speedMultiplier);
  }
}
