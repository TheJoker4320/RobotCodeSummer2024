// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class SwitchArmIsConstrainted extends InstantCommand {

  private final Arm m_arm;

  /** Creates a new SwitchArmIsConstrainted. */
  public SwitchArmIsConstrainted(Arm arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.switchConstraint();
  }
}
