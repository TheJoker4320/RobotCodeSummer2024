// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToDegree extends Command {
  private Arm m_arm;
  private double m_degree;
  
  public MoveArmToDegree(Arm arm, double degree) {
    m_arm = arm; 
    addRequirements(arm);
    m_degree = degree;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setReference(m_degree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(m_arm.getPosition() - m_degree) <= 2;
    return true;
  }
}
