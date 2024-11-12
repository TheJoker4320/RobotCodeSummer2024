// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArm extends Command {
  private boolean m_isReversed;
  private Arm m_arm;
  private double m_degree;
  public MoveArm(Arm arm, boolean isReversed) {
    m_arm = arm;
    m_isReversed = isReversed;
    addRequirements(m_arm);
    SmartDashboard.putNumber("MOVEARMDEGREE", m_degree);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_degree = m_arm.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setReference(m_degree);
    if (m_isReversed){
      if (m_degree >= 3) {m_degree -= Constants.ArmConstants.DPMS_SPEED;}
    }
    else{
      if (m_degree <= 90){m_degree += Constants.ArmConstants.DPMS_SPEED;}
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
