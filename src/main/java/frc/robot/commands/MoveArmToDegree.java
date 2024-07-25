// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class MoveArmToDegree extends Command {
  private final Arm m_arm;
  private PIDController m_pidController;

  /** Creates a new MoveArmToDegree. */
  public MoveArmToDegree(double degree) {
    m_arm = Arm.getInstance();
    m_pidController = m_arm.getPidController();
    m_pidController.setP(Constants.ArmConstants.PID_P);
    m_pidController.setI(Constants.ArmConstants.PID_I);
    m_pidController.setD(Constants.ArmConstants.PID_D);
    m_pidController.setTolerance(Constants.ArmConstants.PID_TOLERANCE);
    m_pidController.setSetpoint(degree);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_pidController.calculate(m_pidController.getSetpoint());
    m_arm.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atSetpoint();
  }
}
