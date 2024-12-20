// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootTimeBased extends Command {
  /** Creates a new ShootTimeBased. */
  private Shooter mShooter;
  private double mSpeed = Constants.ShooterConstants.SHOOTER_SPEED;
  private Timer mTimer;
  private double mTimeout;
  public ShootTimeBased(Shooter shooter, double timeOut) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mTimeout = timeOut;
    mTimer = new Timer();
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.shoot(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTimer.stop();
    mShooter.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.get() >= mTimeout;
  }
}
