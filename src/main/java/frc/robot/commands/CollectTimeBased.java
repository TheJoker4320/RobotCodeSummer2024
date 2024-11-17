// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Collector;

public class CollectTimeBased extends Command {
  /** Creates a new CollectTimeBased. */
  private Collector mCollector;
  private double mTimeout;
  private Timer mTimer;
  public CollectTimeBased(Collector collector, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    mCollector = collector;
    mTimeout = timeout;
    mTimer = new Timer();
    addRequirements(mCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mCollector.setSpeed(Constants.CollectorConstants.COLLECTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCollector.setSpeed(0);
    mTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.get() >= mTimeout;
  }
}
