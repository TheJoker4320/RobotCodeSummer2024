// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private static Shooter shooterInstance = null;
  private TalonSRX master;
  private TalonSRX slave;
  private Encoder encoder;
  //TODO: set pid values
  private PIDController pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

  public static Shooter getInstance(){
    if (shooterInstance == null){
      shooterInstance = new Shooter();
    }
    return shooterInstance;
  }

  public Shooter() {
    //TODO: set motor ports and encoder ports
    master = new TalonSRX(ShooterConstants.MASTER_MOTOR_PORT);
    slave = new TalonSRX(ShooterConstants.SLAVE_MOTOR_PORT);
    slave.follow(master);
    encoder = new Encoder(ShooterConstants.SHOOTER_ENCODER_PORT_A, ShooterConstants.SHOOTER_ENCODER_PORT_B, false);
    encoder.reset();
    encoder.setDistancePerPulse(1.0/2048);
  }
  public double getSpeed(){
    return encoder.getRate();
  }

  public void shoot(double speed){
    pidController.setSetpoint(speed);
    double output = pidController.calculate(getSpeed());
    master.set(TalonSRXControlMode.PercentOutput, output);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
