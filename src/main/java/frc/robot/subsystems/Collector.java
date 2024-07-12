// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;


public class Collector extends SubsystemBase {
  /** Creates a new Collector. */
  private static Collector collectorInstance = null;
  private final DigitalInput m_limitSwitch;
  private final TalonSRX m_primaryMotor;

  public static Collector getInstance() {
    if (collectorInstance == null)
    {
      collectorInstance = new Collector();
    }
    return collectorInstance;
  }
  public Collector() {
    m_limitSwitch = new DigitalInput(CollectorConstants.LIMIT_SWITCH_PORT);
    m_primaryMotor = new TalonSRX(CollectorConstants.PRIMARY_MOTOR_PORT);
  }

  public DigitalInput GetLimitSwitch() {
    return m_limitSwitch;
  }
  public void SetCollectorSpeed(double speed) {
    m_primaryMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }


  @Override
  public void periodic() {
    
  }
}
