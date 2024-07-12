// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;;

public class Climber extends SubsystemBase {
  private CANSparkMax m_primaryMotor;
  private CANSparkMax m_secondaryMotor;
  
  private static Climber m_instance;

  /**
   * Singleton implementation
   * 
   * @return The only instance of the Climber subsystem
   */
  public static Climber getInstance(){
    if (m_instance == null)
      m_instance = new Climber();
    return m_instance;
  }


  /** Creates a new Climber. */
  private Climber() {
    m_primaryMotor = new CANSparkMax(ClimberConstants.PRIMARY_MOTOR_ID, MotorType.kBrushless);
    m_secondaryMotor = new CANSparkMax(ClimberConstants.SECONDARY_MOTOR_ID, MotorType.kBrushless);

    m_primaryMotor.setSmartCurrentLimit(ClimberConstants.SMART_CURRENT_LIMIT);
    m_secondaryMotor.setSmartCurrentLimit(ClimberConstants.SMART_CURRENT_LIMIT);


    m_primaryMotor.setInverted(true);
    m_secondaryMotor.follow(m_primaryMotor);
  }

  /**
   * The method sets the speed for the climber
   * 
   * @param speed The speed to set to the climber
   */
  public void setSpeed(int speed){
    m_primaryMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
