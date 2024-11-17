// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_secondaryMotor;
  private final AbsoluteEncoder m_encoder;

  private final SparkPIDController m_pidController;

  private boolean m_isConstrainted;
  
  private static Arm m_instance = null;

  /**
   * Singleton implementation
   * 
   * @return The only instance of the Arm class
   */
  public static Arm getInstance() {
    if (m_instance == null)
      m_instance = new Arm();
    return m_instance;
  }

  /** Creates a new Arm. */
  private Arm() {
    m_primaryMotor = new CANSparkMax(ArmConstants.PRIMARY_MOTOR_ID, MotorType.kBrushless);
    m_secondaryMotor = new CANSparkMax(ArmConstants.SECONDARY_MOTOR_ID, MotorType.kBrushless);
    m_secondaryMotor.follow(m_primaryMotor, true);
    m_primaryMotor.setSmartCurrentLimit(ArmConstants.MOTOR_CURRENT_LIMIT);
    m_primaryMotor.setIdleMode(IdleMode.kBrake);

    m_encoder = m_primaryMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_encoder.setPositionConversionFactor(ArmConstants.ENCODER_POSITION_CONVERTION_RATE);
    m_encoder.setZeroOffset(ArmConstants.ENCODER_ZERO_OFFSET);
    m_encoder.setInverted(ArmConstants.ENCODER_IS_INVERTED);

    m_pidController = m_primaryMotor.getPIDController();
    m_pidController.setP(ArmConstants.PID_P);
    m_pidController.setI(ArmConstants.PID_I);
    m_pidController.setD(ArmConstants.PID_D);
    m_pidController.setSmartMotionMaxVelocity(ArmConstants.PID_MAX_VELOCITY, 0);
    m_pidController.setSmartMotionMaxAccel(ArmConstants.PID_MAX_ACCELERATION, 0); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	  SmartDashboard.putNumber("Arm angle:", m_encoder.getPosition());
	  SmartDashboard.putBoolean("Is constrained", m_isConstrainted);
  }

  /**
   * Switches whether a constain is applied on the arm
   */
  public void switchConstraint() {
    m_isConstrainted = !m_isConstrainted;
  }

  /**
   * Sets the setpoint for the pid controller
   * @param setpoint The new setpoint
   */
  public void setReference(double reference) {
    m_pidController.setReference(reference, ControlType.kPosition);
  }

  /**
   * @return The arm's pid controller
   */
  public SparkPIDController getPidController() {
    return m_pidController;
  }

  /**
   * This method calculated the required angle based
   * on the distance of the robot from the april tag
   * @param Distance The robot's distance from the april tag
   * @return The angle required for the arm to reach
   */
  public double getAngleByDistanceSpeaker(double distance) {
    double[] distancePolindrom = ArmConstants.DISTANCE_TO_ANGLE_POLINDROM;
    double angle = 0;
    
    for (int i = 0; i < distancePolindrom.length; i++) {
      angle = distancePolindrom[i] * Math.pow(distance, i);
    }

    return angle;
  }

  /**
   * @return The current arm's angle.
   */
  public double getPosition() {
    return m_encoder.getPosition();
  }
}
