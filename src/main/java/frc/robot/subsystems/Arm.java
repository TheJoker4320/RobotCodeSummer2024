// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_secondaryMotor;
  private final AbsoluteEncoder m_encoder;

  private final PIDController m_pidController;

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
    m_secondaryMotor.follow(m_primaryMotor);
    m_primaryMotor.setSmartCurrentLimit(ArmConstants.MOTOR_CURRENT_LIMIT);
    m_primaryMotor.setIdleMode(IdleMode.kBrake);

    m_encoder = m_primaryMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_encoder.setPositionConversionFactor(ArmConstants.ENCODER_POSITION_CONVERTION_RATE);
    m_encoder.setZeroOffset(ArmConstants.ENCODER_ZERO_OFFSET);
    m_encoder.setInverted(ArmConstants.ENCODER_IS_INVERTED);

    m_pidController = new PIDController(ArmConstants.PID_P, ArmConstants.PID_I, ArmConstants.PID_D);
    m_pidController.setTolerance(ArmConstants.PID_TOLERANCE);
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
  public void setSetpoint(double setpoint) {
    m_pidController.setSetpoint(setpoint);
  }

  /**
   * @return The arm's pid controller
   */
  public PIDController getPidController() {
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

  /**
   * This method sets the speed for the arm and makes
   * sure that the arm is constrainted according to the constraints
   * @param speed The speed to set to the arm.
   */
  public void setSpeed(double speed) {
		if(m_isConstrainted){
			if(!((getPosition() > ArmConstants.HIGHER_BOUND_CONSTRAINT && speed > 0 && getPosition() < 350) ||
				   ((getPosition() < ArmConstants.LOWER_BOUND_CONSTRAINT || getPosition() > 350) && speed < 0 ))) {
					if((getPosition() < ArmConstants.NEAR_LOWER_BOUND_CONSTRAINT && speed < 0) || (getPosition() > ArmConstants.NEAR_HIGHER_BOUND_CONSTRAINT && speed > 0)) {
						m_primaryMotor.set(speed * ArmConstants.SLOW_SPEED);
					}
					else {
						m_primaryMotor.set(speed * ArmConstants.CONSTRAINTED_SPEED);
					}	
			}
			else {
				m_primaryMotor.set(0);
			}
		}
		else {
			m_primaryMotor.set(speed * ArmConstants.UNCONSTRAINTED_SPEED);
		}
	}

  /**
   * Stops the arm
   */
  public void stop() {
    setSpeed(0);
  }

  /**
   * This method give the pid controller the measurement
   * and lets it control the motor
   * @param measurement The measurement given to the pid controller
   */
  public void setSpeedByMeasurement(double measurement){
		double output = m_pidController.calculate(measurement);
		output = output >(ArmConstants.UNCONSTRAINTED_SPEED) ? (ArmConstants.UNCONSTRAINTED_SPEED) : output;
		output = output < -(ArmConstants.UNCONSTRAINTED_SPEED) ? -(ArmConstants.UNCONSTRAINTED_SPEED) : output;
		m_primaryMotor.set(output);
	}

  /**
   * @return Whether the arm is at the setpoint given previously.
   */
  public boolean atSetpoint(){
		return m_pidController.atSetpoint();
	}
}
