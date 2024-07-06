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

public class Arm extends SubsystemBase {

  private final CANSparkMax m_primaryMotor;
  private final CANSparkMax m_secondaryMotor;
  private final AbsoluteEncoder m_encoder;

  private final PIDController m_pidController;

  private boolean m_isConstrained;
  
  private static Arm m_instance = null;

  /**
   * Singleton implementation
   * 
   * @return The only instance of the Arm class
   */
  public Arm getInstance() {
    if (m_instance == null)
      m_instance = new Arm();
    return m_instance;
  }

  /** Creates a new Arm. */
  private Arm() {
    m_primaryMotor = new CANSparkMax(1/*Place the primary motor id here*/, MotorType.kBrushless);
    m_secondaryMotor = new CANSparkMax(2/*Place the primary motor id here*/, MotorType.kBrushless);
    m_secondaryMotor.follow(m_primaryMotor);
    m_primaryMotor.setSmartCurrentLimit(50/*Place the current limit here*/);
    m_primaryMotor.setIdleMode(IdleMode.kBrake);

    m_encoder = m_primaryMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_encoder.setPositionConversionFactor(1/*Place positing conversion factor here*/);
    m_encoder.setZeroOffset(0/*Place encoder zero offset here*/);
    m_encoder.setInverted(false/*place encoder inverted here*/);

    m_pidController = new PIDController(0/*Place pid p here*/, 0/*Place pid i here*/, 0/*Place pid d here*/);
    m_pidController.setTolerance(0/*Place pid tolerance here*/);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	  SmartDashboard.putNumber("Arm angle:", m_encoder.getPosition());
	  SmartDashboard.putBoolean("Is constrained", m_isConstrained);
  }

  /**
   * Switches whether a constain is applied on the arm
   */
  public void switchConstraint() {
    m_isConstrained = !m_isConstrained;
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
    double[] distancePolindrom = {1};
    double angle = 0;
    
    for (int i = 0; i < distancePolindrom.length; i++) {
      angle = distancePolindrom[i] * Math.pow(distance, i + 1);
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
   * sure that the arm is constrained according to the constrains
   * @param speed The speed to set to the arm.
   */
  public void setSpeed(double speed) {
		if(m_isConstrained){
			if(!((getPosition() > 0/*Place here lower-bound constrain*/ && speed > 0 && getPosition() < 350) ||
				   ((getPosition() < 90/*Place here higher-bound constrain*/ || getPosition() > 350) && speed < 0 ))) {
					if((getPosition() < 30/*Place here near-lower-bound constrain*/ && speed < 0) || (getPosition() > 70/*Place here near-higher-bound constrain*/ && speed > 0)) {
						m_primaryMotor.set(speed * 0.5/*Place here the arn's speed when near constrain*/);
					}
					else {
						m_primaryMotor.set(speed * 1/*Place here to the arm's speed*/);
					}	
			}
			else {
				m_primaryMotor.set(0);
			}
		}
		else {
			m_primaryMotor.set(speed * 1/*Place here to the arm's speed - unconstrained*/);
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
		output = output > 0.4/*Place here max speed*/ ? 0.4/*Place here max speed*/ : output;
		output = output < -0.4/*Place here max speed*/ ? -0.4/*Place here max speed*/ : output;
		m_primaryMotor.set(output);
	}

  /**
   * @return Whether the arm is at the setpoint given previously.
   */
  public boolean atSetpoint(){
		return m_pidController.atSetpoint();
	}
}
