// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveArmToDegree;
import frc.robot.commands.SwitchArmIsConstrainted;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = DriveSubsystem.getInstance();
  private final Arm m_arm = Arm.getInstance();

  // Creating the Controllers
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final PS4Controller m_operatorController = new PS4Controller(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
                !m_driverController.getRawButton(XboxController.Button.kRightBumper.value), false,
                 true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    JoystickButton raiseArmBtn30Degree = new JoystickButton(m_operatorController, OperatorConstants.RAISE_ARM_BUTTON_30);
    raiseArmBtn30Degree.onTrue(new MoveArmToDegree(m_arm, Constants.ArmConstants.DEGREE_30));
    JoystickButton raiseArmBtn70Degree = new JoystickButton(m_operatorController, OperatorConstants.RAISE_ARM_BUTTON_70);
    raiseArmBtn70Degree.onTrue(new MoveArmToDegree(m_arm, Constants.ArmConstants.DEGREE_70));
    JoystickButton raiseArmBtn90Degree = new JoystickButton(m_operatorController, OperatorConstants.RAISE_ARM_BUTTON_90);
    raiseArmBtn90Degree.onTrue(new MoveArmToDegree(m_arm, Constants.ArmConstants.DEGREE_90));
    JoystickButton switchArmIsConstrainted = new JoystickButton(m_operatorController, OperatorConstants.SWITCH_ARM_CONSTRAINT);
    switchArmIsConstrainted.onTrue(new SwitchArmIsConstrainted(m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
}
