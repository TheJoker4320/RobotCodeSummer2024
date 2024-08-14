// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.SwitchArmIsConstrainted;
import frc.robot.subsystems.Arm;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Climber;
import frc.robot.commands.Shoot;
import frc.robot.commands.Collect;
import frc.robot.commands.Eject;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final Climber m_climber = Climber.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Collector m_collector = Collector.getInstance();
  private final SendableChooser<Command> m_chooser;

  // Creating the Controllers
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final PS4Controller m_operatorController = new PS4Controller(OperatorConstants.kOperatorControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //initializing autonomous chooser and adding options
    m_chooser = new SendableChooser<>();

    m_chooser.addOption("Wait", new WaitCommand(0.0));
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
    JoystickButton shootButton = new JoystickButton(m_operatorController, OperatorConstants.SHOOT_BUTTON);
    shootButton.whileTrue(new Shoot(m_shooter, 60));
    JoystickButton collectButton = new JoystickButton(m_operatorController, OperatorConstants.COLLECT_BUTTON);
    collectButton.onTrue(new Collect(m_collector));
    JoystickButton ejectButton = new JoystickButton(m_operatorController, OperatorConstants.EJECT_BUTTON);
    ejectButton.whileTrue(new Eject(m_collector));
    JoystickButton climbButton = new JoystickButton(m_operatorController, OperatorConstants.CLIMB_BUTTON);
    climbButton.whileTrue(new Climb(m_climber));
    JoystickButton raiseArmBtn = new JoystickButton(m_operatorController, OperatorConstants.RAISE_ARM_BUTTON);
    raiseArmBtn.whileTrue(new MoveArm(m_arm, false));
    JoystickButton lowerArmBtn = new JoystickButton(m_operatorController, OperatorConstants.LOWER_ARM_BUTTON);
    lowerArmBtn.whileTrue(new MoveArm(m_arm, true));
    JoystickButton switchArmIsConstrainted = new JoystickButton(m_operatorController, OperatorConstants.SWITCH_ARM_CONSTRAINT);
    switchArmIsConstrainted.onTrue(new SwitchArmIsConstrainted(m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //reset all robot encoders, odometry and heading for autonomous
    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    return m_chooser.getSelected();
  }
}
