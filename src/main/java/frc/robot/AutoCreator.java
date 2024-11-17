package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCreator {
    //This class contains all the autonomous sequences
    //All commands should be public static
    private static TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
            ).setKinematics(DriveConstants.kDriveKinematics);

    
    public static Command getDrive1MeterCommand(DriveSubsystem m_robotDrive){
        Trajectory drive = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.00, 0.00, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.00, 0.00, new Rotation2d(0)), 
            config.setReversed(false));
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 
          AutoConstants.kIThetaController, 
          AutoConstants.kDThetaController, 
          AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController xController = new PIDController(
          AutoConstants.kPXController,
          AutoConstants.kIXController,
          AutoConstants.kDXController);

        PIDController yController = new PIDController(
          AutoConstants.kPYController,
          AutoConstants.kIYController,
          AutoConstants.kDYController);
          
        SwerveControllerCommand commandDrive = new SwerveControllerCommand(
          drive,
          m_robotDrive::getPose, 
          DriveConstants.kDriveKinematics, 
          xController, 
          yController, 
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);

        m_robotDrive.resetOdometry(drive.getInitialPose());
        m_robotDrive.resetEncoders();
        m_robotDrive.zeroHeading();
        return commandDrive;
    }

    public static Command getDrive1MeterDiagonallyCommand(DriveSubsystem m_robotDrive){
        Trajectory drive = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.00, 0.00, new Rotation2d(0)),
            List.of(),
            new Pose2d(1.00, 1.00, new Rotation2d(0)), 
            config.setReversed(false));
        
        ProfiledPIDController thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 
          AutoConstants.kIThetaController, 
          AutoConstants.kDThetaController, 
          AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController xController = new PIDController(
          AutoConstants.kPXController,
          AutoConstants.kIXController,
          AutoConstants.kDXController);

        PIDController yController = new PIDController(
          AutoConstants.kPYController,
          AutoConstants.kIYController,
          AutoConstants.kDYController);
          
        SwerveControllerCommand commandDrive = new SwerveControllerCommand(
          drive,
          m_robotDrive::getPose, 
          DriveConstants.kDriveKinematics, 
          xController, 
          yController, 
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);

        m_robotDrive.resetOdometry(drive.getInitialPose());
        m_robotDrive.resetEncoders();
        m_robotDrive.zeroHeading();
        return commandDrive;
    }
}
