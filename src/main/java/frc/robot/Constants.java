// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.PS4Controller;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int RAISE_ARM_BUTTON = PS4Controller.Button.kR2.value;
        public static final int LOWER_ARM_BUTTON = PS4Controller.Button.kL2.value;
        public static final int SWITCH_ARM_CONSTRAINT = PS4Controller.Button.kL1.value;
        public static final int CLIMB_BUTTON = PS4Controller.Button.kOptions.value;
      public static final int COLLECT_BUTTON = PS4Controller.Button.kCross.value;
      public static final int EJECT_BUTTON = PS4Controller.Button.kSquare.value;
      public static final int SHOOT_BUTTON = PS4Controller.Button.kCircle.value;
    public static double kDriveDeadband = 0.1;
    }
  
	public static class ModuleConstants {
      // The MAXSwerve module can be configured with one of three pinion gears: 12T,
      // 13T, or 14T.
      // This changes the drive speed of the module (a pinion gear with more teeth
      // will result in a
      // robot that drives faster).
      public static final int kDrivingMotorPinionTeeth = 14;

      // Invert the turning encoder, since the output shaft rotates in the opposite
      // direction of
      // the steering motor in the MAXSwerve Module.
      public static final boolean kTurningEncoderInverted = true;

      // Calculations required for driving motor conversion factors and feed forward
      public static final double kFreeSpeedRpm = 5676;
      public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60;
      public static final double kWheelDiameterMeters = 0.0762;
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
      // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
      // teeth on the bevel pinion
      public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
      public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
          / kDrivingMotorReduction;

      public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
          / kDrivingMotorReduction; // meters
      public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
          / kDrivingMotorReduction) / 60.0; // meters per second

      public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

      public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
      public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

      public static final double[] kDrivingP = {0.04, 0.04, 0.04, 0.04};
      public static final double[] kDrivingI = {0, 0, 0, 0};
      public static final double[] kDrivingD = {0, 0, 0, 0};
      public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;
      
      public static final double kTurningP = 1;
      public static final double kTurningI = 0.002;
      public static final double kTurningD = 0;
      public static final double kTurningFF = 0;
      public static final double kTurningIZone = 0.08;
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;

      public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
      public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

      public static final int kDrivingMotorCurrentLimit = 50; // amps
      public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    

  public static final class DriveConstants {
      // Driving Parameters - Note that these are not the maximum capable speeds of
      // the robot, rather the allowed maximum speeds
      public static final double kMaxSpeedMetersPerSecond = 4.8;
      public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

      public static final double kDirectionSlewRate = 1.2; // radians per second
      public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
      public static final double kRotationalSlewRate = 3; // percent per second (1 = 100%)

      // Chassis configuration
      public static final double kTrackWidth = 0.675;
      // Distance between centers of right and left wheels on robot
      public static final double kWheelBase = 0.675;
      // Distance between front and back wheels on robot
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
      public static final double ROBOT_LENGTH = 30;

      // Angular offsets of the modules relative to the chassis in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      public static final double kFrontRightChassisAngularOffset = 0;
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;

      // SPARK MAX CAN IDs
      public static final int kFrontLeftDrivingCanId = 8;
      public static final int kRearLeftDrivingCanId = 6;
      public static final int kFrontRightDrivingCanId = 2;
      public static final int kRearRightDrivingCanId = 4;

      public static final int kFrontLeftTurningCanId = 7;
      public static final int kRearLeftTurningCanId = 5;
      public static final int kFrontRightTurningCanId = 1;
      public static final int kRearRightTurningCanId = 3;

      public static final int kFrontLeftModuleId = 0;
      public static final int kRearLeftModuleId = 2;
      public static final int kFrontRightModuleId = 1;
      public static final int kRearRightModuleId = 3;

      public static final boolean kGyroReversed = false;
    }

    public static final class ArmConstants {
        public static final int PRIMARY_MOTOR_ID = 10;
        public static final int SECONDARY_MOTOR_ID = 9;

        public static final int MOTOR_CURRENT_LIMIT = 20;

        public static final double ENCODER_POSITION_CONVERTION_RATE = 360;
        public static final double ENCODER_ZERO_OFFSET = 377.6;
        public static final boolean ENCODER_IS_INVERTED = true;

        public static final double PID_P = 0.05;
        public static final double PID_I = 0;
        public static final double PID_D = 0;
        public static final double PID_TOLERANCE = 0.5;

        public static final double[] DISTANCE_TO_ANGLE_POLINDROM = {(-4.28), (8.27E-3), (2.53E-5), (-2.02E8), (6.2E-12), (-6.06E-16)};
    
        public static final double LOWER_BOUND_CONSTRAINT = 3;
        public static final double NEAR_LOWER_BOUND_CONSTRAINT = 30;
        public static final double HIGHER_BOUND_CONSTRAINT = 90;
        public static final double NEAR_HIGHER_BOUND_CONSTRAINT = 70;

        public static final double SLOW_SPEED = 0.1;
        public static final double CONSTRAINTED_SPEED = 0.75;
        public static final double UNCONSTRAINTED_SPEED = 0.4;
    }


    public static class CollectorConstants {
        public static final int LIMIT_SWITCH_PORT = 0;
        public static final int PRIMARY_MOTOR_PORT = 15;
        public static final double COLLECTOR_SPEED = -0.7;
    }


    public static class ShooterConstants{
        public static final int MASTER_MOTOR_PORT = 14;
        public static final int SLAVE_MOTOR_PORT = 13;
        public static final double kP = 0.34;
        public static final double kI = 0.05;
        public static final double kD = 0;
        public static final int SHOOTER_ENCODER_PORT_A = 1;
		public static final int SHOOTER_ENCODER_PORT_B = 2;
    }

    public static class ClimberConstants{
        public static final int PRIMARY_MOTOR_ID = 11;
        public static final int SECONDARY_MOTOR_ID = 12;
        public static final int SMART_CURRENT_LIMIT = 50;
        public static final double CLIMB_SPEED = 0.5;

    }
    public static final class LimelightConstants {
        public static final double LIMELIGHT_HEIGHT_FROM_FLOOR = 27;
        public static final double LIMELIGHT_MOUNT_ANGLE = 0;
        public static final double LIMELIGHT_FROM_ROBOT_EDGE = 16.2;
        public static final Map<Integer, Double> APRIL_TAGS_HEIGHT = new HashMap<>();

    static {
        for (int i = 1; i <= 16; i++) {
            APRIL_TAGS_HEIGHT.put(i, 0.0);
        }}
    }
}
