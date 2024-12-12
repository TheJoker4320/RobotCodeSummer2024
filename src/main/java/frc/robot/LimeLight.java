// package frc.robot;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;

// public class LimeLight {
//     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

//     public boolean isTarget() {
//         return table.getEntry("tv").getDouble(0.0) == 1;
//     }

//     public double getLimeLightTX() {
//         return table.getEntry("tx").getDouble(0.0);
//     }
//     public double getLimeLightTY() {
//         return table.getEntry("ty").getDouble(0.0);
//     }
//     public double getLimeLightTL() {
//         return table.getEntry("t1").getDouble(0.0);
//     }
//     public double getLimeLightCL() {
//         return table.getEntry("c1").getDouble(0.0);
//     }
//     public double getLimeLightArea() {
//         return table.getEntry("ta").getDouble(0.0);
//     }
//     public int getId() {
//         return (int) table.getEntry("tid").getDouble(0.0);
//     }
    
//     public double getTrueDistance() {
//         double targetOffsetAngle_Vertical = getLimeLightTY();
//         double angleToGoalDegrees = Constants.LimelightConstants.LIMELIGHT_MOUNT_ANGLE + targetOffsetAngle_Vertical;
//         double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

//         double distanceFromRobotToGoalCentimeters = ((Constants.LimelightConstants.APRIL_TAGS_HEIGHT.get(getId()) - Constants.LimelightConstants.LIMELIGHT_HEIGHT_FROM_FLOOR) / Math.tan(angleToGoalRadians)) - Constants.LimelightConstants.LIMELIGHT_FROM_ROBOT_EDGE;
//         return distanceFromRobotToGoalCentimeters;
//     }
// }
