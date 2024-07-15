package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public boolean isTarget() {
        return table.getEntry("tv").getDouble(0.0) == 1;
    }

    public double getLimeLightTX() {
        return table.getEntry("tx").getDouble(0.0);
    }
    public double getLimeLightTY() {
        return table.getEntry("ty").getDouble(0.0);
    }
    public double getLimeLightTL() {
        return table.getEntry("t1").getDouble(0.0);
    }
    public double getLimeLightCL() {
        return table.getEntry("c1").getDouble(0.0);
    }
    public double getLimeLightArea() {
        return table.getEntry("ta").getDouble(0.0);
    }
    public int GetId() {
        return (int) table.getEntry("tid").getDouble(0.0);
    }
    // TODO: Add method to calculate true distance
}
