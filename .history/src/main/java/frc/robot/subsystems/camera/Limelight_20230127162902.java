package frc.robot.subsystems.camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTable table;

    private final NetworkTableEntry target;
    private final NetworkTableEntry x;
    private final NetworkTableEntry y;
    private final NetworkTableEntry area;
    private final NetworkTableEntry skew;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        target = table.getEntry("tv");
        x = table.getEntry("tx");
        y = table.getEntry("ty");
        area = table.getEntry("ta");
        skew = table.getEntry("ts");
    }
}
