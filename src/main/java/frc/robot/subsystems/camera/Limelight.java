package frc.robot.subsystems.camera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {
    private final NetworkTable table;

    private final NetworkTableEntry target;
    private final NetworkTableEntry x;
    private final NetworkTableEntry y;
    private final NetworkTableEntry area;
    private final NetworkTableEntry skew;
    private final NetworkTableEntry cameraMode;

    public Limelight() { // TODO: limelight not reading from networkTables (photon vision?)
        table = NetworkTableInstance.getDefault().getTable("limelight");
        target = table.getEntry("tv");
        x = table.getEntry("tx");
        y = table.getEntry("ty");
        area = table.getEntry("ta");
        skew = table.getEntry("ts");
        cameraMode = table.getEntry("camMode");
    }

    public void debugDisplayValues() {
        SmartDashboard.putBoolean("target", target.getBoolean(false));
        SmartDashboard.putNumber("tx", x.getDouble(0));
        SmartDashboard.putNumber("ty", y.getDouble(0));
        SmartDashboard.putNumber("area", area.getDouble(0.0));
        SmartDashboard.putNumber("skew", skew.getDouble(0.0));
        SmartDashboard.putNumber("camMode", cameraMode.getDouble(0.0));
    }
}
