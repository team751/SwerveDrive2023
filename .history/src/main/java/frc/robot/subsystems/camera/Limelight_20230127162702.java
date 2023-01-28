package frc.robot.subsystems.camera;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final NetworkTable table;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
}
