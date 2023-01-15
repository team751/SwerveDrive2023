package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private SwerveDriveSubsystem frontLeft;
    private SwerveDriveSubsystem frontRight;
    private SwerveDriveSubsystem backLeft;
    private SwerveDriveSubsystem backRight;

    private SwerveDriveKinematics kinematics;

    /** Creates a new ExampleSubsystem. */
    public SwerveDrive(SwerveDriveSubsystem frontLeft, SwerveDriveSubsystem frontRight, SwerveDriveSubsystem backLeft,
            SwerveDriveSubsystem backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        kinematics = new SwerveDriveKinematics(Constants.frontRightOffsetMeters, Constants.frontLeftOffsetMeters,
                Constants.backRightOffsetMeters, Constants.backLeftOffsetMeters);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
