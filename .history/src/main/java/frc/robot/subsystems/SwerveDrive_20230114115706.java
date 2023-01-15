package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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

    public void drive(double speedMetersPerSecond, double directionRadians, double rotationRadiansPerSecond) {
        double vx = speedMetersPerSecond * Math.cos(directionRadians);
        double vy = speedMetersPerSecond * Math.sin(directionRadians);

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationRadiansPerSecond);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontRightState = states[0];
        SwerveModuleState frontLeftState = states[1];
        SwerveModuleState backRightState = states[2];
        SwerveModuleState backLeftState = states[3];

        /* Set swerve module speeds */
        frontRight.setSpeed(frontRightState.speedMetersPerSecond);
        frontLeft.setSpeed(frontLeftState.speedMetersPerSecond);
        backRight.setSpeed(backRightState.speedMetersPerSecond);
        backLeft.setSpeed(backLeftState.speedMetersPerSecond);

        /* Set swerve module angles */
        frontRight.setAngle(frontRightState.angle.getRadians());
        frontLeft.setAngle(frontLeftState.angle.getRadians());
        backRight.setAngle(frontLeftState.angle.getRadians())

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
