package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        SmartDashboard.putBoolean("Front Right Motor", false);
        SmartDashboard.putBoolean("Front Left Motor", false);
        SmartDashboard.putBoolean("Back Right Motor", false);
        SmartDashboard.putBoolean("Back Left Motor", false);
        SmartDashboard.putBoolean("Enable All Motors", true);
    }

    public void drive(double speedMetersPerSecond, double directionRadians, double rotationRadiansPerSecond) {
        // Joystick values to a speed vector
        double vx = speedMetersPerSecond * Math.cos(directionRadians);
        double vy = speedMetersPerSecond * Math.sin(directionRadians);

        // Convert speed vector and rotation to module speeds
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, rotationRadiansPerSecond);
        // Unpack module speeds
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontRightState = states[0];
        SwerveModuleState frontLeftState = states[1];
        SwerveModuleState backRightState = states[2];
        SwerveModuleState backLeftState = states[3];

        SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(frontRightState,
                new Rotation2d(frontRight.getCurrentAngleRadians()));
        SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeftState,
                new Rotation2d(frontLeft.getCurrentAngleRadians()));
        SwerveModuleState backRightOptimized = SwerveModuleState.optimize(backRightState,
                new Rotation2d(backRight.getCurrentAngleRadians()));
        SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(backLeftState,
                new Rotation2d(backLeft.getCurrentAngleRadians()));

        if (speedMetersPerSecond < 0.1 && rotationRadiansPerSecond < 0.1) {
            stop();
            return;
        }

        // Override to enable all motors at once on SmartDashboard, on by default
        boolean enableAllMotors = SmartDashboard.getBoolean("Enable All Motors", true);
        /* Set swerve module speeds and rotations, also put them to smartdashboard */
        if (SmartDashboard.getBoolean("Front Right Motor", false) || enableAllMotors) {
            frontRight.setSpeed(frontRightOptimized.speedMetersPerSecond);
            double spinSpeed = frontRight.setAngle(frontRightOptimized.angle.getRadians());
            SmartDashboard.putNumber("Front Right Spin Speed", spinSpeed);
            SmartDashboard.putNumber("Front Right Angle (Degrees)",
                    Units.radiansToDegrees(frontRight.getCurrentAngleRadians()));
        }
        if (SmartDashboard.getBoolean("Front Left Motor", false) || enableAllMotors) {
            frontLeft.setSpeed(frontLeftOptimized.speedMetersPerSecond);
            double spinSpeed = frontLeft.setAngle(frontLeftOptimized.angle.getRadians());
            SmartDashboard.putNumber("Front Left Spin Speed", spinSpeed);
            SmartDashboard.putNumber("Front Left Angle (Degrees)",
                    Units.radiansToDegrees(frontLeft.getCurrentAngleRadians()));
        }
        if (SmartDashboard.getBoolean("Back Right Motor", false) || enableAllMotors) {
            backRight.setSpeed(backRightOptimized.speedMetersPerSecond);
            double spinSpeed = backRight.setAngle(backRightOptimized.angle.getRadians());
            SmartDashboard.putNumber("Back Right Spin Speed", spinSpeed);
            SmartDashboard.putNumber("Back Right Angle (Degrees)",
                    Units.radiansToDegrees(backRight.getCurrentAngleRadians()));
        }
        if (SmartDashboard.getBoolean("Back Left Motor", false) || enableAllMotors) {
            backLeft.drive(backLeftState);
            SmartDashboard.putNumber("Back Left Spin Speed", spinSpeed);
            SmartDashboard.putNumber("Back Left Angle (Degrees)",
                    Units.radiansToDegrees(backLeft.getCurrentAngleRadians()));
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}