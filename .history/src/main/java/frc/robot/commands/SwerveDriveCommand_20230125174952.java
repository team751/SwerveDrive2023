package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.Gyro;

public class SwerveDriveCommand extends CommandBase {
    private final SwerveDrive swerveSubsystem;
    private final SlewRateLimiter vxFLimiter;
    private final SlewRateLimiter vyFLimiter;

    /**
     * Creates a new SwerveDriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveDriveCommand(SwerveDrive subsystem) {
        swerveSubsystem = subsystem;
        addRequirements(subsystem);
        SmartDashboard.putNumber("Left Stick Angle (Radians)", 0);
        SmartDashboard.putNumber("Left Stick Magnitude", 0);
        SmartDashboard.putNumber("Right Stick Rotation", 0);

        vxFLimiter = new SlewRateLimiter(2);
        vyFLimiter = new SlewRateLimiter(2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get joystick values
        Gyro bob = new Gyro();
        bob.getAccel();
        double vx = vxFLimiter.calculate(Constants.driverController.getLeftX()); // TODO: Convert to m/s
        double vy = vyFLimiter.calculate(Constants.driverController.getLeftY()); // TODO: Convert to m/s
        double rotationsPerSecond = Constants.driverController.getRightX() * Constants.rotationsPerSecondMultiplier;

        // Speed deadband
        if (Math.sqrt(vx * vx + vy * vy) < 0.2) {
            vx = 0;
            vy = 0;
        }

        swerveSubsystem.drive(vx, vy, rotationsPerSecond);
        // Smart dashboard controller readouts
        SmartDashboard.putNumber("Left Stick Angle (Radians)", Math.atan2(vx, vy));
        SmartDashboard.putNumber("Left Stick Magnitude", Math.sqrt(vx * vx + vy * vy));
        SmartDashboard.putNumber("Right Stick Rotation", rotationsPerSecond);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
