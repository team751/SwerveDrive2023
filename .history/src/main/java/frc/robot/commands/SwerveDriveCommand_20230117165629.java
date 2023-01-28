package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
    private final SwerveDrive swerveSubsystem;
    private double oldAngle = 0;

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
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get joystick values
        double speed = Constants.driverJoystick.getMagnitude(); // TODO: Convert to m/s
        double angle = Constants.driverJoystick.getDirectionRadians(); // TODO: Figure out fwd angle of joystick
        double rotation = Constants.driverController.getRightX();

        // Speed deadband
        if (Math.abs(speed) < 0.2) {
            speed = 0;
        }
        // if (angle == 0 || angle % 360 == 0) {
        // angle = oldAngle;
        // }
        swerveSubsystem.drive(speed, angle, rotation);
        // Smart dashboard controller readouts
        SmartDashboard.putNumber("Left Stick Angle (Radians)", angle);
        SmartDashboard.putNumber("Left Stick Magnitude", speed);
        SmartDashboard.putNumber("Right Stick Rotation", rotation);
        oldAngle = angle;
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
