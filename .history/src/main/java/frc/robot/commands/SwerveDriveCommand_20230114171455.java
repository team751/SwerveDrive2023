package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
    private final SwerveDrive swerveSubsystem;

    /**
     * Creates a new SwerveDriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveDriveCommand(SwerveDrive subsystem) {
        swerveSubsystem = subsystem;
        addRequirements(subsystem);
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
        swerveSubsystem.drive(speed, angle, rotation);
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
