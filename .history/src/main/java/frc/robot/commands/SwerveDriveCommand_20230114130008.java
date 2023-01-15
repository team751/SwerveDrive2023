package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;
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
        double speed = Constants.driverJoystick.getMagnitude();
        double rotation = Constants.driverStick.getRightX();

        swerveSubsystem.drive(speed, angle, rotation);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
