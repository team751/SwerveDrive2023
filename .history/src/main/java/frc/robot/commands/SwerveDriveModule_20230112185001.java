package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveModule extends CommandBase {
  private final SwerveDriveSubsystem m_subsystem;

  /**
   * Creates a new SwerveDriveModule.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDriveModule(SwerveDriveSubsystem subsystem) {
    m_subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setSpeed(driverStick.getY() * 0.5f);
    m_subsystem.setSpinSpeed(driverStick.getX() * -0.5f);
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
