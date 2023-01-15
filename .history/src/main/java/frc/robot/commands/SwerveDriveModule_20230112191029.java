package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

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
    float speed = (float)Constants.driverStick.getY() * 0.5f;
    float spin = (float)Constants.driverStick.getX() * -0.5f;

    if(!(speed > 0.05f) && !(speed < -0.05f)) speed = 0;
    m_subsystem.setSpeed();
    m_subsystem.setSpinSpeed();
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
