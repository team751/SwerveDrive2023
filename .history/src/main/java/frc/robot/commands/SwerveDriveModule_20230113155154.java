package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SwerveDriveModule extends CommandBase {
    SwerveDriveSubsystem frontLeftSub = new SwerveDriveSubsystem(10,11);
    SwerveDriveSubsystem backLeftSub = new SwerveDriveSubsystem(12,13);
    SwerveDriveSubsystem backRightSub = new SwerveDriveSubsystem(14,15); 
    SwerveDriveSubsystem frontRightSub = new SwerveDriveSubsystem(16,17);

  /**
   * Creates a new SwerveDriveModule.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDriveModule(SwerveDriveSubsystem frontLeft, SwerveDriveSubsystem frontRight, SwerveDriveSubsystem backLeft, SwerveDriveSubsystem backRight) {
    frontLeftSub = frontLeft;
    frontRightSub = frontRight;
    backLeftSub = backLeft;
    backRightSub = backRight;
    
    addRequirements(frontLeftSub);
    addRequirements(frontRightSub);
    addRequirements(backLeftSub);
    addRequirements(backRightSub);
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
    if(!(spin > 0.05f) && !(spin < -0.05f)) spin = 0;
    m_subsystem.setSpeed(speed);
    m_subsystem.setSpinSpeed(spin);
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
