package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    private RelativeEncoder encoder;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem(RelativeEncoder encoder) {
    this.encoder = encoder;
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
