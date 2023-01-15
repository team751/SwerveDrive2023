package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

public class SpinMotorEncoder extends SubsystemBase {
    private RelativeEncoder encoder;

  /** Creates a new ExampleSubsystem. */
  public SpinMotorEncoder(CANEncoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("EncoderValue", encoder.getRotation());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



}
