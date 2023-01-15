package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveDriveSubsystem extends SubsystemBase {

  private final CANSparkMax driveMotor;
  private final CANSparkMax spinMotor;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem(int driveID, int spinID) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpeed(float speed){
    driveMotor.set(speed);
  }

  public void setSpinSpeed(float speed){
    spinMotor.set(speed);
  }
}
