package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.SpinMotorEncoder;

public class SwerveDriveSubsystem extends SubsystemBase {

  public final CANSparkMax driveMotor;
  public final CANSparkMax spinMotor;
  public final SpinMotorEncoder encoder;

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

  public void follow(SwerveDriveSubsystem leader){
    driveMotor.follow(leader.driveMotor);
    spinMotor.follow(leader.spinMotor);
  }
}
