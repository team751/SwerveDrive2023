package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {

  public final CANSparkMax driveMotor;
  public final CANSparkMax spinMotor;
  public final RelativeEncoder encoder;
  public final PIDController pidController;
  public double currentWheelAngleRadians;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem(int driveID, int spinID) {
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
    encoder = spinMotor.getEncoder();
    pidController = new PIDController(SmartDashboard.getNumber("Proportion", 0.3), 0, 0);
    currentWheelAngleRadians = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Readout", encoder.getPosition());
    currentWheelAngleRadians = (encoder.getPosition() / Constants.gearRatio) % (2 * Math.PI);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpeed(double speedMetersPerSecond) {
    double motorSpeed = speedMetersPerSecond / Constants.motorMaxSpeedMetersPerSecond;
    driveMotor.set(motorSpeed);
  }

  public void setSpinSpeed(double speed) {
    spinMotor.set(speed);
  }

  public void setAngle(double angleRadians) {
    double amountToSpinWheel = (angleRadians - currentWheelAngleRadians > Math.PI)
        ? angleRadians - currentWheelAngleRadians - Math.PI
        : (angleRadians - currentWheelAngleRadians < -Math.PI)
            ? angleRadians - currentWheelAngleRadians + Math.PI
            : angleRadians - currentWheelAngleRadians;
    double amountToSpinMotor = angleRadians * Constants.gearRatio;
    double motorSpeed = pidController.calculate(currentWheelAngleRadians, amountToSpinMotor);
    spinMotor.set((motorSpeed > 0) ? Math.min(motorSpeed, 1.0) * 0.1 : Math.max(motorSpeed, -1.0) * 0.1));
  }

  public void follow(SwerveDriveSubsystem leader) {
    driveMotor.follow(leader.driveMotor);
    spinMotor.follow(leader.spinMotor);
  }
}
