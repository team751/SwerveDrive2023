package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  public Rotation2d currentWheelRotation;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem(int driveID, int spinID) {
    pidController = new PIDController(Constants.anglePIDDefaultValue, 0, 0);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
    encoder = spinMotor.getEncoder();
    encoder.setPositionConversionFactor(2 * Math.PI / Constants.gearRatio);
    encoder.setPosition(0);
    currentWheelRotation = new Rotation2d(encoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Readout", encoder.getPosition());
    currentWheelRotation = new Rotation2d(encoder.getPosition());
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

  public Rotation2d getCurrentRotation2d() {
    return new Rotation2d(currentWheelAngleRadians);
  }

  public double getCurrentAngleRadians() {
    return currentWheelAngleRadians;
  }

  public double setAngle(double angleRadians) {
    double motorSpeed = pidController.calculate(getCurrentAngleRadians(), angleRadians);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond) * Constants.gearRatio;
    spinMotor.set(normalizedMotorSpinSpeed);
    return normalizedMotorSpinSpeed;
  }

  public void drive(SwerveModuleState state) {
    SwerveModuleState.optimize(state, getCurrentRotation2d());

    // setting the angle
    double angleSetpoint = state.angle.getRadians();
    setAngle(angleSetpoint);

    double motorSpeed = state.speedMetersPerSecond / Constants.motorMaxSpeedMetersPerSecond;
  }

  public void stop() {
    driveMotor.set(0);
    spinMotor.set(0);
  }
}
