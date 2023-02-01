package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.subsystems.absencoder.AbsoluteEncoder;;

public class SwerveModule extends SubsystemBase {

  // Motors
  private final CANSparkMax driveMotor;
  private final CANSparkMax spinMotor;
  // Motor encoders
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absoluteEncoder;
  // Setpoint for absolute encoder
  private final double ENCODER_OFFSET;
  // PID controllers for rotation
  private final PIDController rotationalPidController;

  // Instance variables for returning values
  private Rotation2d currentWheelRotation;

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveModule(int driveID, int spinID, int encoderID, double absoluteEncoderOffset) {
    rotationalPidController = new PIDController(Constants.anglePIDDefaultValue, 0, 0);
    absoluteEncoder = new AbsoluteEncoder(encoderID);
    rotationalPidController.enableContinuousInput(-Math.PI, Math.PI);
    ENCODER_OFFSET = absoluteEncoderOffset;
    driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
    spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
    encoder = spinMotor.getEncoder();
    currentWheelRotation = new Rotation2d(encoder.getPosition());
  }

  public SwerveModule(Constants.SwerveModule moduleConfig) {
    this(moduleConfig.getDriveID(), moduleConfig.getSpinID(), moduleConfig.getEncoderID(),
        moduleConfig.getEncoderOffset());
    this.setName(moduleConfig.name());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    return currentWheelRotation;
  }

  public double getCurrentAngleRadians() {
    return currentWheelRotation.getRadians();
  }

  public double setAngle(double angleRadians) {
    double motorSpeed = rotationalPidController.calculate(getCurrentAngleRadians(), angleRadians);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond) * Constants.gearRatio;
    spinMotor.set(normalizedMotorSpinSpeed);
    return normalizedMotorSpinSpeed;
  }

  public double drive(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getCurrentRotation2d());
    // setting the angle
    double angleSetpoint = state.angle.getRadians();
    setAngle(angleSetpoint);

    double motorSpeed = state.speedMetersPerSecond / Constants.motorMaxSpeedMetersPerSecond;
    driveMotor.set(motorSpeed);
    return driveMotor.getEncoder().getVelocity();
  }

  public boolean resetSpinMotor() {
    double motorSpeed = rotationalPidController.calculate(absoluteEncoder.getPositionRadians(), ENCODER_OFFSET);
    double normalizedMotorSpinSpeed = (motorSpeed / Constants.spinMotorMaxSpeedMetersPerSecond) * Constants.gearRatio;
    spinMotor.set(normalizedMotorSpinSpeed);
    if (Math.abs(absoluteEncoder.getPositionRadians() - ENCODER_OFFSET) < 0.01) {
      encoder.setPosition(0);
      return true;
    }
    return false;
  }

  public void debugPrintValues() {
    SmartDashboard.putNumber(this.getName() + " Absolute Encoder Angle", absoluteEncoder.getPositionRadians() * 360);
    SmartDashboard.putNumber(this.getName() + " Relative Encoder Angle", encoder.getPosition() * 360);
    SmartDashboard.putNumber(this.getName() + " Angle (Degrees)", Units.radiansToDegrees(getCurrentAngleRadians()));
    SmartDashboard.putNumber("Front Left Encoder Velocity", driveMotor.getEncoder().getVelocity());
  }

  public void stop() {
    driveMotor.set(0);
    spinMotor.set(0);
  }
}
