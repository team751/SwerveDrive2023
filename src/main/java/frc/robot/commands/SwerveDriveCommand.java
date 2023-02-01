package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.absencoder.AbsoluteEncoder;
import frc.robot.subsystems.camera.Limelight;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.ComplementaryFilter;
import frc.robot.subsystems.gyro.Gyro;

public class SwerveDriveCommand extends CommandBase {
    // Swerb controller
    private final SwerveDrive swerveSubsystem;
    // Joystick Limiters
    private final SlewRateLimiter vxFLimiter;
    private final SlewRateLimiter vyFLimiter;
    // Gyroscope things
    private final ComplementaryFilter gyroFilter;
    private final Gyro rawGyro;
    // Camera
    private final Limelight limelight;
    // Auto Level
    private final PIDController levelPIDController;
    // Boolean values for mode toggles
    private boolean autoLevel;
    private boolean zeroModules;
    private double height;
    private double olderTime;
    private double deltaTime;
    private double netDistance;
    private double netSpeed;

    /**
     * Creates a new SwerveDriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SwerveDriveCommand(SwerveDrive subsystem) {
        rawGyro = new Gyro();
        netDistance = 0;
        netSpeed = 0;
        olderTime = 0;
        deltaTime = 0;
        swerveSubsystem = subsystem;
        addRequirements(subsystem);
        height = 0;
        // Gyroscope
        gyroFilter = new ComplementaryFilter();
        autoLevel = false;
        // Camera
        limelight = new Limelight();

        SmartDashboard.putNumber("Left Stick Angle (Radians)", 0);
        SmartDashboard.putNumber("Left Stick Magnitude", 0);
        SmartDashboard.putNumber("Right Stick Rotation", 0);

        vxFLimiter = new SlewRateLimiter(2);
        vyFLimiter = new SlewRateLimiter(2);
        levelPIDController = new PIDController(0.5, 0, 0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get Gyro Readings
        gyroFilter.debugAngle();

        // Get camera values
        limelight.debugDisplayValues();

        // Get Joystick Values
        double vx = vxFLimiter.calculate(Constants.driverController.getLeftX()); // TODO: Convert to m/s
        double vy = vyFLimiter.calculate(Constants.driverController.getLeftY()); // TODO: Convert to m/s
        double rotationsPerSecond = Constants.driverController.getRightX() * Constants.rotationsPerSecondMultiplier;

        // Speed deadband
        if (Math.sqrt(vx * vx + vy * vy) < 0.2) {
            vx = 0;
            vy = 0;
        }

        // TODO: weird behavior where this only works when "A" is held down
        // TODO: perhaps have the function return true if completed?
        if (Constants.driverController.getBButtonPressed()) {
            autoLevel = !autoLevel;
            olderTime = 0;
            netSpeed = 0;
            netDistance = 0;
            SmartDashboard.putString("Current Mode", "Auto Level");
        }

        if (Constants.driverController.getAButtonPressed()) {
            zeroModules = !zeroModules;
            SmartDashboard.putString("Current Mode", "Zeroing Modules");
        }

        if (zeroModules) {
            zeroModules = !swerveSubsystem.zeroModules();
        } else if (autoLevel) {
            autoLevel();
        } else {
            swerveSubsystem.drive(vx, vy, rotationsPerSecond);
        }

        // Smart dashboard controller
        SmartDashboard.putNumber("Left Stick Angle (Radians)", Math.atan2(vx, vy));
        SmartDashboard.putNumber("Left Stick Magnitude", Math.sqrt(vx * vx + vy * vy));
        SmartDashboard.putNumber("Right Stick Rotation", rotationsPerSecond);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    public void autoLevel() {
        if (olderTime == 0) {
            olderTime = System.currentTimeMillis();
            return;
        }
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        double[] angles = gyroFilter.getAngle();
        double[] accel = rawGyro.getAcceleration();
        double xSpeed = levelPIDController.calculate(angles[0], 0) / 2 / Math.PI;
        double ySpeed = levelPIDController.calculate(angles[1], 0) / 2 / Math.PI;
        double xComponent = accel[0] * Math.sin(angles[0])
                + accel[1] * Math.sin(angles[1])
                + accel[2] * Math.sin(Math.PI / 2 - angles[0]) * Math.sin(Math.PI / 2 - angles[1]);
        double netZAccel = (xComponent - 1) * 9.81; // m/s^2
        netSpeed += netZAccel * deltaTime;
        netDistance += netSpeed * deltaTime;
        System.out.println(netDistance);
        // TODO: fix this
        swerveSubsystem.drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
