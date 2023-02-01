package frc.robot.subsystems.gyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Uses multiple inputs from the gyroscope to make the values more accurate
public class ComplementaryFilter {
    private Gyro sensor;
    private double[] anglePrevious;
    private double deltaTime;
    private double olderTime;
    private final double percentage = 0.02;

    public ComplementaryFilter() {
        sensor = new Gyro();
        sensor.calibrate();
        anglePrevious = new double[3];
        deltaTime = 0.05;
        olderTime = System.currentTimeMillis();
    }

    public double[] getAngle() {
        deltaTime = (System.currentTimeMillis() - olderTime) / 1000;
        olderTime = System.currentTimeMillis();
        double[] angle = new double[3];
        double[] gyro = sensor.getRotationalVelocity();
        double[] accel = sensor.getAccelerationAngle();
        angle[0] = (1 - percentage) * (anglePrevious[0] + gyro[0] * deltaTime) + percentage * accel[0];
        angle[1] = (1 - percentage) * (anglePrevious[1] + gyro[1] * deltaTime) + percentage * accel[1];
        angle[2] = (1 - percentage) * (anglePrevious[2] + gyro[2] * deltaTime) + percentage * accel[2];
        anglePrevious[0] = angle[0];
        anglePrevious[1] = angle[1];
        anglePrevious[2] = angle[2];
        angle[0] = Math.toRadians(angle[0]);
        angle[1] = Math.toRadians(angle[1]);
        angle[2] = Math.toRadians(angle[2]);
        return angle;
    }

    public void debugAngle() {
        double[] angle = getAngle();
        SmartDashboard.putNumber("filteredX", angle[0]);
        SmartDashboard.putNumber("FilteredY", angle[1]);
    }

}