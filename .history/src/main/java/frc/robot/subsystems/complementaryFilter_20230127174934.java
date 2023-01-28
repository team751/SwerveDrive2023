package frc.robot.subsystems;

import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class complementaryFilter {
    private Gyro sensor;
    private double[] anglePrevious;
    private double deltaTime;
    private double olderTime;

    public complementaryFilter() {
        sensor = new Gyro();
        sensor.calibrate();
        anglePrevious = new double[3];
        deltaTime = 0.05;
        olderTime = System.currentTimeMillis();
    }

    public double[] getAngle() {
        float oldTime = System.currentTimeMillis();
        double[] angle = new double[3];
        double[] gyro = sensor.getRotationalVelocity();
        double[] accel = sensor.getAccelerationAngle();
        angle[0] = 0.98 * (anglePrevious[0] + gyro[0] * deltaTime) + 0.02 * accel[0];
        angle[1] = 0.98 * (anglePrevious[1] + gyro[1] * deltaTime) + 0.02 * accel[1];
        angle[2] = 0.98 * (anglePrevious[2] + gyro[2] * deltaTime) + 0.02 * accel[2];
        anglePrevious[0] = angle[0];
        anglePrevious[1] = angle[1];
        anglePrevious[2] = angle[2];
        deltaTime = System.currentTimeMillis() - oldTime;
        return angle;
    }

}