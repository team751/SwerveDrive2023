package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private final int LSM9DS1_ADDRESS = 0x6b;

    private static final int LSM9DS1_WHO_AM_I = 0x0f;
    private static final int LSM9DS1_CTRL_REG1_G = 0x10;
    private static final int LSM9DS1_STATUS_REG = 0x17;
    private static final int LSM9DS1_OUT_X_G = 0x18;
    private static final int LSM9DS1_CTRL_REG6_XL = 0x20;
    private static final int LSM9DS1_CTRL_REG8 = 0x22;
    private static final int LSM9DS1_OUT_X_XL = 0x28;

    // magnetometer
    private final int LSM9DS1_ADDRESS_M = 0x1e;

    private static final int LSM9DS1_CTRL_REG1_M = 0x20;
    private static final int LSM9DS1_CTRL_REG2_M = 0x21;
    private static final int LSM9DS1_CTRL_REG3_M = 0x22;
    private static final int LSM9DS1_STATUS_REG_M = 0x27;
    private static final int LSM9DS1_OUT_X_L_M = 0x28;
    private final I2C sensor;
    private final I2C magnetometer;

    public Gyro() {
        sensor = new I2C(I2C.Port.kOnboard, LSM9DS1_ADDRESS);
        magnetometer = new I2C(I2C.Port.kOnboard, LSM9DS1_ADDRESS_M);
        reset();
        configure();
    }

    public void reset() {
        sensor.write(LSM9DS1_CTRL_REG8, 0x05);
        magnetometer.write(LSM9DS1_CTRL_REG2_M, 0x0c);
    }

    public void configure() {
        sensor.write(LSM9DS1_CTRL_REG1_G, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
        sensor.write(LSM9DS1_CTRL_REG6_XL, 0x70); // 119 Hz, 4g
        sensor.write(0x2E, 0xC0); // FIFO
        sensor.write(0x23, 0x02); // continuous mode
        magnetometer.write(LSM9DS1_CTRL_REG1_M, 0xb4);
        magnetometer.write(LSM9DS1_CTRL_REG2_M, 0x00);
        magnetometer.write(LSM9DS1_CTRL_REG3_M, 0x00);
    }

    public double[] getRotationalVelocity() {
        double[] rotVel = new double[3];
        ByteBuffer buf = ByteBuffer.allocate(6);
        sensor.read(0x18, 6, buf);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        int x = buf.getShort(4); // CHECK THIS!! THEY MIGHT BE IN THE WRONG ORDER
        int y = buf.getShort(2);
        int z = buf.getShort(0);
        rotVel[0] = x * 2000.0 / 32768.0;
        rotVel[1] = y * 2000.0 / 32768.0;
        rotVel[2] = z * 2000.0 / 32768.0;
        SmartDashboard.putNumber("rotX", rotVel[0]);
        SmartDashboard.putNumber("rotY", rotVel[1]);
        SmartDashboard.putNumber("rotZ", rotVel[2]);
        return rotVel;
    }

    public double[] getAcceleration() {
        double[] accel = new double[3];
        ByteBuffer buf = ByteBuffer.allocate(6);
        sensor.read(LSM9DS1_OUT_X_XL, 6, buf);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        int x = buf.getShort(4);
        int y = buf.getShort(2);
        int z = buf.getShort(0);
        accel[0] = x * 4.0 / 32768.0;
        accel[1] = y * 4.0 / 32768.0;
        accel[2] = z * 4.0 / 32768.0;
        SmartDashboard.putNumber("accelX", accel[0]);
        SmartDashboard.putNumber("accelY", accel[1]);
        SmartDashboard.putNumber("accelZ", accel[2]);
        return accel;
    }
}
