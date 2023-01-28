package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    private final I2C sensor;

    public Gyro() {
        sensor = new I2C(I2C.Port.kOnboard, 0x6b);
        reset();
        configure();
    }

    public void reset() {
        sensor.write(0x22, 0x05);
    }

    public void configure() {
        sensor.write(0x10, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
        sensor.write(0x20, 0x70); // 119 Hz, 4g

        sensor.write(0x2E, 0xC0);
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
    }
}
