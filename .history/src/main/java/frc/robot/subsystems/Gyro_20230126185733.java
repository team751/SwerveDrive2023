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

    public void getAccel() {
        ByteBuffer buf = ByteBuffer.allocate(3);
        sensor.read(0x18, 6, buf);
        int x = buf.getShort(0);
        int y = buf.getShort(2);
        int z = buf.getShort(4);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        double rotX = x * 2000.0 / 32768.0;
        double rotY = y * 2000.0 / 32768.0;
        double rotZ = z * 2000.0 / 32768.0;
        SmartDashboard.putNumber("rotX", rotX);
        SmartDashboard.putNumber("rotY", rotY);
        SmartDashboard.putNumber("rotZ", rotZ);
    }
}
