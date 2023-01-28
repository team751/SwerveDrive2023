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
        ByteBuffer buf = ByteBuffer.allocate(12);
        sensor.read(0x18, 12, buf);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        double rotX = buf.getInt(0) * 4.0 / 32768.0;
        double rotY = buf.getInt(4) * 4.0 / 32768.0;
        double rotZ = buf.getInt(8) * 4.0 / 32768.0;
        SmartDashboard.putNumber("rotX", rotX);
        SmartDashboard.putNumber("rotY", rotY);
        SmartDashboard.putNumber("rotZ", rotZ);
    }
}
