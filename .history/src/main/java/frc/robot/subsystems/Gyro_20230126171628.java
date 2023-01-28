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
        sensor.write(0x22, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
        sensor.write(0x20, 0x70); // 119 Hz, 4g
    }

    public void getAccel() {
        ByteBuffer buf = ByteBuffer.allocate(4);
        sensor.read(0x18, 4, buf);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        SmartDashboard.putNumber("Accelerometer", buf.getInt(0));
    }
}
