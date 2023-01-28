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
    }

    public void reset() {
        sensor.write(0x22, 0x05);
        sensor.write(0x21, 0x0c);
    }

    public void getAccel() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        sensor.read(0x28, 2, buf);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        SmartDashboard.putNumber("Accelerometer", buf.getShort(0));
    }
}
