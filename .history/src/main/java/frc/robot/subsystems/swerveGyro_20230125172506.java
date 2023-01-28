package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveGyro {
  public final I2C sensor;

  public SwerveGyro() {
    sensor = new I2C(I2C.Port.kOnboard, 0x3b);

  }

  public void getAccel(){
    SmartDashboard.putNumber("some gyro thing", sensor.readWordRegister(0x28))
  }
}
