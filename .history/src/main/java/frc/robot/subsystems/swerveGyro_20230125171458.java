package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import edu.wpi.first.wpilibj.I2C;

public class swerveGyro{
  public final I2C sensor;
  public SwerveGyro(){
    sensor= new I2C(I2C.Port.kOnboard, 0x3b);
  }
}
