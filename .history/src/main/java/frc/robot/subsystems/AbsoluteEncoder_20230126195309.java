package frc.robot.subsystems;

import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.DigitalInput;

public class AbsoluteEncoder {
    private AnalogInput encoder;

    public AbsoluteEncoder(int DIOPort) {
        encoder = new AnalogInput(DIOPort);
    }

    public void displayInput(){
        encoder
    }
}
