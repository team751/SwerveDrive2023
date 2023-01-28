package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class AbsoluteEncoder {
    private DigitalInput encoder;

    public AbsoluteEncoder(int DIOPort) {
        encoder = new DigitalInput(DIOPort)
    }
}
