package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class Sensors {
    public Encoder leftEncoder = new Encoder(
        new DigitalInput(ChannelMap.leftMotorEncoderChannelA),
        new DigitalInput(ChannelMap.leftMotorEncoderChannelB)
    );
    public Encoder rightEncoder = new Encoder(
        new DigitalInput(ChannelMap.rightMotorEncoderChannelA),
        new DigitalInput(ChannelMap.rightMotorEncoderChannelB)
    );

    public Sensors() {
        // 360CPR
        // 8.25" diameter wheel
        // 8.25" * pi = 
        leftEncoder.setDistancePerPulse(8.25 * Math.PI / 360);
        rightEncoder.setDistancePerPulse(8.25 * Math.PI / 360);
    }
}
