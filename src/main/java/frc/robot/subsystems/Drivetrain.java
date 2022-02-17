package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChannelMap;

public class Drivetrain extends SubsystemBase {
  private Spark leftMotor = new Spark(ChannelMap.leftMotorPWMChannel);
  private Spark rightMotor = new Spark(ChannelMap.rightMotorPWMChannel);

  public final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  public final Encoder leftEncoder = new Encoder(
      new DigitalInput(ChannelMap.leftEncoderA),
      new DigitalInput(ChannelMap.leftEncoderB));
  
  public final Encoder rightEncoder = new Encoder(
      new DigitalInput(ChannelMap.rightEncoderA),
      new DigitalInput(ChannelMap.rightEncoderB));

  public Drivetrain() {
    super();
    // On practice bot, with red to + on motor controller, -1 points "forward" on the
    // right, whereas 1 points "forward" on the left. Flip the right motor to make
    // 1 = forward universally.
    rightMotor.setInverted(true);

    // 360 pulses per revolution on the encoders we have installed on the practice bot
    // 8.25" diameter wheel, directly driven on the same shaft as the encoder
    // Distance per pulse = distance covered per revolution / pulses per revolution
    // Distance covered per revolution = wheel circumference = diameter * PI
    double distancePerPulse = 8.25 * Math.PI / 360;
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);
  }
}
