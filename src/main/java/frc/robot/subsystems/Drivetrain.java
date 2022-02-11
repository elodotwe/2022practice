package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChannelMap;

public class Drivetrain extends SubsystemBase {
  private Spark leftMotor = new Spark(ChannelMap.leftMotorPWMChannel);
  private Spark rightMotor = new Spark(ChannelMap.rightMotorPWMChannel);

  public DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
}
