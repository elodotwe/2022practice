package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChannelMap;

public class Drivetrain extends SubsystemBase {
  public final MotorController leftMotor = new Spark(ChannelMap.leftMotorPWMChannel);
  public final MotorController rightMotor = new Spark(ChannelMap.rightMotorPWMChannel);

  public final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  public final Encoder leftEncoder = new Encoder(
      new DigitalInput(ChannelMap.leftEncoderA),
      new DigitalInput(ChannelMap.leftEncoderB));
  
  public final Encoder rightEncoder = new Encoder(
      new DigitalInput(ChannelMap.rightEncoderA),
      new DigitalInput(ChannelMap.rightEncoderB));

  public final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public final Field2d field2d = new Field2d();
  public final DifferentialDriveOdometry odometry;
  public final DifferentialDriveKinematics kinematics;
  public final SimpleMotorFeedforward motorFeedforward;

  public final double trackWidthMeters = 0.69;
  public final double ksVolts = 0.22;
  public final double kvVoltSecondsPerMeter = 1.98;
  public final double kaVoltSecondsSquaredPerMeter = 0.2;
  public final double kPDriveVel = 8.5;

  public Drivetrain() {
    super();
    // On practice bot, with red to + on motor controller, -1 points "forward" on the
    // right, whereas 1 points "forward" on the left. Flip the right motor to make
    // 1 = forward universally.
    rightMotor.setInverted(true);

    // 360 pulses per revolution on the encoders we have installed on the practice bot
    // 8.25" diameter wheel (0.20955 meters), directly driven on the same shaft as the encoder
    // Distance per pulse = distance covered per revolution / pulses per revolution
    // Distance covered per revolution = wheel circumference = diameter * PI
    double distancePerPulse = 0.20955 * Math.PI / 360;
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);

    leftEncoder.reset();
    rightEncoder.reset();

    odometry = new DifferentialDriveOdometry(new Rotation2d());

    SmartDashboard.putData("Field", field2d);

    kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    motorFeedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
  }

  public void periodic() {
    var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
    var pose = odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance());
    field2d.setRobotPose(pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
    drive.feed();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }
}
