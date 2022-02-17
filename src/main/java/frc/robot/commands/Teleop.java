package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Teleop extends CommandBase {
  private Drivetrain drivetrain;
  private Supplier<Double> xSpeed;
  private Supplier<Double> zRotation;

  public Teleop(Drivetrain drivetrain, Supplier<Double> xSpeed, Supplier<Double> zRotation) {
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive.arcadeDrive(xSpeed.get(), zRotation.get());
  }
}
