package frc.robot.commands;

import frc.robot.Joysticks;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Teleop extends CommandBase {
  private Drivetrain drivetrain;
  private Joysticks joysticks;

  public Teleop(Drivetrain drivetrain, Joysticks joysticks) {
    this.drivetrain = drivetrain;
    this.joysticks = joysticks;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive.arcadeDrive(joysticks.getDriveForewardPower(), joysticks.getDriveRotationPower());
  }
}
